#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/property.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/list.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "heimann_uapi.h"
#include "heimann_regs.h"

#ifndef PTAT_VDD_SWITCH
#define PTAT_VDD_SWITCH 1
#endif

#define HEIMANN_RAW_FRAME_SIZE  ((2 * NUMBER_OF_BLOCKS + 2) * BLOCK_LENGTH)
#define HEIMANN_WIDTH           PIXEL_PER_ROW
#define HEIMANN_HEIGHT          PIXEL_PER_COLUMN
#define V4L2_PIX_FMT_HTPA       v4l2_fourcc('H', 'T', 'P', 'A')

struct heimann_buffer {
    struct vb2_v4l2_buffer vb;
    struct list_head list;
};

struct heimann_data {
    struct i2c_client *client;          /* sensor, e.g. 0x1a */
    struct i2c_client *eeprom_client;   /* EEPROM, e.g. 0x50 */

    u8 eeprom_addr;

    /* Protect I2C transfers */
    struct mutex lock;

    /* Protect ramoutput during read() capture + copy_to_user() */
    struct mutex frame_lock;

    /* Basic EEPROM calibration values used to initialize the sensor */
    u32 id;
    u8 mbit_calib;
    u8 bias_calib;
    u8 clk_calib;
    u8 bpa_calib;
    u8 pu_calib;

    /* Raw frame buffer returned to userspace by read() */
    u8 ramoutput[2 * NUMBER_OF_BLOCKS + 2][BLOCK_LENGTH];

    /* Full EEPROM cache returned to userspace by ioctl() */
    u8 *eeprom_cache;
    bool eeprom_cache_valid;

    u8 ptat_vdd_switch;

    /* Legacy control node. Its read() path is retained as a rollback path. */
    struct miscdevice miscdev;

    /* V4L2 capture node and videobuf2 queue. */
    struct v4l2_device v4l2_dev;
    struct video_device video_dev;
    struct vb2_queue vb2_queue;
    struct mutex vb2_lock;
    spinlock_t queued_lock;
    struct list_head queued_buffers;
    wait_queue_head_t capture_wait;
    struct task_struct *capture_thread;
    bool streaming;
    bool capture_abort;
    u32 sequence;
    unsigned int queued_count;
};

static int heimann_write_sensor_byte(struct heimann_data *data,
                                     u8 reg, u8 val)
{
    struct i2c_client *client = data->client;
    u8 buf[2];
    int ret;

    buf[0] = reg;
    buf[1] = val;

    mutex_lock(&data->lock);
    ret = i2c_master_send(client, buf, 2);
    mutex_unlock(&data->lock);

    if (ret < 0) {
        dev_err(&client->dev,
                "write sensor reg 0x%02x failed, ret=%d\n",
                reg, ret);
        return ret;
    }

    if (ret != 2) {
        dev_err(&client->dev,
                "write sensor reg 0x%02x short write, ret=%d\n",
                reg, ret);
        return -EIO;
    }

    return 0;
}

static int heimann_read_sensor_reg(struct heimann_data *data,
                                   u8 reg, u8 *buf, u16 len)
{
    struct i2c_client *client = data->client;
    struct i2c_msg msgs[2];
    int ret;

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = buf;

    mutex_lock(&data->lock);
    ret = i2c_transfer(client->adapter, msgs, 2);
    mutex_unlock(&data->lock);

    if (ret < 0) {
        dev_err(&client->dev,
                "read sensor reg 0x%02x failed, ret=%d\n",
                reg, ret);
        return ret;
    }

    if (ret != 2) {
        dev_err(&client->dev,
                "read sensor reg 0x%02x incomplete, ret=%d\n",
                reg, ret);
        return -EIO;
    }

    return 0;
}

static int heimann_read_eeprom_byte(struct heimann_data *data,
                                    u16 mem_reg, u8 *val)
{
    struct i2c_client *client = data->eeprom_client;
    struct i2c_msg msgs[2];
    u8 addr_buf[2];
    int ret;

    addr_buf[0] = (u8)(mem_reg >> 8);
    addr_buf[1] = (u8)(mem_reg & 0xff);

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = addr_buf;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = val;

    mutex_lock(&data->lock);
    ret = i2c_transfer(client->adapter, msgs, 2);
    mutex_unlock(&data->lock);

    if (ret < 0) {
        dev_err(&data->client->dev,
                "read eeprom addr 0x%04x failed, ret=%d\n",
                mem_reg, ret);
        return ret;
    }

    if (ret != 2) {
        dev_err(&data->client->dev,
                "read eeprom addr 0x%04x incomplete, ret=%d\n",
                mem_reg, ret);
        return -EIO;
    }

    return 0;
}

static int heimann_read_eeprom_block(struct heimann_data *data,
                                     u16 mem_reg, u8 *buf, u16 len)
{
    struct i2c_client *client = data->eeprom_client;
    struct i2c_msg msgs[2];
    u8 addr_buf[2];
    int ret;

    addr_buf[0] = (u8)(mem_reg >> 8);
    addr_buf[1] = (u8)(mem_reg & 0xff);

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = addr_buf;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = buf;

    mutex_lock(&data->lock);
    ret = i2c_transfer(client->adapter, msgs, 2);
    mutex_unlock(&data->lock);

    if (ret < 0) {
        dev_err(&data->client->dev,
                "read eeprom block addr 0x%04x len %u failed, ret=%d\n",
                mem_reg, len, ret);
        return ret;
    }

    if (ret != 2) {
        dev_err(&data->client->dev,
                "read eeprom block addr 0x%04x len %u incomplete, ret=%d\n",
                mem_reg, len, ret);
        return -EIO;
    }

    return 0;
}

static int heimann_cache_full_eeprom(struct heimann_data *data)
{
    u16 addr;
    int ret;
    const u16 chunk = 32;

    if (!data->eeprom_cache)
        return -ENOMEM;

    for (addr = 0; addr < HEIMANN_EEPROM_SIZE; addr += chunk) {
        u16 len = chunk;

        if (addr + len > HEIMANN_EEPROM_SIZE)
            len = HEIMANN_EEPROM_SIZE - addr;

        ret = heimann_read_eeprom_block(data, addr,
                                        &data->eeprom_cache[addr], len);
        if (ret)
            return ret;
    }

    data->eeprom_cache_valid = true;

    dev_info(&data->client->dev,
             "full EEPROM cached: size=%u, first=%02x %02x %02x %02x\n",
             HEIMANN_EEPROM_SIZE,
             data->eeprom_cache[0],
             data->eeprom_cache[1],
             data->eeprom_cache[2],
             data->eeprom_cache[3]);

    return 0;
}

static int heimann_read_eeprom_basic(struct heimann_data *data)
{
    int ret;
    u8 v;

    data->id = 0;

    ret = heimann_read_eeprom_byte(data, E_ID1, &v);
    if (ret)
        return ret;
    data->id |= ((u32)v);

    ret = heimann_read_eeprom_byte(data, E_ID2, &v);
    if (ret)
        return ret;
    data->id |= ((u32)v << 8);

    ret = heimann_read_eeprom_byte(data, E_ID3, &v);
    if (ret)
        return ret;
    data->id |= ((u32)v << 16);

    ret = heimann_read_eeprom_byte(data, E_ID4, &v);
    if (ret)
        return ret;
    data->id |= ((u32)v << 24);

    ret = heimann_read_eeprom_byte(data, E_MBIT_CALIB, &data->mbit_calib);
    if (ret)
        return ret;

    ret = heimann_read_eeprom_byte(data, E_BIAS_CALIB, &data->bias_calib);
    if (ret)
        return ret;

    ret = heimann_read_eeprom_byte(data, E_CLK_CALIB, &data->clk_calib);
    if (ret)
        return ret;

    ret = heimann_read_eeprom_byte(data, E_BPA_CALIB, &data->bpa_calib);
    if (ret)
        return ret;

    ret = heimann_read_eeprom_byte(data, E_PU_CALIB, &data->pu_calib);
    if (ret)
        return ret;

    dev_info(&data->client->dev,
             "EEPROM basic: id=0x%08x, mbit=0x%02x, bias=0x%02x, clk=0x%02x, bpa=0x%02x, pu=0x%02x\n",
             data->id,
             data->mbit_calib,
             data->bias_calib,
             data->clk_calib,
             data->bpa_calib,
             data->pu_calib);

    return 0;
}

static int heimann_write_calibration_settings(struct heimann_data *data)
{
    int ret;

    ret = heimann_write_sensor_byte(data, TRIM_REGISTER1, data->mbit_calib);
    if (ret)
        return ret;
    msleep(5);

    ret = heimann_write_sensor_byte(data, TRIM_REGISTER2, data->bias_calib);
    if (ret)
        return ret;
    msleep(5);

    ret = heimann_write_sensor_byte(data, TRIM_REGISTER3, data->bias_calib);
    if (ret)
        return ret;
    msleep(5);

    ret = heimann_write_sensor_byte(data, TRIM_REGISTER4, data->clk_calib);
    if (ret)
        return ret;
    msleep(5);

    ret = heimann_write_sensor_byte(data, TRIM_REGISTER5, data->bpa_calib);
    if (ret)
        return ret;
    msleep(5);

    ret = heimann_write_sensor_byte(data, TRIM_REGISTER6, data->bpa_calib);
    if (ret)
        return ret;
    msleep(5);

    ret = heimann_write_sensor_byte(data, TRIM_REGISTER7, data->pu_calib);
    if (ret)
        return ret;
    msleep(5);

    dev_info(&data->client->dev, "trim calibration registers written\n");

    return 0;
}

static int heimann_hw_init_minimal(struct heimann_data *data)
{
    int ret;

    ret = heimann_read_eeprom_basic(data);
    if (ret) {
        dev_err(&data->client->dev,
                "failed to read EEPROM basic calibration, ret=%d\n",
                ret);
        return ret;
    }

    ret = heimann_write_sensor_byte(data, CONFIGURATION_REGISTER, 0x01);
    if (ret) {
        dev_err(&data->client->dev,
                "failed to write CONFIGURATION_REGISTER 0x01, ret=%d\n",
                ret);
        return ret;
    }

    msleep(10);

    ret = heimann_write_calibration_settings(data);
    if (ret) {
        dev_err(&data->client->dev,
                "failed to write calibration settings, ret=%d\n",
                ret);
        return ret;
    }

    ret = heimann_write_sensor_byte(data, CONFIGURATION_REGISTER, 0x09);
    if (ret) {
        dev_err(&data->client->dev,
                "failed to write CONFIGURATION_REGISTER 0x09, ret=%d\n",
                ret);
        return ret;
    }

    msleep(10);

    dev_info(&data->client->dev, "minimal hardware init success\n");

    return 0;
}

static int heimann_wait_eoc(struct heimann_data *data)
{
    u8 status = 0;
    int ret;
    int i;

    for (i = 0; i < 100; i++) {
        if (READ_ONCE(data->capture_abort))
            return -ECANCELED;

        ret = heimann_read_sensor_reg(data, STATUS_REGISTER, &status, 1);
        if (ret) {
            dev_err(&data->client->dev,
                    "read STATUS_REGISTER failed, ret=%d\n",
                    ret);
            return ret;
        }

        if (status & 0x01)
            return 0;

        if (msleep_interruptible(10) && READ_ONCE(data->capture_abort))
            return -ECANCELED;
    }

    dev_err(&data->client->dev,
            "wait EOC timeout, last status=0x%02x\n",
            status);

    return -ETIMEDOUT;
}

static int heimann_read_one_block_pair(struct heimann_data *data,
                                       u8 block_num)
{
    u8 bottomblock;
    int ret;

    if (block_num >= NUMBER_OF_BLOCKS)
        return -EINVAL;

    ret = heimann_wait_eoc(data);
    if (ret)
        return ret;

    ret = heimann_read_sensor_reg(data,
                                  TOP_HALF,
                                  data->ramoutput[block_num],
                                  BLOCK_LENGTH);
    if (ret) {
        dev_err(&data->client->dev,
                "read TOP_HALF block %u failed, ret=%d\n",
                block_num, ret);
        return ret;
    }

    bottomblock = (u8)((NUMBER_OF_BLOCKS + 1) * 2 - block_num - 1);

    ret = heimann_read_sensor_reg(data,
                                  BOTTOM_HALF,
                                  data->ramoutput[bottomblock],
                                  BLOCK_LENGTH);
    if (ret) {
        dev_err(&data->client->dev,
                "read BOTTOM_HALF block %u failed, ret=%d\n",
                block_num, ret);
        return ret;
    }

    return 0;
}

static int heimann_read_electrical_offset_pair(struct heimann_data *data)
{
    int ret;

    /*
     * BLIND / electrical offset 采集命令。
     *
     * 原应用层逻辑：
     *     0x0B + 0x04 * switch_ptat_vdd
     */
    ret = heimann_write_sensor_byte(data,
                                    CONFIGURATION_REGISTER,
                                    (u8)(0x0B + 0x04 * data->ptat_vdd_switch));
    if (ret)
        return ret;

    ret = heimann_wait_eoc(data);
    if (ret)
        return ret;

    /*
     * RAMoutput[4] = electrical offset top
     */
    ret = heimann_read_sensor_reg(data,
                                  TOP_HALF,
                                  data->ramoutput[NUMBER_OF_BLOCKS],
                                  BLOCK_LENGTH);
    if (ret) {
        dev_err(&data->client->dev,
                "read electrical offset TOP failed, ret=%d\n",
                ret);
        return ret;
    }

    /*
     * RAMoutput[5] = electrical offset bottom
     */
    ret = heimann_read_sensor_reg(data,
                                  BOTTOM_HALF,
                                  data->ramoutput[NUMBER_OF_BLOCKS + 1],
                                  BLOCK_LENGTH);
    if (ret) {
        dev_err(&data->client->dev,
                "read electrical offset BOTTOM failed, ret=%d\n",
                ret);
        return ret;
    }

    return 0;
}

static int heimann_capture_one_frame(struct heimann_data *data)
{
    int ret;
    u8 block;
    u8 sw;

    sw = data->ptat_vdd_switch;

    /*
     * 启动 block0。
     * 原来固定 0x09，现在加上 0x04 * sw。
     */
    ret = heimann_write_sensor_byte(data,
                                    CONFIGURATION_REGISTER,
                                    (u8)(0x09 + 0x04 * sw));
    if (ret)
        return ret;

    for (block = 0; block < NUMBER_OF_BLOCKS; block++) {
        ret = heimann_read_one_block_pair(data, block);
        if (ret)
            return ret;

        if (block + 1 < NUMBER_OF_BLOCKS) {
            ret = heimann_write_sensor_byte(data,
                                            CONFIGURATION_REGISTER,
                                            (u8)(0x09 +
                                                 0x10 * (block + 1) +
                                                 0x04 * sw));
            if (ret)
                return ret;
        }
    }

    /*
     * 补采 electrical offset，填 RAMoutput[4] 和 RAMoutput[5]。
     * 先每帧都采，保证应用层补偿数据一定有效。
     * 后面稳定后可以改成每 N 帧采一次。
     */
    ret = heimann_read_electrical_offset_pair(data);
    if (ret)
        return ret;

    /*
     * 准备下一帧：PTAT/VDD 交替。
     */
#if PTAT_VDD_SWITCH
    data->ptat_vdd_switch ^= 1;
#endif

    /*
     * 重新启动下一帧 block0。
     */
    ret = heimann_write_sensor_byte(data,
                                    CONFIGURATION_REGISTER,
                                    (u8)(0x09 + 0x04 * data->ptat_vdd_switch));
    if (ret)
        return ret;

    return 0;
}

static void heimann_return_all_buffers(struct heimann_data *data,
                                       enum vb2_buffer_state state)
{
    struct heimann_buffer *buffer;
    unsigned long flags;

    for (;;) {
        spin_lock_irqsave(&data->queued_lock, flags);
        if (list_empty(&data->queued_buffers)) {
            spin_unlock_irqrestore(&data->queued_lock, flags);
            break;
        }

        buffer = list_first_entry(&data->queued_buffers,
                                  struct heimann_buffer, list);
        list_del(&buffer->list);
        data->queued_count--;
        spin_unlock_irqrestore(&data->queued_lock, flags);

        vb2_buffer_done(&buffer->vb.vb2_buf, state);
    }
}

static int heimann_capture_thread(void *arg)
{
    struct heimann_data *data = arg;

    while (!kthread_should_stop()) {
        struct heimann_buffer *buffer = NULL;
        unsigned long flags;
        void *destination;
        u64 timestamp_ns;
        int ret;

        ret = wait_event_interruptible(
            data->capture_wait,
            kthread_should_stop() ||
            (READ_ONCE(data->streaming) && READ_ONCE(data->queued_count)));
        if (kthread_should_stop())
            break;
        if (ret)
            continue;

        spin_lock_irqsave(&data->queued_lock, flags);
        if (!list_empty(&data->queued_buffers)) {
            buffer = list_first_entry(&data->queued_buffers,
                                      struct heimann_buffer, list);
            list_del(&buffer->list);
            data->queued_count--;
        }
        spin_unlock_irqrestore(&data->queued_lock, flags);

        if (!buffer)
            continue;

        if (!READ_ONCE(data->streaming)) {
            vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_ERROR);
            continue;
        }

        destination = vb2_plane_vaddr(&buffer->vb.vb2_buf, 0);
        if (!destination) {
            vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_ERROR);
            continue;
        }

        timestamp_ns = ktime_get_ns();
        mutex_lock(&data->frame_lock);
        ret = heimann_capture_one_frame(data);
        if (!ret)
            memcpy(destination, data->ramoutput, HEIMANN_RAW_FRAME_SIZE);
        mutex_unlock(&data->frame_lock);

        if (ret) {
            if (ret != -ECANCELED)
                dev_err_ratelimited(&data->client->dev,
                                    "V4L2 frame capture failed, ret=%d\n", ret);
            vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_ERROR);
            continue;
        }

        buffer->vb.sequence = data->sequence++;
        buffer->vb.field = V4L2_FIELD_NONE;
        buffer->vb.vb2_buf.timestamp = timestamp_ns;
        vb2_set_plane_payload(&buffer->vb.vb2_buf, 0,
                              HEIMANN_RAW_FRAME_SIZE);
        vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_DONE);
    }

    return 0;
}

static int heimann_queue_setup(struct vb2_queue *queue,
                               unsigned int *num_buffers,
                               unsigned int *num_planes,
                               unsigned int sizes[],
                               struct device *alloc_devs[])
{
    if (*num_planes) {
        if (*num_planes != 1 || sizes[0] < HEIMANN_RAW_FRAME_SIZE)
            return -EINVAL;
        return 0;
    }

    *num_planes = 1;
    sizes[0] = HEIMANN_RAW_FRAME_SIZE;
    if (*num_buffers < 3)
        *num_buffers = 3;

    return 0;
}

static int heimann_buf_prepare(struct vb2_buffer *vb)
{
    if (vb2_plane_size(vb, 0) < HEIMANN_RAW_FRAME_SIZE)
        return -EINVAL;

    vb2_set_plane_payload(vb, 0, HEIMANN_RAW_FRAME_SIZE);
    return 0;
}

static void heimann_buf_queue(struct vb2_buffer *vb)
{
    struct heimann_data *data = vb2_get_drv_priv(vb->vb2_queue);
    struct heimann_buffer *buffer =
        container_of(to_vb2_v4l2_buffer(vb), struct heimann_buffer, vb);
    unsigned long flags;

    spin_lock_irqsave(&data->queued_lock, flags);
    list_add_tail(&buffer->list, &data->queued_buffers);
    data->queued_count++;
    spin_unlock_irqrestore(&data->queued_lock, flags);

    wake_up_interruptible(&data->capture_wait);
}

static int heimann_start_streaming(struct vb2_queue *queue,
                                   unsigned int count)
{
    struct heimann_data *data = vb2_get_drv_priv(queue);
    struct task_struct *thread;

    mutex_lock(&data->frame_lock);
    WRITE_ONCE(data->capture_abort, false);
    WRITE_ONCE(data->streaming, true);
    data->ptat_vdd_switch = 0;
    data->sequence = 0;
    mutex_unlock(&data->frame_lock);

    thread = kthread_run(heimann_capture_thread, data, "heimann-v4l2");
    if (IS_ERR(thread)) {
        int ret = PTR_ERR(thread);

        WRITE_ONCE(data->streaming, false);
        WRITE_ONCE(data->capture_abort, true);
        heimann_return_all_buffers(data, VB2_BUF_STATE_QUEUED);
        return ret;
    }

    data->capture_thread = thread;
    return 0;
}

static void heimann_stop_streaming(struct vb2_queue *queue)
{
    struct heimann_data *data = vb2_get_drv_priv(queue);

    WRITE_ONCE(data->capture_abort, true);
    WRITE_ONCE(data->streaming, false);
    wake_up_interruptible(&data->capture_wait);

    if (data->capture_thread) {
        kthread_stop(data->capture_thread);
        data->capture_thread = NULL;
    }

    heimann_return_all_buffers(data, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops heimann_vb2_ops = {
    .queue_setup = heimann_queue_setup,
    .buf_prepare = heimann_buf_prepare,
    .buf_queue = heimann_buf_queue,
    .start_streaming = heimann_start_streaming,
    .stop_streaming = heimann_stop_streaming,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
};

static void heimann_fill_format(struct v4l2_pix_format *pix)
{
    pix->width = HEIMANN_WIDTH;
    pix->height = HEIMANN_HEIGHT;
    pix->pixelformat = V4L2_PIX_FMT_HTPA;
    pix->field = V4L2_FIELD_NONE;
    pix->bytesperline = 0;
    pix->sizeimage = HEIMANN_RAW_FRAME_SIZE;
    pix->colorspace = V4L2_COLORSPACE_RAW;
    pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
    pix->quantization = V4L2_QUANTIZATION_DEFAULT;
    pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int heimann_vidioc_querycap(struct file *file, void *priv,
                                   struct v4l2_capability *cap)
{
    struct heimann_data *data = video_drvdata(file);

    strscpy(cap->driver, "heimann_v4l2", sizeof(cap->driver));
    strscpy(cap->card, "Heimann HTPA 32x32", sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "I2C:%s",
             dev_name(&data->client->dev));
    return 0;
}

static int heimann_vidioc_enum_fmt(struct file *file, void *priv,
                                   struct v4l2_fmtdesc *format)
{
    if (format->index != 0)
        return -EINVAL;

    format->pixelformat = V4L2_PIX_FMT_HTPA;
    strscpy(format->description, "Heimann raw frame",
            sizeof(format->description));
    return 0;
}

static int heimann_vidioc_g_fmt(struct file *file, void *priv,
                                struct v4l2_format *format)
{
    heimann_fill_format(&format->fmt.pix);
    return 0;
}

static int heimann_vidioc_try_fmt(struct file *file, void *priv,
                                  struct v4l2_format *format)
{
    heimann_fill_format(&format->fmt.pix);
    return 0;
}

static int heimann_vidioc_s_fmt(struct file *file, void *priv,
                                struct v4l2_format *format)
{
    struct heimann_data *data = video_drvdata(file);

    if (vb2_is_busy(&data->vb2_queue))
        return -EBUSY;

    heimann_fill_format(&format->fmt.pix);
    return 0;
}

static int heimann_vidioc_enum_input(struct file *file, void *priv,
                                     struct v4l2_input *input)
{
    if (input->index != 0)
        return -EINVAL;

    input->type = V4L2_INPUT_TYPE_CAMERA;
    strscpy(input->name, "Heimann HTPA", sizeof(input->name));
    return 0;
}

static int heimann_vidioc_g_input(struct file *file, void *priv,
                                  unsigned int *input)
{
    *input = 0;
    return 0;
}

static int heimann_vidioc_s_input(struct file *file, void *priv,
                                  unsigned int input)
{
    return input == 0 ? 0 : -EINVAL;
}

static const struct v4l2_ioctl_ops heimann_ioctl_ops = {
    .vidioc_querycap = heimann_vidioc_querycap,
    .vidioc_enum_fmt_vid_cap = heimann_vidioc_enum_fmt,
    .vidioc_g_fmt_vid_cap = heimann_vidioc_g_fmt,
    .vidioc_try_fmt_vid_cap = heimann_vidioc_try_fmt,
    .vidioc_s_fmt_vid_cap = heimann_vidioc_s_fmt,
    .vidioc_enum_input = heimann_vidioc_enum_input,
    .vidioc_g_input = heimann_vidioc_g_input,
    .vidioc_s_input = heimann_vidioc_s_input,
    .vidioc_reqbufs = vb2_ioctl_reqbufs,
    .vidioc_create_bufs = vb2_ioctl_create_bufs,
    .vidioc_prepare_buf = vb2_ioctl_prepare_buf,
    .vidioc_querybuf = vb2_ioctl_querybuf,
    .vidioc_qbuf = vb2_ioctl_qbuf,
    .vidioc_dqbuf = vb2_ioctl_dqbuf,
    .vidioc_streamon = vb2_ioctl_streamon,
    .vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations heimann_v4l2_fops = {
    .owner = THIS_MODULE,
    .open = v4l2_fh_open,
    .release = vb2_fop_release,
    .read = vb2_fop_read,
    .poll = vb2_fop_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap = vb2_fop_mmap,
};

static int heimann_register_video_device(struct heimann_data *data)
{
    struct vb2_queue *queue = &data->vb2_queue;
    struct video_device *video = &data->video_dev;
    int ret;

    ret = v4l2_device_register(&data->client->dev, &data->v4l2_dev);
    if (ret)
        return ret;

    queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    queue->io_modes = VB2_MMAP | VB2_READ;
    queue->drv_priv = data;
    queue->buf_struct_size = sizeof(struct heimann_buffer);
    queue->ops = &heimann_vb2_ops;
    queue->mem_ops = &vb2_vmalloc_memops;
    queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    queue->min_buffers_needed = 2;
    queue->lock = &data->vb2_lock;
    queue->dev = &data->client->dev;

    ret = vb2_queue_init(queue);
    if (ret)
        goto err_unregister_v4l2;

    strscpy(video->name, "heimann-htpa", sizeof(video->name));
    video->v4l2_dev = &data->v4l2_dev;
    video->fops = &heimann_v4l2_fops;
    video->ioctl_ops = &heimann_ioctl_ops;
    video->release = video_device_release_empty;
    video->lock = &data->vb2_lock;
    video->queue = queue;
    video->device_caps = V4L2_CAP_VIDEO_CAPTURE |
                         V4L2_CAP_STREAMING |
                         V4L2_CAP_READWRITE;
    video_set_drvdata(video, data);

    ret = video_register_device(video, VFL_TYPE_VIDEO, -1);
    if (ret)
        goto err_unregister_v4l2;

    dev_info(&data->client->dev,
             "registered /dev/video%d: HTPA 32x32, payload=%d bytes\n",
             video->num, HEIMANN_RAW_FRAME_SIZE);
    return 0;

err_unregister_v4l2:
    v4l2_device_unregister(&data->v4l2_dev);
    return ret;
}

static int heimann_open(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev = file->private_data;
    struct heimann_data *data;

    data = container_of(miscdev, struct heimann_data, miscdev);

    mutex_lock(&data->frame_lock);
    if (!READ_ONCE(data->streaming))
        data->ptat_vdd_switch = 0;
    mutex_unlock(&data->frame_lock);

    file->private_data = data;

    return nonseekable_open(inode, file);
}

static ssize_t heimann_read(struct file *file,
                            char __user *buf,
                            size_t count,
                            loff_t *ppos)
{
    struct heimann_data *data = file->private_data;
    size_t frame_size = HEIMANN_RAW_FRAME_SIZE;
    int ret;

    if (!data)
        return -ENODEV;

    if (count < frame_size) {
        dev_err(&data->client->dev,
                "user buffer too small: count=%zu, need=%zu\n",
                count, frame_size);
        return -EINVAL;
    }

    mutex_lock(&data->frame_lock);

    if (READ_ONCE(data->streaming)) {
        mutex_unlock(&data->frame_lock);
        return -EBUSY;
    }

    WRITE_ONCE(data->capture_abort, false);

    ret = heimann_capture_one_frame(data);
    if (ret) {
        mutex_unlock(&data->frame_lock);
        dev_err(&data->client->dev,
                "capture one frame failed, ret=%d\n",
                ret);
        return ret;
    }

    if (copy_to_user(buf, data->ramoutput, frame_size)) {
        mutex_unlock(&data->frame_lock);
        dev_err(&data->client->dev, "copy_to_user failed\n");
        return -EFAULT;
    }

    mutex_unlock(&data->frame_lock);

    return frame_size;
}

static long heimann_ioctl(struct file *file,
                          unsigned int cmd,
                          unsigned long arg)
{
    struct heimann_data *data = file->private_data;
    struct heimann_eeprom_dump __user *udump;
    __u32 size = HEIMANN_EEPROM_SIZE;

    if (!data)
        return -ENODEV;

    switch (cmd) {
    case HEIMANN_IOC_GET_EEPROM:
        if (!data->eeprom_cache_valid || !data->eeprom_cache)
            return -EIO;

        udump = (struct heimann_eeprom_dump __user *)arg;

        if (copy_to_user(&udump->size, &size, sizeof(size)))
            return -EFAULT;

        if (copy_to_user(udump->data,
                         data->eeprom_cache,
                         HEIMANN_EEPROM_SIZE))
            return -EFAULT;

        return 0;

    default:
        return -ENOTTY;
    }
}


static const struct file_operations heimann_fops = {
    .owner = THIS_MODULE,
    .open = heimann_open,
    .read = heimann_read,
    .unlocked_ioctl = heimann_ioctl,
    .llseek = no_llseek,
};

static int heimann_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    struct heimann_data *data;
    u32 eeprom_addr;
    u8 test_val;
    int ret;

    dev_info(&client->dev, "========================================\n");
    dev_info(&client->dev, "Heimann probe entered\n");
    dev_info(&client->dev, "I2C adapter number = %d\n", client->adapter->nr);
    dev_info(&client->dev, "sensor i2c addr = 0x%02x\n", client->addr);

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->ptat_vdd_switch = 0;
    data->client = client;
    mutex_init(&data->lock);
    mutex_init(&data->frame_lock);
    mutex_init(&data->vb2_lock);
    spin_lock_init(&data->queued_lock);
    INIT_LIST_HEAD(&data->queued_buffers);
    init_waitqueue_head(&data->capture_wait);
    i2c_set_clientdata(client, data);

    ret = device_property_read_u32(&client->dev,
                                   "eeprom-addr",
                                   &eeprom_addr);
    if (ret) {
        dev_err(&client->dev,
                "failed to read eeprom-addr from device tree, ret=%d\n",
                ret);
        return ret;
    }

    if (eeprom_addr > 0x7f) {
        dev_err(&client->dev,
                "invalid eeprom addr 0x%x, should be 7-bit i2c addr\n",
                eeprom_addr);
        return -EINVAL;
    }

    data->eeprom_addr = (u8)eeprom_addr;
    dev_info(&client->dev, "eeprom i2c addr = 0x%02x\n", data->eeprom_addr);

    data->eeprom_client = i2c_new_dummy_device(client->adapter,
                                               data->eeprom_addr);
    if (IS_ERR(data->eeprom_client)) {
        ret = PTR_ERR(data->eeprom_client);
        dev_err(&client->dev,
                "failed to create eeprom client at 0x%02x, ret=%d\n",
                data->eeprom_addr, ret);
        return ret;
    }

    ret = heimann_read_eeprom_byte(data, 0x0000, &test_val);
    if (ret) {
        dev_err(&client->dev,
                "EEPROM test read failed, ret=%d\n",
                ret);
        goto err_unregister_eeprom;
    }

    dev_info(&client->dev,
             "EEPROM test read success: eeprom[0x0000] = 0x%02x\n",
             test_val);

    data->eeprom_cache = devm_kzalloc(&client->dev,
                                      HEIMANN_EEPROM_SIZE,
                                      GFP_KERNEL);
    if (!data->eeprom_cache) {
        ret = -ENOMEM;
        goto err_unregister_eeprom;
    }

    ret = heimann_cache_full_eeprom(data);
    if (ret) {
        dev_err(&client->dev,
                "failed to cache full EEPROM, ret=%d\n",
                ret);
        goto err_unregister_eeprom;
    }

    ret = heimann_hw_init_minimal(data);
    if (ret) {
        dev_err(&client->dev,
                "minimal hardware init failed, ret=%d\n",
                ret);
        goto err_unregister_eeprom;
    }

    data->miscdev.minor = MISC_DYNAMIC_MINOR;
    data->miscdev.name = "heimann0";
    data->miscdev.fops = &heimann_fops;
    data->miscdev.parent = &client->dev;

    ret = misc_register(&data->miscdev);
    if (ret) {
        dev_err(&client->dev,
                "failed to register /dev/heimann0, ret=%d\n",
                ret);
        goto err_unregister_eeprom;
    }

    ret = heimann_register_video_device(data);
    if (ret) {
        dev_err(&client->dev,
                "failed to register V4L2 video device, ret=%d\n", ret);
        goto err_deregister_misc;
    }

    dev_info(&client->dev,
             "/dev/heimann0 control/legacy node registered, raw frame size=%d bytes\n",
             HEIMANN_RAW_FRAME_SIZE);

    dev_info(&client->dev, "Heimann probe success\n");
    dev_info(&client->dev, "========================================\n");

    return 0;

err_deregister_misc:
    misc_deregister(&data->miscdev);
err_unregister_eeprom:
    if (data->eeprom_client)
        i2c_unregister_device(data->eeprom_client);

    return ret;
}

static int heimann_remove(struct i2c_client *client)
{
    struct heimann_data *data = i2c_get_clientdata(client);

    dev_info(&client->dev, "Heimann remove called\n");

    if (data) {
        video_unregister_device(&data->video_dev);
        v4l2_device_unregister(&data->v4l2_dev);
        misc_deregister(&data->miscdev);

        if (data->eeprom_client)
            i2c_unregister_device(data->eeprom_client);
    }

    return 0;
}

static const struct of_device_id heimann_of_match[] = {
    { .compatible = "heimann,htpa" },
    { }
};
MODULE_DEVICE_TABLE(of, heimann_of_match);

static const struct i2c_device_id heimann_id[] = {
    { "heimann_htpa", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, heimann_id);

static struct i2c_driver heimann_i2c_driver = {
    .driver = {
        .name = "heimann_htpa",
        .of_match_table = heimann_of_match,
    },
    .probe = heimann_probe,
    .remove = heimann_remove,
    .id_table = heimann_id,
};

module_i2c_driver(heimann_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("yuan yuan");
MODULE_DESCRIPTION("Heimann HTPA I2C V4L2 capture driver with videobuf2 streaming");
