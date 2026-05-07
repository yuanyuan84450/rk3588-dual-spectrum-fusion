// #include <heimann_reg.h>
#include <signal.h>
#include <time.h>
//#include <linux/time.h>
#include <poll.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include "heimann_uapi.h"
#include "heimann_drv.h"
#include "opencv_draw.h"
#include "public_cfg.h"

/*static inline uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}*/

mlx_characteristics_t DevConst = {
    .NumberOfPixel = NUMBER_OF_PIXEL,
    .NumberOfBlocks = NUMBER_OF_BLOCKS,
    .RowPerBlock = ROW_PER_BLOCK,
    .PixelPerBlock = PIXEL_PER_BLOCK,
    .PixelPerColumn = PIXEL_PER_COLUMN,
    .PixelPerRow = PIXEL_PER_ROW,
    .AllowedDeadPix = ALLOWED_DEADPIX,
    .TableNumber = TABLENUMBER,
    .TableOffset = TABLEOFFSET,
    .PTATPos = PTAT_POS,
    .VDDPos = VDD_POS,
    .PTATVDDSwitch = PTAT_VDD_SWITCH,
    .CyclopsActive = ATC_ACTIVE,
    .CyclopsPos = ATC_POS,
    .DataPos = DATA_POS};

//-----------------------------------------
// EEPROM DATA
uint8_t mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, mbit_user, bias_user, clk_user, bpa_user, pu_user;
uint8_t nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon, lastepsilon, arraytype;
uint8_t deadpixmask[ALLOWED_DEADPIX];
int8_t globaloff;
int16_t thgrad[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
uint16_t tablenumber, vddth1, vddth2, ptatth1, ptatth2, ptatgr, globalgain;
uint16_t deadpixadr[ALLOWED_DEADPIX * 2];
int16_t thoffset[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
int16_t vddcompgrad[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
int16_t vddcompoff[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
uint32_t id, ptatoff;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;
uint32_t *pixc2_0; // start address of the allocated heap memory
uint32_t *pixc2;   // increasing address pointer

//-----------------------------------------
// SENSOR DATA
uint16_t data_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
uint16_t *data_pixel_flatten = (uint16_t *)data_pixel;
uint8_t RAMoutput[2 * NUMBER_OF_BLOCKS + 2][BLOCK_LENGTH];
bool flag_min_max_initaled = false; // 需要重新计算极值的标志位
uint8_t x_max, y_max, x_min, y_min;
/*
  RAMoutput is the place where the raw values are saved

  example, order for 80x64:
  RAMoutput[0][]... data from block 0 top
  RAMoutput[1][]... data from block 1 top
  RAMoutput[2][]... data from block 2 top
  RAMoutput[3][]... data from block 3 top
  RAMutput[4][]... electrical offset top
  RAMoutput[5][]... electrical offset bottom
  RAMoutput[6][]... data from block 3 bottom
  RAMoutput[7][]... data from block 2 bottom
  RAMoutput[8][]... data from block 1 bottom
  RAMoutput[9][]... data from block 0 bottom

*/
uint16_t eloffset[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
uint8_t statusreg;
uint16_t Ta, ptat_av_uint16, vdd_av_uint16;

// BUFFER for PTAT,VDD and elOffsets
// PTAT:
uint16_t ptat_buffer[PTAT_BUFFER_SIZE];
uint16_t ptat_buffer_average;
uint8_t use_ptat_buffer = 0;
uint8_t ptat_i = 0;
uint8_t PTATok = 0;
// VDD:
uint16_t vdd_buffer[VDD_BUFFER_SIZE];
uint16_t vdd_buffer_average;
uint8_t use_vdd_buffer = 0;
uint8_t vdd_i = 0;
// electrical offsets:
uint8_t use_eloffsets_buffer = 0;
uint8_t eloffsets_i = 0;
uint8_t new_offsets = 1;

// PROGRAMM CONTROL
bool switch_ptat_vdd = 0;
static uint8_t sampled_switch_ptat_vdd = 0;
uint8_t adr_offset = 0x00;
uint8_t send_data = 0;
uint16_t picnum = 0;
uint8_t state = 0;
uint8_t read_block_num = START_WITH_BLOCK; // start with electrical offset
uint8_t read_eloffset_next_pic = 0;
uint8_t gui_mode = 0;
uint8_t wait_pic = 0;
bool ReadingRoutineEnable = 1;

// OTHER
uint32_t gradscale_div;
uint32_t vddscgrad_div;
uint32_t vddscoff_div;
int32_t vddcompgrad_n;
int32_t vddcompoff_n;
uint32_t t1;
uint8_t print_state = 0;

volatile uint32_t NewDataAvailable = 1;

uint16_t timert;
char serial_input = 'm';

uint8_t prob_status = PROB_CONNECTING; // 探头当前状态

// 线程锁
bool prob_lock = true;
uint16_t T_max, T_min; // 温度
uint32_t T_avg;        // 温度平均值

/********************************************************************
   Function:        void read_EEPROM_byte(unsigned int eeaddress )
   Description:     read eeprom register as 8
   Dependencies:    register address (address)
 *******************************************************************/

/*
 * 新内核驱动版本：应用层不再通过 /dev/i2c-5 直接读 EEPROM。
 * 完整 EEPROM 由驱动缓存，并通过 ioctl(HEIMANN_IOC_GET_EEPROM) 传回应用层。
 */
static uint8_t g_heimann_eeprom_cache[HEIMANN_EEPROM_SIZE];
static int g_heimann_eeprom_cache_valid = 0;

static int heimann_load_eeprom_cache_from_driver(int heimann_fd)
{
  struct heimann_eeprom_dump *dump;
  int ret;

  dump = (struct heimann_eeprom_dump *)calloc(1, sizeof(*dump));
  if (!dump)
  {
    perror("calloc heimann_eeprom_dump");
    return -1;
  }

  dump->size = HEIMANN_EEPROM_SIZE;

  ret = ioctl(heimann_fd, HEIMANN_IOC_GET_EEPROM, dump);
  if (ret < 0)
  {
    perror("ioctl HEIMANN_IOC_GET_EEPROM failed");
    free(dump);
    return -1;
  }

  if (dump->size != HEIMANN_EEPROM_SIZE)
  {
    printf("EEPROM dump size mismatch: got=%u expected=%u\n",
           dump->size, HEIMANN_EEPROM_SIZE);
    free(dump);
    return -1;
  }

  memcpy(g_heimann_eeprom_cache, dump->data, HEIMANN_EEPROM_SIZE);
  g_heimann_eeprom_cache_valid = 1;

  printf("EEPROM cache loaded from /dev/heimann0: size=%u, first=%02x %02x %02x %02x\n",
         dump->size,
         g_heimann_eeprom_cache[0],
         g_heimann_eeprom_cache[1],
         g_heimann_eeprom_cache[2],
         g_heimann_eeprom_cache[3]);

  free(dump);
  return 0;
}

/********************************************************************
   Function:        int read_EEPROM_byte(int fd, uint16_t mem_reg)
   Description:     新版本从驱动 ioctl 得到的 EEPROM 缓存里读取 1 字节。
                    fd 参数保留，只是为了兼容原 read_eeprom(fd) 调用。
 *******************************************************************/
int read_EEPROM_byte(int fd, uint16_t mem_reg)
{
  (void)fd;

  if (!g_heimann_eeprom_cache_valid)
  {
    printf("EEPROM cache is not loaded yet!\n");
    return -1;
  }

  if (mem_reg >= HEIMANN_EEPROM_SIZE)
  {
    printf("EEPROM address out of range: 0x%04x\n", mem_reg);
    return -1;
  }

  return g_heimann_eeprom_cache[mem_reg];
}

int write_EEPROM_byte(int fd, uint16_t mem_addr, uint8_t content)
{
  // int retries;
  uint8_t data[3];

  data[0] = (uint8_t)(mem_addr >> 8);   // MSB
  data[1] = (uint8_t)(mem_addr & 0xFF); // LSB
  data[2] = content;

  // 设置地址长度：0为7位地址
  ioctl(fd, I2C_TENBIT, 0);

  // 设置从机地址
  if (ioctl(fd, I2C_SLAVE, EEPROM_ADDRESS) < 0)
  {
    printf("fail to set i2c device slave address!\n");
    close(fd);
    return -1;
  }

  // 设置收不到ACK时的重试次数
  ioctl(fd, I2C_RETRIES, 5);

  if (write(fd, data, 3) == 3)
  {
    return 0;
  }
  else
  {
    printf("fail to write_EEPROM_byte!\n");
    return -1;
  }
}

/********************************************************************
   Function:        void write_sensor_byte( unsigned short addr)
   Description:     write to sensor register
   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
int write_sensor_byte(int fd, uint8_t reg, uint8_t val)
{
  // int retries;
  uint8_t data[2];

  data[0] = reg;
  data[1] = val;

  // 设置地址长度：0为7位地址
  ioctl(fd, I2C_TENBIT, 0);

  // 设置从机地址
  if (ioctl(fd, I2C_SLAVE, SENSOR_ADDRESS) < 0)
  {
    printf("fail to set i2c device slave address!\n");
    close(fd);
    return -1;
  }

  // 设置收不到ACK时的重试次数
  ioctl(fd, I2C_RETRIES, 5);

  if (write(fd, data, 2) == 2)
  {
    return 0;
  }
  else
  {
    printf("fail to write_sensor_byte!\n");
    return -1;
  }
}

/********************************************************************
   Function:        void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n)
   Description:     read sensor register
 *******************************************************************/
int read_sensor_register(int fd, uint8_t reg, uint8_t *dest, uint16_t n)
{
  struct i2c_rdwr_ioctl_data ioctl_data;
  struct i2c_msg messages[2];

  // 写寄存器地址（不发 STOP）
  messages[0].addr = SENSOR_ADDRESS;
  messages[0].flags = 0; // 写
  messages[0].len = 1;
  messages[0].buf = &reg;

  // 读数据（发 REPEATED START）
  messages[1].addr = SENSOR_ADDRESS;
  messages[1].flags = I2C_M_RD; // 读
  messages[1].len = n;
  messages[1].buf = dest;

  ioctl_data.msgs = messages;
  ioctl_data.nmsgs = 2;

  if (ioctl(fd, I2C_RDWR, &ioctl_data) < 0)
  {
    perror("I2C_RDWR failed");
    return -1;
  }

  return 0; // 成功
}

/********************************************************************
   Function:        void read_eeprom()
   Description:     read all values from eeprom
 *******************************************************************/
void read_eeprom(int fd)
{
  int m = 0;
  int n = 0;
  uint8_t b[4];
  id = read_EEPROM_byte(fd, E_ID4) << 24 |
       read_EEPROM_byte(fd, E_ID3) << 16 |
       read_EEPROM_byte(fd, E_ID2) << 8 |
       read_EEPROM_byte(fd, E_ID1);
  printf("read eeprom-id: %d\n", id);
  mbit_calib = read_EEPROM_byte(fd, E_MBIT_CALIB);
  // printf("read eeprom-mbit_calib: %d\n", mbit_calib);
  bias_calib = read_EEPROM_byte(fd, E_BIAS_CALIB);
  clk_calib = read_EEPROM_byte(fd, E_CLK_CALIB);
  bpa_calib = read_EEPROM_byte(fd, E_BPA_CALIB);
  pu_calib = read_EEPROM_byte(fd, E_PU_CALIB);
  mbit_user = read_EEPROM_byte(fd, E_MBIT_USER);
  bias_user = read_EEPROM_byte(fd, E_BIAS_USER);
  clk_user = read_EEPROM_byte(fd, E_CLK_USER);
  bpa_user = read_EEPROM_byte(fd, E_BPA_USER);
  pu_user = read_EEPROM_byte(fd, E_PU_USER);
  vddth1 = read_EEPROM_byte(fd, E_VDDTH1_2) << 8 | read_EEPROM_byte(fd, E_VDDTH1_1);
  vddth2 = read_EEPROM_byte(fd, E_VDDTH2_2) << 8 | read_EEPROM_byte(fd, E_VDDTH2_1);
  vddscgrad = read_EEPROM_byte(fd, E_VDDSCGRAD);
  vddscoff = read_EEPROM_byte(fd, E_VDDSCOFF);
  ptatth1 = read_EEPROM_byte(fd, E_PTATTH1_2) << 8 | read_EEPROM_byte(fd, E_PTATTH1_1);
  ptatth2 = read_EEPROM_byte(fd, E_PTATTH2_2) << 8 | read_EEPROM_byte(fd, E_PTATTH2_1);
  nrofdefpix = read_EEPROM_byte(fd, E_NROFDEFPIX);
  gradscale = read_EEPROM_byte(fd, E_GRADSCALE);
  tablenumber = read_EEPROM_byte(fd, E_TABLENUMBER2) << 8 | read_EEPROM_byte(fd, E_TABLENUMBER1);
  arraytype = read_EEPROM_byte(fd, E_ARRAYTYPE);
  b[0] = read_EEPROM_byte(fd, E_PTATGR_1);
  b[1] = read_EEPROM_byte(fd, E_PTATGR_2);
  b[2] = read_EEPROM_byte(fd, E_PTATGR_3);
  b[3] = read_EEPROM_byte(fd, E_PTATGR_4);
  ptatgr_float = *(float *)b;
  b[0] = read_EEPROM_byte(fd, E_PTATOFF_1);
  b[1] = read_EEPROM_byte(fd, E_PTATOFF_2);
  b[2] = read_EEPROM_byte(fd, E_PTATOFF_3);
  b[3] = read_EEPROM_byte(fd, E_PTATOFF_4);
  ptatoff_float = *(float *)b;
  b[0] = read_EEPROM_byte(fd, E_PIXCMIN_1);
  b[1] = read_EEPROM_byte(fd, E_PIXCMIN_2);
  b[2] = read_EEPROM_byte(fd, E_PIXCMIN_3);
  b[3] = read_EEPROM_byte(fd, E_PIXCMIN_4);
  pixcmin = *(float *)b;
  b[0] = read_EEPROM_byte(fd, E_PIXCMAX_1);
  b[1] = read_EEPROM_byte(fd, E_PIXCMAX_2);
  b[2] = read_EEPROM_byte(fd, E_PIXCMAX_3);
  b[3] = read_EEPROM_byte(fd, E_PIXCMAX_4);
  pixcmax = *(float *)b;
  epsilon = read_EEPROM_byte(fd, E_EPSILON);
  globaloff = read_EEPROM_byte(fd, E_GLOBALOFF);
  globalgain = read_EEPROM_byte(fd, E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(fd, E_GLOBALGAIN_1);

  // for (int m = 0; m < DevConst.PixelPerColumn; m++) {
  //   for (int n = 0; n < DevConst.PixelPerRow; n++) {

  // --- DeadPixAdr ---
  for (int i = 0; i < nrofdefpix; i++)
  {
    deadpixadr[i] = read_EEPROM_byte(fd, E_DEADPIXADR + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_DEADPIXADR + 2 * i);
    if (deadpixadr[i] > (unsigned short)(DevConst.NumberOfPixel / 2))
    { // adaptedAdr:
      deadpixadr[i] = (unsigned short)(DevConst.NumberOfPixel) + (unsigned short)(DevConst.NumberOfPixel / 2) - deadpixadr[i] + 2 * (unsigned short)(deadpixadr[i] % DevConst.PixelPerRow) - DevConst.PixelPerRow;
    }
  }

  // --- DeadPixMask ---
  for (int i = 0; i < nrofdefpix; i++)
  {
    deadpixmask[i] = read_EEPROM_byte(fd, E_DEADPIXMASK + i);
  }

  // --- Thgrad_ij, ThOffset_ij and P_ij ---
  m = 0;
  n = 0;
  pixc2 = pixc2_0; // set pointer to start address of the allocated heap // reset pointer to initial address
  // top half
  for (int i = 0; i < (unsigned short)(DevConst.NumberOfPixel / 2); i++)
  {
    thgrad[m][n] = read_EEPROM_byte(fd, E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_THGRAD + 2 * i);
    thoffset[m][n] = read_EEPROM_byte(fd, E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_THOFFSET + 2 * i);
    *(pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(fd, E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_PIJ + 2 * i);
    n++;
    if (n == DevConst.PixelPerRow)
    {
      n = 0;
      m++; // !!!! forwards !!!!
    }
  }
  // bottom half
  m = (unsigned char)(DevConst.PixelPerColumn - 1);
  n = 0;
  for (int i = (unsigned short)(DevConst.NumberOfPixel / 2); i < (unsigned short)(DevConst.NumberOfPixel); i++)
  {
    thgrad[m][n] = read_EEPROM_byte(fd, E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_THGRAD + 2 * i);
    thoffset[m][n] = read_EEPROM_byte(fd, E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_THOFFSET + 2 * i);
    *(pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(fd, E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_PIJ + 2 * i);
    n++;

    if (n == DevConst.PixelPerRow)
    {
      n = 0;
      m--; // !!!! backwards !!!!
    }
  }

  //---VddCompGrad and VddCompOff---
  // top half
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < (unsigned short)(DevConst.PixelPerBlock); i++)
  {
    vddcompgrad[m][n] = read_EEPROM_byte(fd, E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_VDDCOMPGRAD + 2 * i);
    vddcompoff[m][n] = read_EEPROM_byte(fd, E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_VDDCOMPOFF + 2 * i);
    n++;
    if (n == DevConst.PixelPerRow)
    {
      n = 0;
      m++; // !!!! forwards !!!!
    }
  }
  // bottom half
  m = (unsigned char)(DevConst.RowPerBlock * 2 - 1);
  n = 0;
  for (int i = (unsigned short)(DevConst.PixelPerBlock); i < (unsigned short)(DevConst.PixelPerBlock * 2); i++)
  {
    vddcompgrad[m][n] = read_EEPROM_byte(fd, E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_VDDCOMPGRAD + 2 * i);
    vddcompoff[m][n] = read_EEPROM_byte(fd, E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(fd, E_VDDCOMPOFF + 2 * i);
    n++;
    if (n == DevConst.PixelPerRow)
    {
      n = 0;
      m--; // !!!! backwards !!!!
    }
  }
}

/********************************************************************
   Function:        void write_calibration_settings_to_sensor()
   Description:     write calibration data (from eeprom) to trim registers (sensor)
 *******************************************************************/
void write_calibration_settings_to_sensor(int fd)
{

  write_sensor_byte(fd, TRIM_REGISTER1, mbit_calib);
  usleep(5000);
  write_sensor_byte(fd, TRIM_REGISTER2, bias_calib);
  usleep(5000);
  write_sensor_byte(fd, TRIM_REGISTER3, bias_calib);
  usleep(5000);
  write_sensor_byte(fd, TRIM_REGISTER4, clk_calib);
  usleep(5000);
  write_sensor_byte(fd, TRIM_REGISTER5, bpa_calib);
  usleep(5000);
  write_sensor_byte(fd, TRIM_REGISTER6, bpa_calib);
  usleep(5000);
  write_sensor_byte(fd, TRIM_REGISTER7, pu_calib);
  usleep(5000);
}

/********************************************************************
   Function:      calcPixC
   Description:   calculates the pixel constants with the unscaled
                  values from EEPROM
 *******************************************************************/
void calcPixC()
{
  /* uses the formula from datasheet:

                     PixC_uns[m][n]*(PixCmax-PixCmin)               epsilon   GlobalGain
      PixC[m][n] = ( -------------------------------- + PixCmin ) * ------- * ----------
                                  65535                               100        1000
  */

  double pixcij;
  pixc2 = pixc2_0; // set pointer to start address of the allocated heap

  for (int m = 0; m < DevConst.PixelPerColumn; m++)
  {
    for (int n = 0; n < DevConst.PixelPerRow; n++)
    {

      pixcij = (double)pixcmax;
      pixcij -= (double)pixcmin;
      pixcij /= (double)65535.0;
      pixcij *= (double)*pixc2;
      pixcij += (double)pixcmin;
      pixcij /= (double)100.0;
      pixcij *= (double)epsilon;
      pixcij /= (double)10000.0;
      pixcij *= (double)globalgain;
      pixcij += 0.5;

      *pixc2 = (unsigned long)pixcij;
      pixc2++;
    }
  }

  lastepsilon = epsilon;
}

/********************************************************************
   Function:      calc_timert(uint8_t clk, uint8_t mbit)
   Description:   calculate the duration of the timer which reads the sensor blocks
 *******************************************************************/
uint16_t calc_timert(uint8_t clk, uint8_t mbit)
{

  float a;
  uint16_t calculated_timer_duration;

  float Fclk_float = 12000000.0 / 63.0 * (float)clk + 1000000.0; // calc clk in Hz
  a = 32.0 * ((float)pow(2, (unsigned char)(mbit & 0b00001111)) + 4.0) / Fclk_float;

  calculated_timer_duration = (unsigned short)(0.98 * a * 1000000); // c in s | timer_duration in µs
  return calculated_timer_duration;
}

// 定时器触发的信号处理函数
void heimann_timer_handle()
{
  if (ReadingRoutineEnable)
  {
    NewDataAvailable = 1; // 标记新数据可用
  }
}

int stop_timer(int timer_fd)
{
  struct itimerspec ts = {0};
  if (timerfd_settime(timer_fd, 0, &ts, NULL) == -1)
  {
    perror("timerfd_settime (stop)");
    return -1;
  }
  return 0;
}

int set_timer(int timer_fd, uint32_t interval_us)
{
  struct itimerspec ts;
  ts.it_value.tv_sec = interval_us / 1000000;
  ts.it_value.tv_nsec = (interval_us % 1000000) * 1000;
  ts.it_interval = ts.it_value;

  if (timerfd_settime(timer_fd, 0, &ts, NULL) == -1)
  {
    perror("timerfd_settime (restart)");
    return -1;
  }
  return 0;
}

/********************************************************************
   Function:        void pixel_masking()
   Description:     repair dead pixel by using the average of the neighbors
 *******************************************************************/
void pixel_masking()
{
  uint8_t number_neighbours[ALLOWED_DEADPIX];
  uint32_t temp_defpix[ALLOWED_DEADPIX];
  for (int i = 0; i < nrofdefpix; i++)
  {
    number_neighbours[i] = 0;
    temp_defpix[i] = 0;

    // top half
    if (deadpixadr[i] < (unsigned short)(NUMBER_OF_PIXEL / 2))
    {

      if ((deadpixmask[i] & 1) == 1)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ((deadpixmask[i] & 2) == 2)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ((deadpixmask[i] & 4) == 4)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ((deadpixmask[i] & 8) == 8)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ((deadpixmask[i] & 16) == 16)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ((deadpixmask[i] & 32) == 32)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ((deadpixmask[i] & 64) == 64)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ((deadpixmask[i] & 128) == 128)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }
    }

    // bottom half
    else
    {

      if ((deadpixmask[i] & 1) == 1)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ((deadpixmask[i] & 2) == 2)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ((deadpixmask[i] & 4) == 4)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ((deadpixmask[i] & 8) == 8)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ((deadpixmask[i] & 16) == 16)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ((deadpixmask[i] & 32) == 32)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ((deadpixmask[i] & 64) == 64)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ((deadpixmask[i] & 128) == 128)
      {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }
    }

    temp_defpix[i] = temp_defpix[i] / number_neighbours[i];
    data_pixel[deadpixadr[i] / PIXEL_PER_ROW][deadpixadr[i] % PIXEL_PER_ROW] = temp_defpix[i];
  }
}

/********************************************************************
   Function:        calculate_pixel_temp()
   Description:     compensate thermal, electrical offset and vdd and multiply sensitivity coeff
                    look for the correct temp in lookup table
 *******************************************************************/
void calculate_pixel_temp()
{

  int64_t vij_pixc_and_pcscaleval;
  // int64_t pixcij;
  int64_t vdd_calc_steps;
  uint16_t table_row = 0, table_col = 0;
  int32_t vx, vy, ydist, dta;
  signed long pixel;
  pixc2 = pixc2_0; // set pointer to start address of the allocated heap

  /******************************************************************************************************************
    step 0: find column of lookup table
  ******************************************************************************************************************/
  for (int i = 0; i < NROFTAELEMENTS; i++)
  {
    if (Ta > XTATemps[i])
    {
      table_col = i;
    }
  }
  dta = Ta - XTATemps[table_col];
  ydist = (int32_t)ADEQUIDISTANCE;

  flag_min_max_initaled = false;
  for (int m = 0; m < DevConst.PixelPerColumn; m++)
  {
    for (int n = 0; n < DevConst.PixelPerRow; n++)
    {

      /******************************************************************************************************************
         step 1: use a variable with bigger data format for the compensation steps
       ******************************************************************************************************************/
      prob_lock = true;
      pixel = (signed long)data_pixel[m][n];
      prob_lock = false;
      /******************************************************************************************************************
         step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
       ******************************************************************************************************************/
      pixel -= (int32_t)(((int32_t)thgrad[m][n] * (int32_t)ptat_av_uint16) / (int32_t)gradscale_div);
      pixel -= (int32_t)thoffset[m][n];

      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      if (m < DevConst.PixelPerColumn / 2)
      { // top half
        pixel -= eloffset[m % DevConst.RowPerBlock][n];
      }
      else
      { // bottom half
        pixel -= eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }

      /******************************************************************************************************************
         step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
       ******************************************************************************************************************/
      // first select VddCompGrad and VddCompOff for pixel m,n:
      if (m < DevConst.PixelPerColumn / 2)
      { // top half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock][n];
      }
      else
      { // bottom half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      // now do the vdd calculation
      vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * (vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16 - ptatth1));
      vdd_calc_steps = vdd_calc_steps / vddscoff_div;
      pixel -= vdd_calc_steps;

      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
      pixel = (int32_t)(vij_pixc_and_pcscaleval / *pixc2);
      pixc2++;
      /******************************************************************************************************************
         step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
       ******************************************************************************************************************/
      table_row = pixel + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      pixel = (uint32_t)((vy - vx) * ((int32_t)(pixel + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

      /******************************************************************************************************************
         step 7: add GlobalOffset (stored as signed char)
       ******************************************************************************************************************/
      pixel += globaloff;

      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      data_pixel[m][n] = (unsigned short)pixel;

      /******************************************************************************************************************
        step 9: find min and max value
      ******************************************************************************************************************/

      if (flag_min_max_initaled == false)
      {
        T_max = data_pixel[m][n];
        T_min = data_pixel[m][n];
        T_avg = data_pixel[m][n];
        flag_min_max_initaled = true;
      }
      if (n < 30 && m < 30 && n > 2 && m > 2)
      {
        if (data_pixel[m][n] < T_min)
        {
          T_min = data_pixel[m][n];
          x_min = n;
          y_min = m;
        }

        if (data_pixel[m][n] > T_max)
        {
          T_max = data_pixel[m][n];
          x_max = n;
          y_max = m;
        }
        T_avg = T_avg + data_pixel[m][n];
      }
    }
  }
  /******************************************************************************************************************
    step 8: overwrite the uncompensate pixel with the new calculated compensated value
  ******************************************************************************************************************/
  T_avg = T_avg / 729;
  pixel_masking();
}

/********************************************************************
   Function:        void sort_data()
   Description:     sort the raw data blocks in 2d array and calculate ambient temperature, ptat and vdd
 *******************************************************************/
void sort_data()
{

  unsigned long sum = 0;
  unsigned short pos = 0;

  for (int m = 0; m < DevConst.RowPerBlock; m++)
  {
    for (int n = 0; n < DevConst.PixelPerRow; n++)
    {

      /*
         for example: a normal line of RAMoutput for HTPAd80x64 looks like:
         RAMoutput[0][] = [ PTAT(MSB), PTAT(LSB), DATA0[MSB], DATA0[LSB], DATA1[MSB], DATA1[LSB], ... , DATA640[MSB], DATA640LSB];
                                                      |
                                                      |-- DATA_Pos = 2 (first data byte)
      */
      pos = (unsigned short)(2 * n + DevConst.DataPos + m * 2 * DevConst.PixelPerRow);

      /******************************************************************************************************************
        new PIXEL values
      ******************************************************************************************************************/
      for (int i = 0; i < DevConst.NumberOfBlocks; i++)
      {
        // top half
        data_pixel[m + i * DevConst.RowPerBlock][n] =
            (unsigned short)(RAMoutput[i][pos] << 8 | RAMoutput[i][pos + 1]);
        // bottom half
        data_pixel[DevConst.PixelPerColumn - 1 - m - i * DevConst.RowPerBlock][n] =
            (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos + 1]);
      }

      /******************************************************************************************************************
        new electrical offset values (store them in electrical offset buffer and calculate the average for pixel compensation
      ******************************************************************************************************************/
      if (picnum % ELOFFSETS_BUFFER_SIZE == 1)
      {
        if ((!eloffset[m][n]) || (picnum < ELOFFSETS_FILTER_START_DELAY))
        {
          // top half
          eloffset[m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
          // bottom half
          eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
          use_eloffsets_buffer = 1;
        }
        else
        {
          // use a moving average filter
          // top half
          sum = (unsigned long)eloffset[m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
          eloffset[m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
          // bottom half
          sum = (unsigned long)eloffset[2 * DevConst.RowPerBlock - 1 - m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
          eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
        }
      }
    }
  }

  /******************************************************************************************************************
    new PTAT values (store them in PTAT buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  if (sampled_switch_ptat_vdd == 1)
  {
    sum = 0;
    // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
    for (int i = 0; i < DevConst.NumberOfBlocks; i++)
    {
      // block top half
      sum += (unsigned short)(RAMoutput[i][DevConst.PTATPos] << 8 | RAMoutput[i][DevConst.PTATPos + 1]);
      // block bottom half
      sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos + 1]);
    }
    ptat_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));
    Ta = (unsigned short)((unsigned short)ptat_av_uint16 * (float)ptatgr_float + (float)ptatoff_float);

    ptat_buffer[ptat_i] = ptat_av_uint16;
    ptat_i++;
    if (ptat_i == PTAT_BUFFER_SIZE)
    {
      if (use_ptat_buffer == 0)
      {
        // Serial.print(" | PTAT buffer complete");
        use_ptat_buffer = 1;
      }
      ptat_i = 0;
    }

    if (use_ptat_buffer)
    {
      // now overwrite the old ptat average
      sum = 0;
      for (int i = 0; i < PTAT_BUFFER_SIZE; i++)
      {
        sum += ptat_buffer[i];
      }
      ptat_av_uint16 = (uint16_t)((float)sum / PTAT_BUFFER_SIZE);
    }
  }

  /******************************************************************************************************************
    new VDD values (store them in VDD buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  if (sampled_switch_ptat_vdd == 0)
  {
    sum = 0;
    // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
    for (int i = 0; i < DevConst.NumberOfBlocks; i++)
    {
      // block top half
      sum += (unsigned short)(RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][DevConst.VDDPos + 1]);
      // block bottom half
      sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos + 1]);
    }
    vdd_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));

    // write into vdd buffer
    vdd_buffer[vdd_i] = vdd_av_uint16;
    vdd_i++;
    if (vdd_i == VDD_BUFFER_SIZE)
    {
      if (use_vdd_buffer == 0)
      {
        // Serial.print(" | VDD buffer complete");
        use_vdd_buffer = 1;
      }
      vdd_i = 0;
    }
    if (use_vdd_buffer)
    {
      sum = 0;
      for (int i = 0; i < VDD_BUFFER_SIZE; i++)
      {
        sum += vdd_buffer[i];
      }
      // now overwrite the old vdd average
      vdd_av_uint16 = (uint16_t)((float)sum / VDD_BUFFER_SIZE);
    }
  }
}
/********************************************************************
   Function:        void readblockinterrupt()
   Description:     read one sensor block and change configuration register to next block
                    (also read electrical offset when read_eloffset_next_pic is set)
 *******************************************************************/
void readblockinterrupt(int sensor_fd, int timer_fd, uint32_t interval_us)
{
  // printf("readblockinterrupt_in\n");
  unsigned char bottomblock;
  ReadingRoutineEnable = 0;
  stop_timer(timer_fd);
  // printf("stop_timer\n");
  // delay(15);
  // wait for end of conversion bit (~27ms)
  // check EOC bit
  read_sensor_register(sensor_fd, STATUS_REGISTER, (uint8_t *)&statusreg, 1);
  // printf("statusreg: %d\n", statusreg);
  while ((statusreg & (1 << 0)) == 0)
  {
    read_sensor_register(sensor_fd, STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    usleep(10000);
  }
  // get data of top half:
  read_sensor_register(sensor_fd, TOP_HALF, (uint8_t *)&RAMoutput[read_block_num], BLOCK_LENGTH);
  bottomblock = (unsigned char)((unsigned char)(NUMBER_OF_BLOCKS + 1) * 2 - read_block_num - 1);
  read_sensor_register(sensor_fd, BOTTOM_HALF, (uint8_t *)&RAMoutput[bottomblock], BLOCK_LENGTH);

  read_block_num++;
  // printf("block %d sampled\n", read_block_num);
  if (read_block_num < NUMBER_OF_BLOCKS)
  {
    // to start sensor set configuration register to 0x09
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  x  |  x  |   1   |    0     |   0   |    1   |
    write_sensor_byte(sensor_fd, CONFIGURATION_REGISTER, (unsigned char)(0x09 + (0x10 * read_block_num) + (0x04 * switch_ptat_vdd)));
  }
  else
  {
    //*******************************************************************
    // all blocks for the current image are sampled, now check if its time
    // to get new electrical offsets and/or for switching PTAT and VDD
    //*******************************************************************
    // Serial.println("all blocks for the current image are sampled");
    if (read_eloffset_next_pic)
    {
      read_eloffset_next_pic = 0;
      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
      write_sensor_byte(sensor_fd, CONFIGURATION_REGISTER, (unsigned char)(0x0B + (0x04 * switch_ptat_vdd)));
      new_offsets = 1;
    }
    else
    {
      sampled_switch_ptat_vdd = switch_ptat_vdd;

      if (picnum > 1)
        state = 1; // state = 1 means that all required blocks are sampled
      picnum++;    // increase the picture counter
      // check if the next sample routine should include electrical offsets
      if ((unsigned char)(picnum % READ_ELOFFSET_EVERYX) == 0)
        read_eloffset_next_pic = 1;
      if (DevConst.PTATVDDSwitch)
        switch_ptat_vdd ^= 1;
      read_block_num = 0;
      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
      write_sensor_byte(sensor_fd, CONFIGURATION_REGISTER, (unsigned char)(0x09 + (0x04 * switch_ptat_vdd)));
    }
  }
  set_timer(timer_fd, interval_us);
  // printf("set_timer\n");
  ReadingRoutineEnable = 1;
}

/********************************************************************
 ********************************************************************
    - - - PART 4: SERIAL FUNCTIONS - - -
  checkSerial()
  print_eeprom_header()
  print_eeprom_hex()
  print_menu()
 ********************************************************************
 ********************************************************************/
/********************************************************************
   Function:        print_eeprom_header()
   Description:
 *******************************************************************/
void print_eeprom_header() {
    printf("data\t\tregister\ttype\t\tvalue\n");
    printf("------------------------------------------------------------\n");
    printf("PixCmin\t\t0x00-0x03\tfloat\t\t");
    printf("%.1f\n", pixcmin);
    printf("PixCmax\t\t0x04-0x07\tfloat\t\t");
    printf("%.1f\n", pixcmax);
    printf("gradScale\t0x08\t\tunsigned char\t");
    printf("%d\n", gradscale);
    printf("TN\t\t0x0B-0x0C\tunsigned short\t");
    printf("%d\n", tablenumber);
    printf("epsilon\t\t0x0D\t\tunsigned char\t");
    printf("%d\n", epsilon);
    printf("MBIT(calib)\t0x1A\t\tunsigned char\t");
    printf("%d\n", mbit_calib);
    printf("BIAS(calib)\t0x1B\t\tunsigned char\t");
    printf("%d\n", bias_calib);
    printf("CLK(calib)\t0x1C\t\tunsigned char\t");
    printf("%d\n", clk_calib);
    printf("BPA(calib)\t0x1D\t\tunsigned char\t");
    printf("%d\n", bpa_calib);
    printf("PU(calib)\t0x1E\t\tunsigned char\t");
    printf("%d\n", pu_calib);
    printf("Arraytype\t0x22\t\tunsigned char\t");
    printf("%d\n", arraytype);
    printf("VDDTH1\t\t0x26-0x27\tunsigned short\t");
    printf("%d\n", vddth1);
    printf("VDDTH2\t\t0x28-0x29\tunsigned short\t");
    printf("%d\n", vddth2);
    printf("PTAT-gradient\t0x34-0x37\tfloat\t\t");
    printf("%.4f\n", ptatgr_float);
    printf("PTAT-offset\t0x38-0x3B\tfloat\t\t");
    printf("%.4f\n", ptatoff_float);
    printf("PTAT(Th1)\t0x3C-0x3D\tunsigned short\t");
    printf("%d\n", ptatth1);
    printf("PTAT(Th2)\t0x3E-0x3F\tunsigned short\t");
    printf("%d\n", ptatth2);
    printf("VddScGrad\t0x4E\t\tunsigned char\t");
    printf("%d\n", vddscgrad);
    printf("VddScOff\t0x4F\t\tunsigned char\t");
    printf("%d\n", vddscoff);
    printf("GlobalOff\t0x54\t\tsigned char\t");
    printf("%d\n", globaloff);
    printf("GlobalGain\t0x55-0x56\tunsigned short\t");
    printf("%d\n", globalgain);
    printf("SensorID\t0x74-0x77\tunsigned long\t");
    printf("%d\n", id);
  }

  /********************************************************************
     Function:        print_eeprom_hex()
     Description:     print eeprom contint as hex values
   *******************************************************************/
  void print_eeprom_hex(int fd) {

    printf("\n\n\n---PRINT EEPROM (HEX)---\n");
    printf("\n\n\t\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

    // line
    for (int i = 0; i < 75; i++) {
      printf("- ");
    }

    for (int i = 0; i < EEPROM_SIZE; i++) {

      if (i % 16 == 0) {
        printf("\n");

        if (i < E_DEADPIXADR) {
          printf("HEADER\t0x");
          printf("%X", i);
          printf("\t|\t");
        }
        else if (i < 0x00D0) {
          printf("DEADPIX\t0x");
          printf("%X", i);
          printf("\t|\t");
        }
        else if (i < E_VDDCOMPGRAD) {
          printf("FREE\t0x");
          printf("%X", i);
          printf("\t|\t");
        }
        else if (i < E_VDDCOMPOFF) {
          printf("VDDGRAD\t0x");
          printf("%X", i);
          printf("\t|\t");
        }
        else if (i < E_THGRAD) {
          printf("VDDOFF\t0x");
          printf("%X", i);
          printf("\t|\t");
        }
        else if (i < E_THOFFSET) {
          printf("THGRAD\t0x");
          printf("%X", i);
          printf("\t|\t");
        }
        else if (i < E_PIJ) {
          printf("THOFF\t0x");
          printf("%X", i);
          printf("\t|\t");
        }
        else if (i < (E_PIJ + 2*NUMBER_OF_PIXEL)) {
          printf("PIXC\t0x");
          printf("%X", i);
          printf("\t|\t");
        }
        else {
          printf("FREE\t0x");
          printf("%X", i);
          printf("\t|\t");
        }
      }
      else {
        printf("\t");
      }

      printf("0x");
      if (read_EEPROM_byte(fd, i) < 0x10) {
        printf("0");
      }
      printf("%X", read_EEPROM_byte(fd, i));

    }

    printf("\n\n\n\ndone (m... back to menu)\n\n\n");
  }

//   /********************************************************************
//      Function:      print_menu()
//      Description:
//    *******************************************************************/
//   void print_menu() {
//     Serial.println("\n\n\n***************************************************");
//     Serial.println("Application Shield                      /_/eimann");
//     Serial.println("for ESP32-DevkitC                      / /   Sensor");

//     Serial.println("\nYou can choose one of these options by sending the \ncharacter\n ");
//     Serial.println("read SENSOR values:");
//     Serial.println("  a... final array temperatures (in deci Kelvin)");
//     Serial.println("  b... show all raw values (in digits)");
//     Serial.println("  c... show all calculation steps");
//     Serial.println("read EEPROM values:");
//     Serial.println("  d... whole eeprom content (in hexadecimal)");
//     Serial.println("  e... Header values");
//     Serial.println("  f... VddCompGrad");
//     Serial.println("  g... VddCompOff");
//     Serial.println("  h... ThGrad");
//     Serial.println("  i... ThOff");
//     Serial.println("  j... PixC (scaled)");
//     Serial.println("write/change EEPROM values:");
//     Serial.println("  k... increase emissivity by 1");
//     Serial.println("  l... decrease emissivity by 1");
//     Serial.println("\t\t\t\t\tver2.2 (dp)");
//     Serial.println("***************************************************\n\n\n");
//   }

//   /********************************************************************
//      Function:        print_final_array()
//      Description:
//    *******************************************************************/
//   void print_final_array(void) {
//     Serial.println("\n\n---pixel data ---");
//     for (int m = 0; m < DevConst.PixelPerColumn; m++) {
//       for (int n = 0; n < DevConst.PixelPerRow; n++) {
//         Serial.print(data_pixel[m][n]);
//         Serial.print("\t");
//       }
//       Serial.println("");
//     }
//   }

//   /********************************************************************
//      Function:        print_RAM_array()
//      Description:
//    *******************************************************************/
//   void print_RAM_array(void) {
//     Serial.print("\n\n\n---pixel data ---\n");
//     for (int m = 0; m < (2 * NUMBER_OF_BLOCKS + 2); m++) {
//       for (int n = 0; n < BLOCK_LENGTH; n++) {
//         Serial.print(RAMoutput[m][n], HEX);
//         Serial.print("\t");
//       }
//       Serial.print("\n");
//     }
//     Serial.print("\n\n\n");
//   }

//   /********************************************************************
//      Function:        checkSerial()
//      Description:
//    *******************************************************************/
//   void checkSerial() {
//     serial_input = Serial.read();

//     switch (serial_input) {
//       case 0xFF:
//         //nothing
//         break;

//       case 'a':
//         if (send_data)
//           Serial.println("stop data stream in GUI before");
//         else
//           print_state = 1;
//         break;

//       case 'b':
//         if (send_data)
//           Serial.println("stop data stream in GUI before");
//         else
//           print_state = 2;
//         break;

//       case 'c':
//         if (send_data)
//           Serial.println("stop data stream in GUI before");
//         else
//           print_state = 3;
//         break;

//       case 'm':
//         while (state);
//         ReadingRoutineEnable = 0;
//         print_menu();
//         ReadingRoutineEnable = 1;
//         break;

//       case 'd':
//         while (state);
//         ReadingRoutineEnable = 0;
//         print_eeprom_hex();
//         ReadingRoutineEnable = 1;
//         break;

//       case 'e':
//         while (state);
//         ReadingRoutineEnable = 0;
//         print_eeprom_header();
//         ReadingRoutineEnable = 1;
//         break;

//       case 'f':
//         while (state);
//         ReadingRoutineEnable = 0;
//         Serial.print("\n\n\n---VddCompGrad---\n");
//         for (int m = 0; m < (DevConst.RowPerBlock * 2); m++) {
//           for (int n = 0; n < DevConst.PixelPerRow; n++) {
//             Serial.print(vddcompgrad[m][n]);
//             Serial.print("\t");
//           }
//           Serial.print("\n");
//         }
//         Serial.print("\n\n\n");
//         ReadingRoutineEnable = 1;
//         break;

//       case 'g':
//         while (state);
//         ReadingRoutineEnable = 0;
//         Serial.print("\n\n\n---VddCompOff---\n");
//         for (int m = 0; m < (DevConst.RowPerBlock * 2); m++) {
//           for (int n = 0; n < DevConst.PixelPerRow; n++) {
//             Serial.print(vddcompoff[m][n]);
//             Serial.print("\t");
//           }
//           Serial.print("\n");
//         }
//         Serial.print("\n\n\n");
//         ReadingRoutineEnable = 1;
//         break;

//       case 'h':
//         while (state);
//         ReadingRoutineEnable = 0;
//         Serial.print("\n\n\n---ThGrad---\n");
//         for (int m = 0; m < DevConst.PixelPerColumn; m++) {
//           for (int n = 0; n < DevConst.PixelPerRow; n++) {
//             Serial.print(thgrad[m][n]);
//             Serial.print("\t");
//           }
//           Serial.print("\n");
//         }
//         Serial.print("\n\n\n");
//         ReadingRoutineEnable = 1;
//         break;

//       case 'i':
//         while (state);
//         ReadingRoutineEnable = 0;
//         // print ThOffset in serial monitor
//         Serial.print("\n\n\n---ThOffset---\n");
//         for (int m = 0; m < DevConst.PixelPerColumn; m++) {
//           for (int n = 0; n < DevConst.PixelPerRow; n++) {
//             Serial.print(thoffset[m][n]);
//             Serial.print("\t");
//           }
//           Serial.print("\n");
//         }
//         Serial.print("\n\n\n");
//         ReadingRoutineEnable = 1;
//         break;

//       case 'j':
//         while (state);
//         ReadingRoutineEnable = 0;
//         // print PixC in serial monitor
//         Serial.print("\n\n\n---PixC---\n");
//         pixc2 = pixc2_0; // set pointer to start address of the allocated heap
//         for (int m = 0; m < DevConst.PixelPerColumn; m++) {
//           for (int n = 0; n < DevConst.PixelPerRow; n++) {
//             Serial.print(*(pixc2 + m * DevConst.PixelPerRow + n));
//             Serial.print("\t");
//           }
//           Serial.print("\n");
//         }
//         Serial.print("\n\n\n");
//         ReadingRoutineEnable = 1;
//         break;

//       case 'k':
//         ReadingRoutineEnable = 0;
//         TimerLib.clearTimer();
//         Serial.println("\n\n\n---Increase emissivity---");
//         Serial.print("old emissivity: \t");
//         Serial.println(epsilon);
//         Serial.print("new emissivity: \t");
//         if (epsilon < 100) {
//           epsilon++;
//           Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
//           write_EEPROM_byte(E_EPSILON, epsilon);
//           Wire.setClock(CLOCK_SENSOR);

//           // calculate pixcij with new epsilon
//           pixc2 = pixc2_0; // set pointer to start address of the allocated heap
//           double d = (double)epsilon / (double)lastepsilon;
//           for (int m = 0; m < DevConst.PixelPerColumn; m++) {
//             for (int n = 0; n < DevConst.PixelPerRow; n++) {
//               *(pixc2 + m * DevConst.PixelPerRow + n) = (unsigned long)((double) * (pixc2 + m * DevConst.PixelPerRow + n) * (double)d);
//             }
//           }
//           lastepsilon = epsilon;
//           Serial.print(epsilon);
//           Serial.println(" (new emissivity is stored in the EEPROM now)");
//         }
//         else {
//           Serial.print(epsilon);
//           Serial.println(" (you cannot set the emissivity higher than 100%)");
//         }
//         delay(1000);
//         TimerLib.setInterval_us(ISR, timert);
//         ReadingRoutineEnable = 1;
//         break;

//       case 'l':
//         ReadingRoutineEnable = 0;
//         TimerLib.clearTimer();
//         Serial.print("\n\n\n---Decrease emissivity---");
//         Serial.print("\nold emissivity: \t");
//         Serial.print(epsilon);
//         Serial.print("\nnew emissivity: \t");
//         if (epsilon > 0) {
//           epsilon--;
//           Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
//           write_EEPROM_byte(E_EPSILON, epsilon);
//           Wire.setClock(CLOCK_SENSOR);
//           // calculate pixcij with new epsilon
//           pixc2 = pixc2_0; // set pointer to start address of the allocated heap
//           double d = (double)epsilon / (double)lastepsilon;
//           for (int m = 0; m < DevConst.PixelPerColumn; m++) {
//             for (int n = 0; n < DevConst.PixelPerRow; n++) {
//               *(pixc2 + m * DevConst.PixelPerRow + n) = (unsigned long)((double) * (pixc2 + m * DevConst.PixelPerRow + n) * (double)d);
//             }
//           }
//           lastepsilon = epsilon;
//           Serial.print(epsilon);
//           Serial.print(" (new emissivity is stored in the EEPROM now)");
//         }
//         else {
//           Serial.print(epsilon);
//           Serial.print(" (you cannot set the emissivity lower as 0%)");
//         }
//         delay(1000);
//         TimerLib.setInterval_us(ISR, timert);
//         ReadingRoutineEnable = 1;
//         break;

//     }

// }
//红外传感器的I2C 连接探测与确认函数
int connect_sensor(int fd)
{
	int error;
	while (1)
	{
		printf("Trying to connect to HTPAd at address: %d\n", SENSOR_ADDRESS);
		// 设置从机地址，此时为SENSOR，1MHz
		if (ioctl(fd, I2C_SLAVE, SENSOR_ADDRESS) < 0)
		{
			perror("Failed to set slave address");
			return -1;
		}

		// 简单尝试：发送一个空传输，看是否 ACK（等价于 Wire.beginTransmission + endTransmission）
		uint8_t dummy = 0x00;
		error = write(fd, &dummy, 0); // 写 0 字节，探测地址
		if (error >= 0)
		{
			printf("Sensor is ready!\n");
			break;
		}

		printf("Sensor not ready, retrying..., error code: %d\n", error);
		sleep(2);
	}
	return 0;
}

//核心初始化总入口，整合了 “内存分配、传感器连接校验、
//EEPROM 校准参数读取、传感器硬件唤醒 / 配置、校准参数写入、前置计算” 等全流程
int sensor_init(int sensor_fd, int eeprom_fd)
{
  // 步骤1：为像素常数PixC分配堆内存（核心数据载体）
	pixc2_0 = (uint32_t *)malloc(NUMBER_OF_PIXEL * 4);
	if (pixc2_0 == NULL)
	{
		printf("heap_caps_malloc failed\n");
	}
	else
	{
		printf("heap_caps_malloc succeeded\n");
		pixc2 = pixc2_0; // set pointer to start address of the allocated heap
	}

	//*******************************************************************
	// searching for sensor; if connected: read the whole EEPROM
	//*******************************************************************
  // 步骤2：前置校验——确保传感器I2C通信正常
	connect_sensor(sensor_fd);//测试iic连接状态

  // 步骤3：更新驱动状态机——标记“初始化中”
	prob_status = PROB_INITIALIZING; // 探头连接状态

  // 步骤4：读取EEPROM中所有校准参数（核心配置）
  // 读取pixcmin/mbit_calib/clk_calib等参数到全局变量
	read_eeprom(eeprom_fd);

	//*******************************************************************
	// wake up and start the sensor
	//*******************************************************************
	// to wake up sensor set configuration register to 0x01
	// |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
	// |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
  // 步骤5：硬件唤醒——向传感器配置寄存器写入0x01，唤醒传感器
	write_sensor_byte(sensor_fd, CONFIGURATION_REGISTER, 0x01);
	// write the calibration settings into the trim registers
  // 步骤6：写入校准参数到传感器TRIM寄存器
	write_calibration_settings_to_sensor(sensor_fd);
	// to start sensor set configuration register to 0x09
	// |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
	// |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  // 步骤7：硬件启动——向配置寄存器写入0x09，启动传感器采集
	write_sensor_byte(sensor_fd, CONFIGURATION_REGISTER, 0x09);
	printf("HTPAd is ready\n");

  // 步骤8：更新驱动状态机——标记“初始化完成/准备中”
	prob_status = PROB_PREPARING;

	//*******************************************************************
	// do bigger calculation here before you jump into the loop() function
	//*******************************************************************
	gradscale_div = pow(2, gradscale);
	vddscgrad_div = pow(2, vddscgrad);
	vddscoff_div = pow(2, vddscoff);
  // 步骤10：计算每个像素的灵敏度常数PixC（温度计算核心）
	calcPixC(); // calculate the pixel constants

	return 0;
}


/*
 * 新应用层初始化：
 * 1. 从 /dev/heimann0 通过 ioctl 获取完整 EEPROM 缓存；
 * 2. 复用原 read_eeprom() 解析校准参数；
 * 3. 复用原 calcPixC() 计算像素常数。
 *
 * 注意：sensor 唤醒、TRIM 写入、启动采集已经由内核驱动完成，
 * 这里不再调用 connect_sensor()/write_sensor_byte()/write_calibration_settings_to_sensor()。
 */
static int heimann_app_calibration_init(int heimann_fd)
{
  if (heimann_load_eeprom_cache_from_driver(heimann_fd) < 0)
  {
    printf("failed to load EEPROM cache from heimann driver\n");
    return -1;
  }

  if (!pixc2_0)
  {
    pixc2_0 = (uint32_t *)malloc(NUMBER_OF_PIXEL * sizeof(uint32_t));
    if (pixc2_0 == NULL)
    {
      printf("malloc pixc2_0 failed\n");
      return -1;
    }
    printf("pixc2 malloc succeeded\n");
  }

  pixc2 = pixc2_0;

  prob_status = PROB_INITIALIZING;

  /* read_eeprom() 内部会通过 read_EEPROM_byte() 从 g_heimann_eeprom_cache[] 取数据 */
  read_eeprom(heimann_fd);

  gradscale_div = pow(2, gradscale);
  vddscgrad_div = pow(2, vddscgrad);
  vddscoff_div = pow(2, vddscoff);

  calcPixC();

  prob_status = PROB_PREPARING;

  printf("Heimann app calibration init success: id=0x%08x, mbit=0x%02x, bias=0x%02x, clk=0x%02x, bpa=0x%02x, pu=0x%02x\n",
         id, mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib);

  return 0;
}

/*
 * 更新热成像画面。
 * 新版本：应用层只通过 /dev/heimann0 读取 raw frame，
 * EEPROM 从同一设备的 ioctl 获取，温度计算仍放在应用层。
 */
void* thermal_thread(void *arg)
{
  static uint64_t thermal_frame_id = 0;
  static uint64_t thermal_last_print_ts = 0;
  static int thermal_fps_cnt = 0;

  pthread_setname_np(pthread_self(), "thermal");
  pid_t tid = syscall(SYS_gettid);
  printf("thermal_thread start, tid=%d\n", tid);

  thread_context_t* ctx = (thread_context_t*)arg;
  int argc = ctx->thread_args.argc;
  char **argv = ctx->thread_args.argv;

  int heimann_fd = -1;
  ssize_t ret;

  if (argc < 2)
  {
    printf("Wrong use!\n");
    printf("Usage: %s [heimann-dev] [video-dev]\n", argv[0]);
    return (void*)-1;
  }

  heimann_fd = open(argv[1], O_RDONLY);
  if (heimann_fd < 0)
  {
    printf("Can't open heimann device %s\n", argv[1]);
    perror("open /dev/heimann0 failed");
    return (void*)-1;
  }

  printf("heimann device %s open successfully\n", argv[1]);
  printf("thermal raw frame size expected: %zu bytes\n", sizeof(RAMoutput));

  if (heimann_app_calibration_init(heimann_fd) < 0)
  {
    printf("heimann_app_calibration_init failed\n");
    close(heimann_fd);
    return (void*)-1;
  }

  /*
   * 和内核驱动 open() 中的 ptat_vdd_switch 初始值保持一致。
   * 驱动每次 read() 之后会切换下一帧状态；应用层处理完当前帧后也同步切换。
   */
  switch_ptat_vdd = 0;
  sampled_switch_ptat_vdd = 0;
  picnum = 0;
  flag_min_max_initaled = false;
  T_avg = 0;

  while (!ctx->cmd_req.exit_req)
  {
    uint64_t t_read0 = now_us();

    ret = read(heimann_fd, RAMoutput, sizeof(RAMoutput));

    uint64_t t_read1 = now_us();

    if (ret < 0)
    {
      perror("read /dev/heimann0 failed");
      usleep(10000);
      continue;
    }

    if ((size_t)ret != sizeof(RAMoutput))
    {
      printf("read /dev/heimann0 size mismatch: ret=%zd, expected=%zu\n",
             ret, sizeof(RAMoutput));
      usleep(10000);
      continue;
    }

    /*
     * 关键：告诉原算法当前 read() 返回的这一帧是 PTAT 还是 VDD。
     * sort_data() 会根据 sampled_switch_ptat_vdd 更新 Ta 或 VDD。
     */
    sampled_switch_ptat_vdd = switch_ptat_vdd;
    picnum++;

    /* 每帧重新统计温度范围，避免沿用上一帧极值。 */
    flag_min_max_initaled = false;
    T_avg = 0;

    sort_data();
    calculate_pixel_temp();

    /* 当前帧处理完后，准备下一帧 PTAT/VDD 状态。 */
    if (DevConst.PTATVDDSwitch)
      switch_ptat_vdd ^= 1;

    thermal_frame_id++;
    thermal_fps_cnt++;

    uint64_t thermal_ts = now_us();

    pthread_mutex_lock(&ctx->thermal_buf.mutex);
    memcpy(ctx->thermal_buf.thermal_data, data_pixel, sizeof(ctx->thermal_buf.thermal_data));
    ctx->thermal_buf.meta.frame_id = thermal_frame_id;
    ctx->thermal_buf.meta.ts_us = thermal_ts;
    ctx->thermal_buf.updated = 1;
    pthread_cond_signal(&ctx->thermal_buf.cond);
    pthread_mutex_unlock(&ctx->thermal_buf.mutex);

    /* 每 1 秒打印一次状态，避免刷屏。 */
    if (thermal_last_print_ts == 0 || (t_read1 - thermal_last_print_ts) >= 1000000ULL)
    {
      double cost_ms = (double)(t_read1 - t_read0) / 1000.0;
      double fps = 0.0;
      uint16_t center = data_pixel[PIXEL_PER_COLUMN / 2][PIXEL_PER_ROW / 2];

      if (thermal_last_print_ts != 0)
        fps = (double)thermal_fps_cnt * 1000000.0 / (double)(t_read1 - thermal_last_print_ts);

      printf("[THERMAL] frame=%llu read=%zd cost=%.2f ms fps=%.2f sampled=%u next=%u picnum=%u "
             "RAM4=%02x %02x %02x %02x RAM5=%02x %02x %02x %02x "
             "Ta=%u ptat=%u vdd=%u Tmin=%u Tmax=%u center=%u first=%02x %02x %02x %02x\n",
             (unsigned long long)thermal_frame_id,
             ret,
             cost_ms,
             fps,
             sampled_switch_ptat_vdd,
             switch_ptat_vdd,
             picnum,
             RAMoutput[4][0], RAMoutput[4][1], RAMoutput[4][2], RAMoutput[4][3],
             RAMoutput[5][0], RAMoutput[5][1], RAMoutput[5][2], RAMoutput[5][3],
             Ta,
             ptat_av_uint16,
             vdd_av_uint16,
             T_min,
             T_max,
             center,
             RAMoutput[0][0], RAMoutput[0][1], RAMoutput[0][2], RAMoutput[0][3]);

      thermal_last_print_ts = t_read1;
      thermal_fps_cnt = 0;
      fflush(stdout);
    }

    if (ctx->cmd_req.print_eeprom_header_req)
    {
      print_eeprom_header();
      ctx->cmd_req.print_eeprom_header_req = 0;
    }

    if (ctx->cmd_req.print_eeprom_hex_req)
    {
      /* print_eeprom_hex() 内部会调用 read_EEPROM_byte()，现在读的是 EEPROM cache。 */
      print_eeprom_hex(heimann_fd);
      ctx->cmd_req.print_eeprom_hex_req = 0;
    }

    usleep(10000);
  }

  if (pixc2_0)
  {
    free(pixc2_0);
    pixc2_0 = NULL;
    pixc2 = NULL;
  }

  close(heimann_fd);

  return NULL;
}
