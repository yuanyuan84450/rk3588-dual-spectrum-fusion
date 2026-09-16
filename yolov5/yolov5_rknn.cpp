#include "yolov5_rknn.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "public_cfg.h"
#include "rga_preprocess.h"
#include "rknn_api.h"

namespace {

constexpr float kObjectThreshold = 0.25f;
constexpr float kNmsThreshold = 0.45f;
constexpr int kInputWidth = 640;
constexpr int kInputHeight = 640;
constexpr int kClassCount = 80;
constexpr uint32_t kRgbInputBytes = kInputWidth * kInputHeight * 3;

const char *kClassNames[kClassCount] = {
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable",
    "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

struct Object {
    cv::Rect bbox;
    int label;
    float score;
};

struct LetterboxInfo {
    float scale = 1.0f;
    int pad_x = 0;
    int pad_y = 0;
};

struct DetectorState {
    rknn_context context = 0;
    std::vector<uint8_t> model;
    rknn_input_output_num io_num{};
    std::vector<rknn_tensor_attr> input_attrs;
    std::vector<rknn_tensor_attr> output_attrs;
    rknn_tensor_attr io_input_attr{};
    std::vector<rknn_tensor_attr> io_output_attrs;
    rknn_tensor_mem *input_mem = nullptr;
    std::vector<rknn_tensor_mem *> output_mems;
    bool input_mem_bound = false;
    bool output_mems_bound = false;
    bool use_rga = true;
    bool use_io_mem = true;
    bool initialized = false;
    bool warned_multi_output = false;
};

DetectorState g_detector;

bool environment_flag_is_set(const char *name)
{
    const char *value = std::getenv(name);
    return value && std::strcmp(value, "0") != 0 && value[0] != '\0';
}

bool load_binary_file(const char *path, std::vector<uint8_t> *data)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        std::cerr << "Failed to open RKNN model: " << path << std::endl;
        return false;
    }
    const std::streamsize size = file.tellg();
    if (size <= 0) {
        std::cerr << "RKNN model is empty or unreadable: " << path << std::endl;
        return false;
    }
    data->resize(static_cast<size_t>(size));
    file.seekg(0, std::ios::beg);
    if (!file.read(reinterpret_cast<char *>(data->data()), size)) {
        std::cerr << "Failed to read complete RKNN model: " << path << std::endl;
        data->clear();
        return false;
    }
    return true;
}

void print_tensor_attr(const char *kind, const rknn_tensor_attr &attr)
{
    std::cout << "[RKNN] " << kind << '[' << attr.index << "] name=" << attr.name
              << " dims=";
    for (uint32_t i = 0; i < attr.n_dims; ++i) {
        std::cout << attr.dims[i] << (i + 1 == attr.n_dims ? "" : "x");
    }
    std::cout << " size=" << attr.size << " type=" << get_type_string(attr.type)
              << " fmt=" << get_format_string(attr.fmt)
              << " qnt=" << get_qnt_type_string(attr.qnt_type)
              << " zp=" << attr.zp << " scale=" << attr.scale << std::endl;
}

float iou(const cv::Rect &a, const cv::Rect &b)
{
    const int left = std::max(a.x, b.x);
    const int top = std::max(a.y, b.y);
    const int right = std::min(a.x + a.width, b.x + b.width);
    const int bottom = std::min(a.y + a.height, b.y + b.height);
    const float intersection = static_cast<float>(std::max(0, right - left) *
                                                   std::max(0, bottom - top));
    const float union_area = static_cast<float>(a.area() + b.area()) - intersection;
    return union_area > 0.0f ? intersection / union_area : 0.0f;
}

void nms(std::vector<Object> *objects)
{
    std::sort(objects->begin(), objects->end(),
              [](const Object &a, const Object &b) { return a.score > b.score; });
    std::vector<bool> removed(objects->size(), false);
    for (size_t i = 0; i < objects->size(); ++i) {
        if (removed[i]) continue;
        for (size_t j = i + 1; j < objects->size(); ++j) {
            if (!removed[j] && (*objects)[i].label == (*objects)[j].label &&
                iou((*objects)[i].bbox, (*objects)[j].bbox) > kNmsThreshold) {
                removed[j] = true;
            }
        }
    }
    std::vector<Object> kept;
    kept.reserve(objects->size());
    for (size_t i = 0; i < objects->size(); ++i) {
        if (!removed[i]) kept.push_back((*objects)[i]);
    }
    *objects = std::move(kept);
}

cv::Mat make_letterbox_rgb(const cv::Mat &image, LetterboxInfo *info)
{
    info->scale = std::min(static_cast<float>(kInputWidth) / image.cols,
                           static_cast<float>(kInputHeight) / image.rows);
    const int resized_width = std::max(1, static_cast<int>(std::round(image.cols * info->scale)));
    const int resized_height = std::max(1, static_cast<int>(std::round(image.rows * info->scale)));
    info->pad_x = (kInputWidth - resized_width) / 2;
    info->pad_y = (kInputHeight - resized_height) / 2;

    cv::Mat resized;
    if (resized_width == image.cols && resized_height == image.rows) {
        resized = image;
    } else {
        cv::resize(image, resized, cv::Size(resized_width, resized_height), 0, 0, cv::INTER_LINEAR);
    }

    cv::Mat letterbox(kInputHeight, kInputWidth, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(letterbox(cv::Rect(info->pad_x, info->pad_y, resized_width, resized_height)));
    cv::Mat rgb;
    cv::cvtColor(letterbox, rgb, cv::COLOR_BGR2RGB);
    return rgb;
}

std::vector<Object> decode_flat_output(const float *output, int element_count,
                                       const LetterboxInfo &letterbox,
                                       int image_width, int image_height)
{
    constexpr int kValuesPerDetection = kClassCount + 5;
    std::vector<Object> objects;
    for (int i = 0; i < element_count / kValuesPerDetection; ++i) {
        const float *values = output + i * kValuesPerDetection;
        if (values[4] < kObjectThreshold) continue;

        int class_id = -1;
        float class_probability = 0.0f;
        for (int class_index = 0; class_index < kClassCount; ++class_index) {
            if (values[5 + class_index] > class_probability) {
                class_probability = values[5 + class_index];
                class_id = class_index;
            }
        }
        const float confidence = values[4] * class_probability;
        if (class_id < 0 || confidence < kObjectThreshold) continue;

        const float center_x = (values[0] - letterbox.pad_x) / letterbox.scale;
        const float center_y = (values[1] - letterbox.pad_y) / letterbox.scale;
        const float width = values[2] / letterbox.scale;
        const float height = values[3] / letterbox.scale;
        Object object;
        object.bbox = cv::Rect(static_cast<int>(center_x - width * 0.5f),
                               static_cast<int>(center_y - height * 0.5f),
                               static_cast<int>(width), static_cast<int>(height)) &
                      cv::Rect(0, 0, image_width, image_height);
        object.label = class_id;
        object.score = confidence;
        if (object.bbox.area() > 0) objects.push_back(object);
    }
    return objects;
}

void draw_objects(cv::Mat *image, const std::vector<Object> &objects)
{
    for (const Object &object : objects) {
        cv::rectangle(*image, object.bbox, cv::Scalar(0, 255, 0), 2);
        const std::string text = cv::format("%s: %.2f", kClassNames[object.label], object.score);
        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        const int x = object.bbox.x;
        const int y = std::max(0, object.bbox.y - text_size.height);
        cv::rectangle(*image, cv::Rect(x, y, text_size.width, text_size.height + baseline),
                      cv::Scalar(0, 255, 0), cv::FILLED);
        cv::putText(*image, text, cv::Point(x, y + text_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}

void initialize_accelerated_io()
{
    g_detector.use_rga = !environment_flag_is_set("RKNN_DISABLE_RGA");
    const bool force_io_mem = environment_flag_is_set("RKNN_FORCE_IO_MEM");
    const bool integer_model = g_detector.input_attrs[0].type == RKNN_TENSOR_UINT8 ||
                               g_detector.input_attrs[0].type == RKNN_TENSOR_INT8;
    g_detector.use_io_mem = !environment_flag_is_set("RKNN_DISABLE_IO_MEM") &&
                            (integer_model || force_io_mem);
    if (!g_detector.use_io_mem && !environment_flag_is_set("RKNN_DISABLE_IO_MEM")) {
        std::cout << "[RKNN] FP16 input detected: I/O Memory is auto-disabled because "
                     "A/B testing showed conversion moved into rknn_run; set "
                     "RKNN_FORCE_IO_MEM=1 only for diagnostics" << std::endl;
    }

    g_detector.io_input_attr = g_detector.input_attrs[0];
    g_detector.io_input_attr.type = RKNN_TENSOR_UINT8;
    g_detector.io_input_attr.fmt = RKNN_TENSOR_NHWC;
    g_detector.io_input_attr.pass_through = 0;

    const uint32_t input_allocation =
        std::max(kRgbInputBytes, g_detector.io_input_attr.size_with_stride);
    g_detector.input_mem = rknn_create_mem(g_detector.context, input_allocation);
    if (!g_detector.input_mem) {
        std::cerr << "[RKNN] input DMA allocation failed; using legacy CPU input" << std::endl;
        g_detector.use_rga = false;
        g_detector.use_io_mem = false;
        return;
    }

    if (g_detector.use_io_mem) {
        int ret = rknn_set_io_mem(g_detector.context, g_detector.input_mem,
                                  &g_detector.io_input_attr);
        if (ret == RKNN_SUCC) {
            g_detector.input_mem_bound = true;
        } else {
            std::cerr << "[RKNN] input I/O memory binding failed, ret=" << ret
                      << "; keeping DMA memory as RGA staging" << std::endl;
        }
    }

    // The current model has one flat FP16 output. Bind a persistent FP32 output
    // buffer so post-processing does not allocate and convert on every frame.
    if (g_detector.use_io_mem && g_detector.io_num.n_output == 1) {
        g_detector.io_output_attrs = g_detector.output_attrs;
        g_detector.io_output_attrs[0].type = RKNN_TENSOR_FLOAT32;
        const uint32_t output_bytes =
            g_detector.io_output_attrs[0].n_elems * sizeof(float);
        rknn_tensor_mem *output_mem = rknn_create_mem(g_detector.context, output_bytes);
        if (output_mem) {
            int ret = rknn_set_io_mem(g_detector.context, output_mem,
                                      &g_detector.io_output_attrs[0]);
            if (ret == RKNN_SUCC) {
                g_detector.output_mems.push_back(output_mem);
                g_detector.output_mems_bound = true;
            } else {
                std::cerr << "[RKNN] output I/O memory binding failed, ret=" << ret
                          << "; using rknn_outputs_get" << std::endl;
                rknn_destroy_mem(g_detector.context, output_mem);
            }
        }
    }

    std::cout << "[RKNN] backend rga=" << (g_detector.use_rga ? "on" : "off")
              << " input_io_mem=" << (g_detector.input_mem_bound ? "bound" : "legacy")
              << " output_io_mem=" << (g_detector.output_mems_bound ? "bound" : "legacy")
              << " input_dma_fd=" << g_detector.input_mem->fd << std::endl;
}

int run_preprocessed_inference(const cv::Mat &display_image,
                               const LetterboxInfo &letterbox,
                               const void *input_data,
                               bool data_is_in_input_mem,
                               uint64_t total_begin_us,
                               cv::Mat *result,
                               yolov5_timing_t *measured)
{
    const uint64_t input_begin_us = now_us();
    int ret = RKNN_SUCC;
    if (g_detector.input_mem_bound) {
        if (!data_is_in_input_mem) {
            std::memcpy(g_detector.input_mem->virt_addr, input_data, kRgbInputBytes);
        }
        ret = rknn_mem_sync(g_detector.context, g_detector.input_mem,
                            RKNN_MEMORY_SYNC_TO_DEVICE);
    } else {
        rknn_input input{};
        input.index = 0;
        input.buf = data_is_in_input_mem ? g_detector.input_mem->virt_addr
                                         : const_cast<void *>(input_data);
        input.size = kRgbInputBytes;
        input.type = RKNN_TENSOR_UINT8;
        input.fmt = RKNN_TENSOR_NHWC;
        input.pass_through = 0;
        ret = rknn_inputs_set(g_detector.context, 1, &input);
    }
    measured->input_set_us = now_us() - input_begin_us;
    if (ret != RKNN_SUCC) {
        std::cerr << "RKNN input preparation failed, ret=" << ret << std::endl;
        return -1;
    }

    const uint64_t inference_begin_us = now_us();
    ret = rknn_run(g_detector.context, nullptr);
    measured->inference_us = now_us() - inference_begin_us;
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_run failed, ret=" << ret << std::endl;
        return -1;
    }

    const float *output_data = nullptr;
    int element_count = 0;
    std::vector<rknn_output> outputs;
    const uint64_t output_begin_us = now_us();
    if (g_detector.output_mems_bound) {
        ret = rknn_mem_sync(g_detector.context, g_detector.output_mems[0],
                            RKNN_MEMORY_SYNC_FROM_DEVICE);
        if (ret == RKNN_SUCC) {
            output_data = static_cast<const float *>(g_detector.output_mems[0]->virt_addr);
            element_count = static_cast<int>(g_detector.io_output_attrs[0].n_elems);
        }
    } else {
        outputs.resize(g_detector.io_num.n_output);
        for (rknn_output &output : outputs) {
            std::memset(&output, 0, sizeof(output));
            output.want_float = 1;
        }
        ret = rknn_outputs_get(g_detector.context, g_detector.io_num.n_output,
                               outputs.data(), nullptr);
        if (ret == RKNN_SUCC) {
            output_data = reinterpret_cast<const float *>(outputs[0].buf);
            element_count = static_cast<int>(outputs[0].size / sizeof(float));
        }
    }
    measured->output_get_us = now_us() - output_begin_us;
    if (ret != RKNN_SUCC || !output_data) {
        std::cerr << "RKNN output preparation failed, ret=" << ret << std::endl;
        return -1;
    }

    const uint64_t postprocess_begin_us = now_us();
    if (g_detector.io_num.n_output != 1 && !g_detector.warned_multi_output) {
        std::cerr << "[RKNN] warning: model has " << g_detector.io_num.n_output
                  << " outputs; the legacy decoder currently decodes output[0] only" << std::endl;
        g_detector.warned_multi_output = true;
    }
    std::vector<Object> objects = decode_flat_output(output_data, element_count,
                                                      letterbox, display_image.cols,
                                                      display_image.rows);
    nms(&objects);
    display_image.copyTo(*result);
    draw_objects(result, objects);
    measured->postprocess_us = now_us() - postprocess_begin_us;

    if (!outputs.empty()) {
        rknn_outputs_release(g_detector.context, g_detector.io_num.n_output, outputs.data());
    }
    measured->total_us = now_us() - total_begin_us;
    return 0;
}

}  // namespace

int yolov5_init(const char *model_path)
{
    if (g_detector.initialized) return 0;
    const uint64_t begin_us = now_us();
    if (!model_path || !load_binary_file(model_path, &g_detector.model)) return -1;

    int ret = rknn_init(&g_detector.context, g_detector.model.data(),
                        static_cast<uint32_t>(g_detector.model.size()), 0, nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_init failed, ret=" << ret << std::endl;
        g_detector.model.clear();
        return -1;
    }

    rknn_sdk_version version{};
    if (rknn_query(g_detector.context, RKNN_QUERY_SDK_VERSION, &version, sizeof(version)) == RKNN_SUCC) {
        std::cout << "[RKNN] API=" << version.api_version << " driver=" << version.drv_version << std::endl;
    }

    ret = rknn_query(g_detector.context, RKNN_QUERY_IN_OUT_NUM,
                     &g_detector.io_num, sizeof(g_detector.io_num));
    if (ret != RKNN_SUCC || g_detector.io_num.n_input != 1 || g_detector.io_num.n_output == 0) {
        std::cerr << "Invalid RKNN tensor count, ret=" << ret
                  << " inputs=" << g_detector.io_num.n_input
                  << " outputs=" << g_detector.io_num.n_output << std::endl;
        yolov5_deinit();
        return -1;
    }

    g_detector.input_attrs.resize(g_detector.io_num.n_input);
    for (uint32_t i = 0; i < g_detector.io_num.n_input; ++i) {
        rknn_tensor_attr &attr = g_detector.input_attrs[i];
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;
        ret = rknn_query(g_detector.context, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "RKNN input attribute query failed, index=" << i << " ret=" << ret << std::endl;
            yolov5_deinit();
            return -1;
        }
        print_tensor_attr("input", attr);
        if (attr.type != RKNN_TENSOR_UINT8 && attr.type != RKNN_TENSOR_INT8) {
            std::cout << "[RKNN] notice: model input is " << get_type_string(attr.type)
                      << "; UINT8 input requires runtime conversion in rknn_inputs_set" << std::endl;
        }

        rknn_tensor_attr native_attr{};
        native_attr.index = i;
        if (rknn_query(g_detector.context, RKNN_QUERY_NATIVE_INPUT_ATTR,
                       &native_attr, sizeof(native_attr)) == RKNN_SUCC) {
            print_tensor_attr("native_input", native_attr);
        }
    }

    g_detector.output_attrs.resize(g_detector.io_num.n_output);
    for (uint32_t i = 0; i < g_detector.io_num.n_output; ++i) {
        rknn_tensor_attr &attr = g_detector.output_attrs[i];
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;
        ret = rknn_query(g_detector.context, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "RKNN output attribute query failed, index=" << i << " ret=" << ret << std::endl;
            yolov5_deinit();
            return -1;
        }
        print_tensor_attr("output", attr);

        rknn_tensor_attr native_attr{};
        native_attr.index = i;
        if (rknn_query(g_detector.context, RKNN_QUERY_NATIVE_OUTPUT_ATTR,
                       &native_attr, sizeof(native_attr)) == RKNN_SUCC) {
            print_tensor_attr("native_output", native_attr);
        }
    }

    initialize_accelerated_io();
    g_detector.initialized = true;
    std::cout << "[RKNN] initialized once in " << (now_us() - begin_us) / 1000.0
              << " ms, model bytes=" << g_detector.model.size() << std::endl;
    return 0;
}

void yolov5_deinit()
{
    if (g_detector.context != 0) {
        for (rknn_tensor_mem *memory : g_detector.output_mems) {
            if (memory) rknn_destroy_mem(g_detector.context, memory);
        }
        if (g_detector.input_mem) {
            rknn_destroy_mem(g_detector.context, g_detector.input_mem);
        }
        rknn_destroy(g_detector.context);
    }
    g_detector = DetectorState{};
}

bool yolov5_is_initialized()
{
    return g_detector.initialized;
}

int yolov5_detect(const cv::Mat &image, cv::Mat &result, yolov5_timing_t *timing)
{
    yolov5_timing_t measured{};
    const uint64_t total_begin_us = now_us();
    if (!g_detector.initialized || image.empty()) return -1;

    const uint64_t preprocess_begin_us = now_us();
    LetterboxInfo letterbox;
    bool prepared_in_input_mem = false;
    cv::Mat input_rgb;

    if (g_detector.use_rga && g_detector.input_mem) {
        rga_letterbox_t rga_letterbox{};
        prepared_in_input_mem = rga_letterbox_bgr_to_rgb(
            image.data, image.cols, image.rows,
            static_cast<int>(image.step / image.elemSize()),
            g_detector.input_mem->fd, kInputWidth, kInputHeight,
            &rga_letterbox);
        if (prepared_in_input_mem) {
            letterbox.scale = rga_letterbox.scale;
            letterbox.pad_x = rga_letterbox.pad_x;
            letterbox.pad_y = rga_letterbox.pad_y;
        } else {
            std::cerr << "[RGA] disabling backend after preprocessing failure" << std::endl;
            g_detector.use_rga = false;
        }
    }

    if (!prepared_in_input_mem) {
        input_rgb = make_letterbox_rgb(image, &letterbox);
    }
    measured.preprocess_us = now_us() - preprocess_begin_us;

    const void *input_data = prepared_in_input_mem ? g_detector.input_mem->virt_addr
                                                    : input_rgb.data;
    int ret = run_preprocessed_inference(image, letterbox, input_data,
                                         prepared_in_input_mem, total_begin_us,
                                         &result, &measured);
    if (timing) *timing = measured;
    return ret;
}

int yolov5_detect_nv12_dmabuf(int dma_fd,
                              int width, int height,
                              int width_stride, int height_stride,
                              const cv::Mat &display_image,
                              cv::Mat &result,
                              yolov5_timing_t *timing)
{
    yolov5_timing_t measured{};
    const uint64_t total_begin_us = now_us();
    if (!g_detector.initialized || !g_detector.use_rga ||
        !g_detector.input_mem || display_image.empty()) {
        return -1;
    }

    const uint64_t preprocess_begin_us = now_us();
    rga_letterbox_t rga_letterbox{};
    if (!rga_letterbox_nv12_to_rgb(dma_fd, width, height,
                                   width_stride, height_stride,
                                   g_detector.input_mem->fd,
                                   kInputWidth, kInputHeight,
                                   &rga_letterbox)) {
        return -1;
    }
    measured.preprocess_us = now_us() - preprocess_begin_us;

    LetterboxInfo letterbox;
    letterbox.scale = rga_letterbox.scale;
    letterbox.pad_x = rga_letterbox.pad_x;
    letterbox.pad_y = rga_letterbox.pad_y;
    int ret = run_preprocessed_inference(display_image, letterbox,
                                         g_detector.input_mem->virt_addr, true,
                                         total_begin_us, &result, &measured);
    if (timing) *timing = measured;
    return ret;
}
