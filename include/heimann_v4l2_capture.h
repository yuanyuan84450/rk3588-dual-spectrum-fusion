#ifndef HEIMANN_V4L2_CAPTURE_H
#define HEIMANN_V4L2_CAPTURE_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

typedef struct heimann_v4l2_capture heimann_v4l2_capture_t;

int heimann_v4l2_capture_open(const char *device,
                              heimann_v4l2_capture_t **out_capture);
ssize_t heimann_v4l2_capture_frame(heimann_v4l2_capture_t *capture,
                                   void *destination, size_t destination_size,
                                   uint64_t *timestamp_us,
                                   uint32_t *sequence);
void heimann_v4l2_capture_close(heimann_v4l2_capture_t *capture);

#endif
