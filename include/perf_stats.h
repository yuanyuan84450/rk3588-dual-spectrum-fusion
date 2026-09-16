#ifndef PERF_STATS_H
#define PERF_STATS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PERF_SAMPLE_WINDOW 256

typedef struct {
    const char *name;
    uint64_t samples_us[PERF_SAMPLE_WINDOW];
    uint64_t total_count;
    uint64_t total_us;
    uint64_t max_us;
    size_t window_count;
    size_t write_index;
} perf_metric_t;

void perf_metric_init(perf_metric_t *metric, const char *name);
void perf_metric_record(perf_metric_t *metric, uint64_t elapsed_us);
void perf_metric_print(const perf_metric_t *metric);

#ifdef __cplusplus
}
#endif

#endif
