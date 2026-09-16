#include "perf_stats.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int compare_u64(const void *lhs, const void *rhs)
{
    const uint64_t a = *(const uint64_t *)lhs;
    const uint64_t b = *(const uint64_t *)rhs;
    return (a > b) - (a < b);
}

static uint64_t percentile(const uint64_t *sorted, size_t count, unsigned percent)
{
    if (count == 0) {
        return 0;
    }

    const size_t nearest_rank = (count * percent + 99) / 100;
    size_t index = nearest_rank > 0 ? nearest_rank - 1 : 0;
    if (index >= count) {
        index = count - 1;
    }
    return sorted[index];
}

void perf_metric_init(perf_metric_t *metric, const char *name)
{
    memset(metric, 0, sizeof(*metric));
    metric->name = name;
}

void perf_metric_record(perf_metric_t *metric, uint64_t elapsed_us)
{
    metric->samples_us[metric->write_index] = elapsed_us;
    metric->write_index = (metric->write_index + 1) % PERF_SAMPLE_WINDOW;
    if (metric->window_count < PERF_SAMPLE_WINDOW) {
        metric->window_count++;
    }

    metric->total_count++;
    metric->total_us += elapsed_us;
    if (elapsed_us > metric->max_us) {
        metric->max_us = elapsed_us;
    }
}

void perf_metric_print(const perf_metric_t *metric)
{
    if (metric->window_count == 0) {
        return;
    }

    uint64_t sorted[PERF_SAMPLE_WINDOW];
    memcpy(sorted, metric->samples_us, metric->window_count * sizeof(sorted[0]));
    qsort(sorted, metric->window_count, sizeof(sorted[0]), compare_u64);

    const double lifetime_avg_ms =
        (double)metric->total_us / (double)metric->total_count / 1000.0;

    printf("[PERF] %-16s count=%" PRIu64
           " avg=%7.3f ms p50=%7.3f ms p95=%7.3f ms p99=%7.3f ms max=%7.3f ms\n",
           metric->name ? metric->name : "unnamed",
           metric->total_count,
           lifetime_avg_ms,
           percentile(sorted, metric->window_count, 50) / 1000.0,
           percentile(sorted, metric->window_count, 95) / 1000.0,
           percentile(sorted, metric->window_count, 99) / 1000.0,
           metric->max_us / 1000.0);
}
