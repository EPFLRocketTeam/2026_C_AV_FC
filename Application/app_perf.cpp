#include "Application/app_perf.h"

#if APP_PERF_TRACE

#include "Application/app_printf.h"
#include "Application/app_timebase.h"

namespace {

struct SectionStats {
	uint32_t max_us = 0;
};

struct LoopStats {
	uint32_t count = 0;
	uint64_t sum_us = 0;
	uint32_t max_us = 0;
	uint32_t over_5ms = 0;
	uint32_t over_16ms = 0;
	uint32_t over_64ms = 0;
};

SectionStats g_sections[APP_PERF_SECTION_COUNT];
LoopStats g_loop;
uint64_t g_last_mark_us = 0;

const char *const kSectionNames[APP_PERF_SECTION_COUNT] = {
	"shell", "periph", "sdlog", "sd", "imu", "baro", "kal", "fsm",
	"radio", "can", "report",
};

} // namespace

extern "C" uint64_t app_perf_begin(void) {
	return app_timebase_now_us();
}

extern "C" void app_perf_end(app_perf_section_t section, uint64_t t0) {
	if (section >= APP_PERF_SECTION_COUNT) {
		return;
	}
	const uint64_t dt = app_timebase_now_us() - t0;
	const uint32_t dt32 = (dt > UINT32_MAX) ? UINT32_MAX : (uint32_t) dt;
	if (dt32 > g_sections[section].max_us) {
		g_sections[section].max_us = dt32;
	}
}

extern "C" void app_perf_loop_mark(void) {
	const uint64_t now = app_timebase_now_us();
	if (g_last_mark_us != 0u) {
		const uint64_t dt = now - g_last_mark_us;
		const uint32_t dt32 = (dt > UINT32_MAX) ? UINT32_MAX : (uint32_t) dt;
		++g_loop.count;
		g_loop.sum_us += dt32;
		if (dt32 > g_loop.max_us) g_loop.max_us = dt32;
		if (dt32 > 5000u) ++g_loop.over_5ms;
		if (dt32 > 16000u) ++g_loop.over_16ms;
		if (dt32 > 64000u) ++g_loop.over_64ms;
	}
	g_last_mark_us = now;
}

extern "C" void app_perf_print(void) {
	const uint32_t avg = (g_loop.count > 0u)
			? (uint32_t) (g_loop.sum_us / g_loop.count) : 0u;
	// One line, one write: each printf is a USB transfer that can wait.
	char line[320];
	int n = snprintf(line, sizeof(line),
			"[PERF] ms=%lu it=%lu avg=%luus max=%luus >5ms=%lu >16ms=%lu >64ms=%lu |",
			(unsigned long)app_timebase_now_ms(), (unsigned long) g_loop.count, (unsigned long) avg,
			(unsigned long) g_loop.max_us, (unsigned long) g_loop.over_5ms,
			(unsigned long) g_loop.over_16ms, (unsigned long) g_loop.over_64ms);
	for (int i = 0; i < APP_PERF_SECTION_COUNT; ++i) {
		if (n > 0 && n < (int) sizeof(line)) {
			n += snprintf(line + n, sizeof(line) - n, " %s=%lu",
					kSectionNames[i], (unsigned long) g_sections[i].max_us);
		}
		g_sections[i] = SectionStats{};
	}
	g_loop = LoopStats{};
	app_printf("%s\r\n", line);
}

#endif
