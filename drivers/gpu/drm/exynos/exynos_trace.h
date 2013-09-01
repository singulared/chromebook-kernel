#if !defined(_EXYNOS_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _EXYNOS_TRACE_H_

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#include <drm/drmP.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM exynos
#define TRACE_SYSTEM_STRING __stringify(TRACE_SYSTEM)
#define TRACE_INCLUDE_FILE exynos_trace

/* object tracking */

TRACE_EVENT(exynos_flip_request,
	TP_PROTO(unsigned int pipe),
	TP_ARGS(pipe),

	TP_STRUCT__entry(
		__field(unsigned int, pipe)
	),

	TP_fast_assign(
		__entry->pipe = pipe;
	),

	TP_printk("pipe=%u", __entry->pipe)
);

TRACE_EVENT(exynos_flip_complete,
	TP_PROTO(unsigned int pipe),
	TP_ARGS(pipe),

	TP_STRUCT__entry(
		__field(unsigned int, pipe)
	),

	TP_fast_assign(
		__entry->pipe = pipe;
	),

	TP_printk("pipe=%u", __entry->pipe)
);

TRACE_EVENT(exynos_fake_flip_complete,
	TP_PROTO(unsigned int pipe),
	TP_ARGS(pipe),

	TP_STRUCT__entry(
		__field(unsigned int, pipe)
	),

	TP_fast_assign(
		__entry->pipe = pipe;
	),

	TP_printk("pipe=%u", __entry->pipe)
);

/*
 * exynos_page_flip_state exposes more detail of frame buffer when GPU is
 * rendering. This allows chrome://tracing to trace the states that a frame
 * buffer passes through while being flipped on particular crtc.
 *
 * The state transition is as follows:
 * (state)            wait_kds --> wait_apply --> wait_flip --> flipped
 *                  ^           ^              ^             ^
 * (trigger)  flip_request  kds_callback  finish_pageflip  vblank
 *
 * TODO(kao): These trace events describe the typical flipping cases,
 * but does not yet accurately describe the various flips scenarios
 * involving mode sets and dpms requests.
 */

TRACE_EVENT(exynos_page_flip_state,
	TP_PROTO(unsigned int pipe, unsigned int fb, const char *state),
	TP_ARGS(pipe, fb, state),

	TP_STRUCT__entry(
		__field(unsigned int, pipe)
		__field(unsigned int, fb)
		__field(const char *, state)
	),

	TP_fast_assign(
		__entry->pipe = pipe;
		__entry->fb = fb;
		__entry->state = state;
	),

	TP_printk("pipe=%u, fb=%u, state=%s",
			__entry->pipe, __entry->fb, __entry->state)
);

#endif /* _EXYNOS_TRACE_H_ */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#include <trace/define_trace.h>

