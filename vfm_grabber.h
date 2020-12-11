#pragma once

#include <linux/ioctl.h>
#include <linux/types.h>

typedef unsigned int u32;
typedef unsigned long u64;

typedef struct
{
	u32 index;
	ulong addr;
	u32 width;
	u32 height;
} vfm_grabber_canvasinfo;

typedef struct
{
	u32 index;
	u32 type;
	u32 duration;
	u32 duration_pulldown;
	u32 pts;
	u64 pts_us64;
	u32 flag;

	u32 canvas0Addr;
	u32 canvas1Addr;

	u32 bufWidth;
	u32 width;
	u32 height;
	u32 ratio_control;
	u32 bitdepth;

	vfm_grabber_canvasinfo canvas0plane0;
	vfm_grabber_canvasinfo canvas0plane1;
	vfm_grabber_canvasinfo canvas0plane2;
} vfm_grabber_frameinfo;

// IOCTL defines
#define VFM_GRABBER_GRAB_FRAME			_IOR('V', 0x10, vfm_grabber_frameinfo)
#define VFM_GRABBER_HINT_INVALIDATE		_IO('V', 0x11)

