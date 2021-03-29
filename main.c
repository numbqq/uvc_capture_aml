#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <getopt.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>

#include <codec.h>
#include <vpcodec_1_0.h>
#include <linux/meson_ion.h>
#include <linux/ge2d.h>
#include <linux/fb.h>
#include <ge2d_port.h>
#include <aml_ge2d.h>

#include <ion/ion.h>
#include <ion/IONmem.h>
#include <linux/ion.h>
#include <Amsysfsutils.h>
#include <amvideo.h>

#define MESON_BUFFER_SIZE 4

struct buffer_mapping
{
	void* start;
	size_t length;
};

struct out_buffer_t {
	int index;
	int size;
	bool own_by_v4l;
	void *ptr;
	IONMEM_AllocParams buffer;
	unsigned long phy_addr;
} vbuffer[MESON_BUFFER_SIZE];

const size_t EXTERNAL_PTS = 0x01;
const size_t SYNC_OUTSIDE = 0x02;
const size_t USE_IDR_FRAMERATE = 0x04;
const size_t UCODE_IP_ONLY_PARAM = 0x08;
const size_t MAX_REFER_BUF = 0x10;
const size_t ERROR_RECOVERY_MODE_IN = 0x20;

#define DEFAULT_DEVICE		"/dev/video0"
#define BUFFER_COUNT		4

#define DEFAULT_OUTPUT		"default.h264"

#define MAX_SCREEN_BUFFERS 2

#define DEFAULT_WIDTH		640
#define DEFAULT_HEIGHT		480
#define DEFAULT_FRAME_RATE	30
#define DEFAULT_BITRATE		(1000000 * 5)

#define ION_DEVICE_NODE		"/dev/ion"
#define FB_DEVICE_NODE		"/dev/fb0"

#define MJPEG_DHT_LENGTH    0x1A4

unsigned char mjpeg_dht[MJPEG_DHT_LENGTH] = {
    /* JPEG DHT Segment for YCrCb omitted from MJPG data */
    0xFF,0xC4,0x01,0xA2,
    0x00,0x00,0x01,0x05,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x01,0x00,0x03,0x01,0x01,0x01,0x01,
    0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
    0x08,0x09,0x0A,0x0B,0x10,0x00,0x02,0x01,0x03,0x03,0x02,0x04,0x03,0x05,0x05,0x04,0x04,0x00,
    0x00,0x01,0x7D,0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,
    0x07,0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,0x24,
    0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,0x29,0x2A,0x34,
    0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x53,0x54,0x55,0x56,
    0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,
    0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,
    0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,
    0xBA,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,
    0xDA,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,
    0xF8,0xF9,0xFA,0x11,0x00,0x02,0x01,0x02,0x04,0x04,0x03,0x04,0x07,0x05,0x04,0x04,0x00,0x01,
    0x02,0x77,0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
    0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xA1,0xB1,0xC1,0x09,0x23,0x33,0x52,0xF0,0x15,0x62,
    0x72,0xD1,0x0A,0x16,0x24,0x34,0xE1,0x25,0xF1,0x17,0x18,0x19,0x1A,0x26,0x27,0x28,0x29,0x2A,
    0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x53,0x54,0x55,0x56,
    0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,
    0x79,0x7A,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,
    0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,
    0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,
    0xD9,0xDA,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,
    0xF9,0xFA
};


struct option longopts[] = {
	{ "device",         required_argument,  NULL,   'd' },
	{ "width",          required_argument,  NULL,   'w' },
	{ "height",         required_argument,  NULL,   'h' },
	{ "fps",            required_argument,  NULL,   'f' },
	{ "bitrate",        required_argument,  NULL,   'b' },
	{ "preview",        no_argument,        NULL,   'p' },
	{ "encode",         no_argument,        NULL,   'e' },
	{ "output",         required_argument,  NULL,   'o' },
	{ "help",           no_argument,        NULL,   'H' },
	{ 0, 0, 0, 0 }
};

codec_para_t codec_context;

int capture_fd = -1;
int ion_fd = -1;
int fb_fd = -1;
int is_running = 1;

const char *device = DEFAULT_DEVICE;
const char *output = DEFAULT_OUTPUT;
int width = DEFAULT_WIDTH;
int height = DEFAULT_HEIGHT;
int fps = DEFAULT_FRAME_RATE;
int bitrate = DEFAULT_BITRATE;

struct buffer_mapping buffer_mappings[BUFFER_COUNT];

struct fb_var_screeninfo var_info;

// amvideo
struct amvideo_dev *amvideo;

// ge2d
aml_ge2d_t amlge2d;

int is_preview = 0;
int is_encoding = 0;

pthread_t thread;

// encoder
vl_codec_handle_t handle;
int encoder_file_fd = -1;
unsigned char* encode_buffer = NULL;
unsigned char* encode_bitstream_buffer = NULL;

int open_codec(int width, int height, int fps)
{
	int ret;
	// Initialize the codec
	memset(&codec_context, 0, sizeof(codec_context));

	codec_context.stream_type = STREAM_TYPE_ES_VIDEO;
	codec_context.video_type = VFORMAT_MJPEG;
	codec_context.has_video = 1;
	codec_context.noblock = 0;
	codec_context.am_sysinfo.format = VIDEO_DEC_FORMAT_MJPEG;
	codec_context.am_sysinfo.width = width;
	codec_context.am_sysinfo.height = height;
	codec_context.am_sysinfo.rate = (96000.0 / fps);
	codec_context.am_sysinfo.param = (void*)(EXTERNAL_PTS | SYNC_OUTSIDE);

	ret = codec_init(&codec_context);
	if (ret != 0) {
		printf("codec_init failed.\n");
	}

	return ret;
}

void reset_codec(void)
{
	codec_reset(&codec_context);
}

void close_codec(void)
{
	codec_close(&codec_context);
}

vl_codec_handle_t open_encoder(int width, int height, int fps, int bitrate, int gop)
{
	// Initialize the encoder
	vl_codec_id_t codec_id = CODEC_ID_H264;

	printf("vl_video_encoder_init: width=%d, height=%d, fps=%d, bitrate=%d, gop=%d\n",
	width, height, fps, bitrate, gop);

	vl_img_format_t img_format = IMG_FMT_YV12;
	handle = vl_video_encoder_init(codec_id, width, height, fps, bitrate, gop, img_format);
	printf("encoder handle = %ld\n", handle);

	return handle;
}

int close_encoder(vl_codec_handle_t handle)
{
	vl_video_encoder_destory(handle);
}

void encode_frame(void)
{
	int format = 3; // RGB24
	// Encode the video frames
	vl_frame_type_t type = FRAME_TYPE_AUTO;
	unsigned char* in = (unsigned char*)encode_buffer;
	int in_size = 0;
	unsigned char* out = encode_bitstream_buffer;

	int out_count = vl_video_encoder_encode(handle, type, in, in_size, out, format);

	if (out_count > 0) {
		ssize_t write_count = write(encoder_file_fd, encode_bitstream_buffer, out_count);
		if (write_count < 0) {
			printf("write bitstream failed.\n");
		}
	}
}

void set_vfm_state(void)
{
	amsysfs_set_sysfs_str("/sys/class/vfm/map", "rm default");
	amsysfs_set_sysfs_str("/sys/class/vfm/map", "add default decoder ionvideo");
}

void reset_vfm_state(void)
{
	amsysfs_set_sysfs_str("/sys/class/vfm/map", "rm default");
	amsysfs_set_sysfs_str("/sys/class/vfm/map", "add default decoder ppmgr deinterlace amvideo");
}

void free_buffers(void)
{
    int i;
    for (i = 0; i < MESON_BUFFER_SIZE; i++) {
        if (vbuffer[i].ptr) {
            munmap(vbuffer[i].ptr, vbuffer[i].size);
			ion_free(ion_fd, vbuffer[i].buffer.mIonHnd);
			close(vbuffer[i].buffer.mImageFd);
			close(ion_fd);
        }
    }
}

int alloc_buffers(int width, int height)
{
    int i = 0;
	int size = 0;
	int ret = -1;
	struct meson_phys_data phy_data;
	struct ion_custom_data custom_data;

	ion_fd = ion_mem_init();
	if (ion_fd < 0) {
		printf("ion_open failed!\n");
		goto fail;
	}
	printf("ion_fd: %d\n", ion_fd);

	size = width * height * 3;

	for (i=0; i<MESON_BUFFER_SIZE; i++) {
		ret = ion_mem_alloc(ion_fd, size, &vbuffer[i].buffer, true);
		if (ret < 0) {
			printf("ion_mem_alloc failed\n");
			free_buffers();
			goto fail;
		}
		vbuffer[i].index = i;
		vbuffer[i].size = size;
		vbuffer[i].ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, vbuffer[i].buffer.mImageFd, 0);

		phy_data.handle = vbuffer[i].buffer.mImageFd;
		phy_data.phys_addr = 0;
		phy_data.size = 0;
		custom_data.cmd = ION_IOC_MESON_PHYS_ADDR;
		custom_data.arg = (unsigned long)&phy_data;
		ret = ioctl(ion_fd, ION_IOC_CUSTOM, (unsigned long)&custom_data);
		if (ret < 0) {
			vbuffer[i].phy_addr = 0;
			free_buffers();
			goto fail;
		} else {
			vbuffer[i].phy_addr = phy_data.phys_addr;
		}
    }

fail:
    return ret;
}

static int ionvideo_init(int width, int height)
{
    int i, ret;

    alloc_buffers(width, height);

    amvideo = new_amvideo(FLAGS_V4L_MODE);
    if (!amvideo) {
        printf("amvideo create failed\n");
        ret = -ENODEV;
        goto fail;
    }
    amvideo->display_mode = 0;
    amvideo->use_frame_mode = 0;

    ret = amvideo_init(amvideo, 0, width, height,
            V4L2_PIX_FMT_RGB24, MESON_BUFFER_SIZE);
    if (ret < 0) {
        printf("amvideo_init failed\n");
        amvideo_release(amvideo);
        goto fail;
    }
    ret = amvideo_start(amvideo);
    if (ret < 0) {
        amvideo_release(amvideo);
        goto fail;
    }
    for (i = 0; i < MESON_BUFFER_SIZE; i++) {
        vframebuf_t vf;
        vf.fd = vbuffer[i].buffer.mImageFd;
        vf.length = vbuffer[i].buffer.size;
        vf.index = vbuffer[i].index;
        ret = amlv4l_queuebuf(amvideo, &vf);
    }
fail:
    return ret;
}

void ionvideo_close()
{
	amvideo_stop(amvideo);
	amvideo_release(amvideo);
}

int ge2d_init(int width, int height)
{
	int i;
	int ret;

	memset(&amlge2d, 0, sizeof(aml_ge2d_t));
	memset(&(amlge2d.ge2dinfo.src_info[0]), 0, sizeof(buffer_info_t));
	memset(&(amlge2d.ge2dinfo.src_info[1]), 0, sizeof(buffer_info_t));
	memset(&(amlge2d.ge2dinfo.dst_info), 0, sizeof(buffer_info_t));

	amlge2d.ge2dinfo.src_info[0].canvas_w = width;
	amlge2d.ge2dinfo.src_info[0].canvas_h = height;
	amlge2d.ge2dinfo.src_info[0].format = PIXEL_FORMAT_RGB_888;
	amlge2d.ge2dinfo.src_info[0].plane_number = 1;

	amlge2d.ge2dinfo.dst_info.canvas_w = width;
	amlge2d.ge2dinfo.dst_info.canvas_h = height;
	amlge2d.ge2dinfo.dst_info.format = PIXEL_FORMAT_RGB_888;
	amlge2d.ge2dinfo.dst_info.plane_number = 1;
	amlge2d.ge2dinfo.dst_info.rotation = GE2D_ROTATION_0;
	amlge2d.ge2dinfo.offset = 0;
	amlge2d.ge2dinfo.ge2d_op = AML_GE2D_STRETCHBLIT;
	amlge2d.ge2dinfo.blend_mode = BLEND_MODE_PREMULTIPLIED;

	amlge2d.ge2dinfo.src_info[0].memtype = GE2D_CANVAS_ALLOC;
	amlge2d.ge2dinfo.src_info[1].memtype = GE2D_CANVAS_TYPE_INVALID;
	if (0 == is_preview)
		amlge2d.ge2dinfo.dst_info.memtype = GE2D_CANVAS_ALLOC;
	else
		amlge2d.ge2dinfo.dst_info.memtype = GE2D_CANVAS_OSD0;

	amlge2d.ge2dinfo.src_info[0].mem_alloc_type = AML_GE2D_MEM_INVALID;//AML_GE2D_MEM_DMABUF
	amlge2d.ge2dinfo.src_info[1].mem_alloc_type = AML_GE2D_MEM_INVALID;//AML_GE2D_MEM_ION;
	amlge2d.ge2dinfo.dst_info.mem_alloc_type = AML_GE2D_MEM_ION;

	ret = aml_ge2d_init(&amlge2d);
	if (ret < 0) {
		printf("aml_ge2d_init failed!\n");
		return -1;
	}

	ret = aml_ge2d_mem_alloc(&amlge2d);
	if (ret < 0) {
		printf("aml_ge2d_mem_alloc failed!\n");
		return -1;
	}

	return 0;
}

int ge2d_destroy(void)
{
	int i;

	if (amlge2d.ge2dinfo.dst_info.mem_alloc_type == AML_GE2D_MEM_ION)
		aml_ge2d_invalid_cache(&amlge2d.ge2dinfo);

	for (i = 0; i < amlge2d.ge2dinfo.src_info[0].plane_number; i++) {
		if (amlge2d.src_data[i]) {
			free(amlge2d.src_data[i]);
			amlge2d.src_data[i] = NULL;
		}
	}

	for (i = 0; i < amlge2d.ge2dinfo.src_info[1].plane_number; i++) {
		if (amlge2d.src2_data[i]) {
			free(amlge2d.src2_data[i]);
			amlge2d.src2_data[i] = NULL;
		}
	}

	for (i = 0; i < amlge2d.ge2dinfo.dst_info.plane_number; i++) {
		if (amlge2d.dst_data[i]) {
			free(amlge2d.dst_data[i]);
			amlge2d.dst_data[i] = NULL;
		}
	}

	aml_ge2d_mem_free(&amlge2d);
	aml_ge2d_exit(&amlge2d);
}

int open_device_node(const char *path, int *pfd)
{
	if (NULL == path || NULL == pfd)
		return -1;

	int fd = open(path, O_RDWR);
	if (fd < 0) {
		printf("open %s failed.\n", path);
		return fd;
	}

	*pfd = fd;

	printf("open %s, fd: %d\n", path, *pfd);

	return 0;
}

void close_device_node(int fd)
{
	if (fd > 0)
		close(fd);
}

void signal_handler(int s)
{
	void *retval;
	is_running = 0;
	pthread_join(thread, &retval);
	close_device_node(fb_fd);
	if (1 == is_encoding && 0 == is_preview) {
		free(encode_bitstream_buffer);
		close_encoder(handle);
		close(encoder_file_fd);
	}
	ge2d_destroy();
	ionvideo_close();
	free_buffers();
	close_device_node(capture_fd);
	reset_vfm_state();
	signal(s, SIG_DFL);
	raise(s);
}

void* video_decoder_thread(void* argument)
{
	int ret;
	int current_buffer = 0;
	int i;
	int frames = 0;
	struct timeval time_start, time_end;
	float total_time = 0;
	vframebuf_t vf;
	uint32_t isize;
	// Create codec
	ret = open_codec(width, height, 60);
	if (ret < 0) {
		printf("open codec failed.\n");
		return NULL;
	}

	// Start streaming
	int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	ret = ioctl(capture_fd, VIDIOC_STREAMON, &buffer_type);
	if (ret < 0) {
		printf("VIDIOC_STREAMON failed.\n");
		close_codec();
		return NULL;
	}

	char is_first_frame = 1;

	gettimeofday(&time_start, 0);
	while (is_running) {
		char needs_mjpeg_dht = 1;
		// get buffer
		struct v4l2_buffer buffer = { 0 };
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;

		ret = ioctl(capture_fd, VIDIOC_DQBUF, &buffer);
		if (ret < 0) {
			printf("VIDIOC_DQBUF failed.\n");
			return NULL;
		}

		// MJPEG
		unsigned char* data = (unsigned char*)buffer_mappings[buffer.index].start;
		size_t data_length = buffer.bytesused;

		if (is_first_frame)
		{
			unsigned char* scan = data;
			while (scan < data + data_length - 4)
			{
				if (scan[0] == mjpeg_dht[0] &&
					scan[1] == mjpeg_dht[1] &&
					scan[2] == mjpeg_dht[2] &&
					scan[3] == mjpeg_dht[3])
				{
					needs_mjpeg_dht = 0;
					break;
				}
				++scan;
			}

			is_first_frame = 0;

			printf("needsMjpegDht = %d\n", needs_mjpeg_dht);
		}

		if (needs_mjpeg_dht)
		{
			// Find the start of scan (SOS)
			unsigned char* sos = data;
			while (sos < data + data_length - 1)
			{
				if (sos[0] == 0xff && sos[1] == 0xda)
					break;
				++sos;
			}

			// Send everthing up to SOS
			int header_length = sos - data;

			isize = 0;
			do {
				ret = amlv4l_dequeuebuf(amvideo, &vf);
				if (ret >= 0) {
					//printf("vf idx%d pts 0x%x\n", vf.index, vf.pts);
					ret = amlv4l_queuebuf(amvideo, &vf);
					if (ret < 0) {
						//printf("amlv4l_queuebuf %d\n", ret);
					}
				} else {
					//printf("amlv4l_dequeuebuf %d\n", ret);
					//continue;
				}

				ret = codec_write(&codec_context, data + isize, header_length);
				if (ret < 0) {
					if (errno != EAGAIN) {
						printf("write data failed, errno %d\n", errno);
						return NULL;
					} else {
						continue;
					}
				} else {
					isize += ret;
				}
			} while (isize < header_length);

			// Send DHT
			isize = 0;
			do {
				ret = amlv4l_dequeuebuf(amvideo, &vf);
				if (ret >= 0) {
					//printf("vf idx%d pts 0x%x\n", vf.index, vf.pts);
					ret = amlv4l_queuebuf(amvideo, &vf);
					if (ret < 0) {
						//printf("amlv4l_queuebuf %d\n", ret);
					}
				} else {
					//printf("amlv4l_dequeuebuf %d\n", ret);
					//continue;
				}

				ret = codec_write(&codec_context, mjpeg_dht + isize, MJPEG_DHT_LENGTH);
				if (ret < 0) {
					if (errno != EAGAIN) {
						printf("write data failed, errno %d\n", errno);
						return NULL;
					} else {
						continue;
					}
				} else {
					isize += ret;
				}
			} while (isize < MJPEG_DHT_LENGTH);


			// Send remaining data
			isize = 0;
			do {
				ret = amlv4l_dequeuebuf(amvideo, &vf);
				if (ret >= 0) {
					//printf("vf idx%d pts 0x%x\n", vf.index, vf.pts);
					ret = amlv4l_queuebuf(amvideo, &vf);
					if (ret < 0) {
						//printf("amlv4l_queuebuf %d\n", ret);
					}
				} else {
					//printf("amlv4l_dequeuebuf %d\n", ret);
					//continue;
				}

				ret = codec_write(&codec_context, sos + isize, data_length - header_length);
				if (ret < 0) {
					if (errno != EAGAIN) {
						printf("write data failed, errno %d\n", errno);
						return NULL;
					} else {
						continue;
					}
				} else {
					isize += ret;
				}
			} while (isize < (data_length - header_length));

//			printf("data_length=%lu, found SOS @ %d\n", data_length, header_length);
        } else {
			isize = 0;
			do {
				ret = amlv4l_dequeuebuf(amvideo, &vf);
				if (ret >= 0) {
					//printf("vf idx%d pts 0x%x\n", vf.index, vf.pts);
					ret = amlv4l_queuebuf(amvideo, &vf);
					if (ret < 0) {
						//printf("amlv4l_queuebuf %d\n", ret);
					}
				} else {
					//printf("amlv4l_dequeuebuf %d\n", ret);
					//continue;
				}

				ret = codec_write(&codec_context, (unsigned char*)buffer_mappings[buffer.index].start + isize, buffer.bytesused);
				if (ret < 0) {
					if (errno != EAGAIN) {
						printf("write data failed, errno %d\n", errno);
						return NULL;
					} else {
						continue;
					}
				} else {
					isize += ret;
				}
			} while (isize < buffer.bytesused);
		}

		amlge2d.ge2dinfo.src_info[0].offset[0] = vbuffer[vf.index].phy_addr;

		amlge2d.ge2dinfo.src_info[0].rect.x = 0;
		amlge2d.ge2dinfo.src_info[0].rect.y = 0;
		amlge2d.ge2dinfo.src_info[0].rect.w = amlge2d.ge2dinfo.src_info[0].canvas_w;
		amlge2d.ge2dinfo.src_info[0].rect.h = amlge2d.ge2dinfo.src_info[0].canvas_h;

		amlge2d.ge2dinfo.dst_info.rect.x = 0;
		if (is_preview) {
			amlge2d.ge2dinfo.dst_info.rect.y = var_info.yres * current_buffer;
			amlge2d.ge2dinfo.dst_info.rect.w = var_info.xres;
			amlge2d.ge2dinfo.dst_info.rect.h = var_info.yres;
		} else {
			amlge2d.ge2dinfo.dst_info.rect.y = 0;
			amlge2d.ge2dinfo.dst_info.rect.w = width;
			amlge2d.ge2dinfo.dst_info.rect.h = height;
		}
		amlge2d.ge2dinfo.dst_info.rotation = GE2D_ROTATION_0;
		amlge2d.ge2dinfo.src_info[0].layer_mode = 0;
		amlge2d.ge2dinfo.src_info[0].plane_alpha = 0xff;

		ret = aml_ge2d_process(&amlge2d.ge2dinfo);
		if (ret < 0) {
			printf("aml_ge2d_process failed!\n");
			return NULL;
		}

		// return buffer
		ret = ioctl(capture_fd, VIDIOC_QBUF, &buffer);
		if (ret < 0) {
			printf("VIDIOC_QBUF failed.\n");
			close_codec();
			return NULL;
		}

		// Syncronize the destination data
		ret = ion_sync_fd(ion_fd, vbuffer[vf.index].buffer.mImageFd);
		if (ret != 0) {
			printf("ion_sync_fd failed.\n");
			return NULL;
		}

		if (0 == is_preview && 0 == is_encoding) {
			// capture one RGB24 frame
			static int only_once = 0;
			// drop first image
			if (3 == only_once) {
				only_once = 0;
				int fd = open("test.rgb", O_RDWR | O_CREAT | O_TRUNC, 0666);
				if (fd) {
					ret = write(fd, (unsigned char*)amlge2d.ge2dinfo.dst_info.vaddr[0], width * height * 3);
					close(fd);
					printf("Save one RGBA image done!\n");
					// Exit
					break;
				}
			} else {
				only_once++;
			}
		}

		if (is_preview) {
			var_info.yoffset = var_info.yres * current_buffer;
			ioctl(fb_fd, FBIOPAN_DISPLAY, &var_info);
			ioctl(fb_fd, FBIO_WAITFORVSYNC, 0);

			++current_buffer;
			current_buffer %= MAX_SCREEN_BUFFERS;
			if (current_buffer >= MAX_SCREEN_BUFFERS) {
				current_buffer = 0;
			}
		}

		if (1 == is_encoding && 0 == is_preview) {
			encode_buffer = (unsigned char*)amlge2d.ge2dinfo.dst_info.vaddr[0];

			encode_frame();
		}

		// Measure FPS
		++frames;

		gettimeofday(&time_end, 0);
		total_time += (float)((time_end.tv_sec - time_start.tv_sec) + (time_end.tv_usec - time_start.tv_usec) / 1000.0f / 1000.0f);
		gettimeofday(&time_start, 0);

		if (total_time >= 1.0f) {
			int fps = (int)(frames / total_time);
			fprintf(stderr, "FPS: %i\n", fps);
			frames = 0;
			total_time = 0;
		}

		pthread_testcancel();
	}

	close_codec();
	printf("%s exit!\n", __func__);
}


int main(int argc, char** argv)
{
	int ret;
	void *retval;

	int c;
	while ((c = getopt_long(argc, argv, "d:w:h:f:b:peo:H", longopts, NULL)) != -1) {
		switch (c) {
			case 'd':
				device = optarg;
				break;

			case 'w':
				width = atoi(optarg);
				break;

			case 'h':
				height = atoi(optarg);
				break;

			case 'f':
				fps = atoi(optarg);
				break;

			case 'b':
				bitrate = atoi(optarg);
				break;

			case 'p':
				is_preview = 1;
				break;

			case 'e':
				is_encoding = 1;
				break;

			case 'o':
				output = optarg;
				break;

			case 'H':

			default:
				printf("%s [-d device] [-w width] [-h height] [-f fps] [-b bitrate] [-e] [-o output] [-p] [-H]\n", argv[0]);
				exit(1);
		}
	}

	signal(SIGINT, signal_handler);

	ret = open_device_node(device, &capture_fd);
	if (ret < 0) {
		printf("capture device open failed.\n");
		exit(1);
	}

	struct v4l2_capability caps = { 0 };
	ret = ioctl(capture_fd, VIDIOC_QUERYCAP, &caps);
	if (ret < 0) {
		printf("VIDIOC_QUERYCAP failed.\n");
		close_device_node(capture_fd);
		exit(1);
	}

	printf("card = %s\n", (char*)caps.card);
	printf("\tbus_info = %s\n", (char*)caps.bus_info);
	printf("\tdriver = %s\n", (char*)caps.driver);

	if (!caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
		printf("V4L2_CAP_VIDEO_CAPTURE not supported.\n");
		close_device_node(capture_fd);
		exit(1);
	} else {
		printf("V4L2_CAP_VIDEO_CAPTURE supported.\n");
	}

	struct v4l2_fmtdesc format_desc = { 0 };
	format_desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	printf("Supported formats:\n");
	while (1) {
		ret = ioctl(capture_fd, VIDIOC_ENUM_FMT, &format_desc);
		if (ret < 0) {
			break;
		}

		printf("\tdescription = %s, pixelformat=0x%x\n", format_desc.description, format_desc.pixelformat);

		struct v4l2_frmsizeenum format_size = { 0 };
		format_size.pixel_format = format_desc.pixelformat;

		while (1) {
			ret = ioctl(capture_fd, VIDIOC_ENUM_FRAMESIZES, &format_size);
			if (ret < 0) {
				break;
			}

			printf("\t\twidth = %d, height = %d\n", format_size.discrete.width, format_size.discrete.height);

			struct v4l2_frmivalenum frameInterval = { 0 };
			frameInterval.pixel_format = format_size.pixel_format;
			frameInterval.width = format_size.discrete.width;
			frameInterval.height = format_size.discrete.height;

			while (1) {
				ret = ioctl(capture_fd, VIDIOC_ENUM_FRAMEINTERVALS, &frameInterval);
				if (ret < 0) {
					break;
				}

				printf("\t\t\tnumerator = %d, denominator = %d\n", frameInterval.discrete.numerator, frameInterval.discrete.denominator);
				++frameInterval.index;
			}

			++format_size.index;
		}

		++format_desc.index;
	}

	// Apply capture settings
	struct v4l2_format format = { 0 };
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = width;
	format.fmt.pix.height = height;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	format.fmt.pix.field = V4L2_FIELD_ANY;

	ret = ioctl(capture_fd, VIDIOC_S_FMT, &format);
	if (ret < 0) {
		printf("VIDIOC_S_FMT failed.\n");
		close_device_node(capture_fd);
		exit(1);
	}

	printf("v4l2_format: width=%d, height=%d, pixelformat=0x%x\n",
		format.fmt.pix.width, format.fmt.pix.height, format.fmt.pix.pixelformat);

	// Readback device selected settings
	width = format.fmt.pix.width;
	height = format.fmt.pix.height;

	struct v4l2_streamparm stream_parm = { 0 };
	stream_parm.type = format.type;
	stream_parm.parm.capture.timeperframe.numerator = 1;
	stream_parm.parm.capture.timeperframe.denominator = fps;

	ret = ioctl(capture_fd, VIDIOC_S_PARM, &stream_parm);
	if (ret < 0) {
		printf("VIDIOC_S_PARM failed.\n");
		close_device_node(capture_fd);
		exit(1);
	}

	printf("capture.timeperframe: numerator=%d, denominator=%d\n",
		stream_parm.parm.capture.timeperframe.numerator,
		stream_parm.parm.capture.timeperframe.denominator);

	// Request buffers
	struct v4l2_requestbuffers request_buffers = { 0 };
	request_buffers.count = BUFFER_COUNT;
	request_buffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	request_buffers.memory = V4L2_MEMORY_MMAP;

	ret = ioctl(capture_fd, VIDIOC_REQBUFS, &request_buffers);
	if (ret < 0) {
		printf("VIDIOC_REQBUFS failed.\n");
		close_device_node(capture_fd);
		exit(1);
	}

	for (int i = 0; i < request_buffers.count; ++i) {
		struct v4l2_buffer buffer = { 0 };
		buffer.type = request_buffers.type;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;

		ret = ioctl(capture_fd, VIDIOC_QUERYBUF, &buffer);
		if (ret < 0) {
			printf("VIDIOC_QUERYBUF failed.\n");
			close_device_node(capture_fd);
			exit(1);
		}

		buffer_mappings[i].length = buffer.length;
		buffer_mappings[i].start = mmap(NULL, buffer.length,
			PROT_READ | PROT_WRITE, /* recommended */
			MAP_SHARED,             /* recommended */
			capture_fd, buffer.m.offset);
	}

	// Queue buffers
	for (int i = 0; i < request_buffers.count; ++i) {
		struct v4l2_buffer buffer = { 0 };
		buffer.index = i;
		buffer.type = request_buffers.type;
		buffer.memory = request_buffers.memory;

		ret = ioctl(capture_fd, VIDIOC_QBUF, &buffer);
		if (ret < 0) {
			printf("VIDIOC_QBUF failed.\n");
			close_device_node(capture_fd);
			exit(1);
		}
	}

	set_vfm_state();

	ret = ionvideo_init(width, height);
	if (ret < 0) {
		printf("ionvideo_init failed!\n");
		close_device_node(capture_fd);
		exit(1);
	}

	// GE2D
	ret = ge2d_init(width, height);
	if (ret < 0) {
		printf("ge2d_init failed!\n");
		goto close_ionvideo;
	}

	if (1 == is_encoding && 0 == is_preview) {
		int fd_out;
		if (strcmp(output, "-") == 0) {
			fd_out = 1; //stdout
		} else {
			mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;

			fd_out = open(output, O_CREAT | O_TRUNC | O_WRONLY, mode);
			if (fd_out < 0)
			{
				printf("open output failed.\n");
				goto destroy_ge2d;
			}
		}

		encoder_file_fd = fd_out;

		// encoder
		handle = open_encoder(format.fmt.pix.width, format.fmt.pix.height,
							(int)((double)stream_parm.parm.capture.timeperframe.denominator / (double)stream_parm.parm.capture.timeperframe.numerator),
							bitrate,
							10);

		if (handle < 0) {
			printf("open encoder failed.\n");
			goto close_encoder_file_fd;
		}

		encode_bitstream_buffer = malloc(width * height * 4);
		if (NULL == encode_bitstream_buffer) {
			printf("malloc failed!\n");
			goto close_encoder;
		}
		printf("encode_bitstream_buffer: %p, %d\n", encode_bitstream_buffer, width * height * 4);
	}

	ret = open_device_node(FB_DEVICE_NODE, &fb_fd);
	if (ret < 0) {
		printf("open fb failed.\n");
		goto free;
	}

	ret = ioctl(fb_fd, FBIOGET_VSCREENINFO, &var_info);
	if (ret < 0) {
		printf("FBIOGET_VSCREENINFO failed.\n");
		goto close_fb_fd;
	}

	printf("var_info.xres = %d\n", var_info.xres);
	printf("var_info.yres = %d\n", var_info.yres);
	printf("var_info.xres_virtual = %d\n", var_info.xres_virtual);
	printf("var_info.yres_virtual = %d\n", var_info.yres_virtual);

	ret = pthread_create(&thread, NULL, video_decoder_thread, NULL);
	if (ret != 0) {
		printf("pthread_create failed.\n");
		goto close_fb_fd;
	}

	pthread_join(thread, &retval);

close_fb_fd:
	close_device_node(fb_fd);

free:
	if (1 == is_encoding && 0 == is_preview)
		free(encode_bitstream_buffer);

close_encoder:
	if (1 == is_encoding && 0 == is_preview)
		close_encoder(handle);

close_encoder_file_fd:
	if (1 == is_encoding && 0 == is_preview)
		close(encoder_file_fd);

destroy_ge2d:
	ge2d_destroy();

close_ionvideo:
	ionvideo_close();
	free_buffers();

	close_device_node(capture_fd);

	reset_vfm_state();

	printf("main exit ..\n");

	return ret;
}
