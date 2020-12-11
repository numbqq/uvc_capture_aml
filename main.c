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

#include "vfm_grabber.h"

struct buffer_mapping
{
	void* start;
	size_t length;
};

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

#define GE2D_DEVICE_NODE	"/dev/ge2d"
#define ION_DEVICE_NODE		"/dev/ion"
#define VFM_GRABBER_NODE	"/dev/vfm_grabber"
#define FB_DEVICE_NODE		"/dev/fb0"

#define MJPEG_DHT_LENGTH	0x1A4

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
int ge2d_fd = -1;
int ion_fd = -1;
int fb_fd = -1;
int vfm_grabber_fd = -1;
IONMEM_AllocParams ionmem_params = { 0 };
int ionmem_handle_fd = -1;
int is_running = 1;

int raw_data_dest_length = 0;
void *raw_data_dest_ptr = NULL;
unsigned long raw_data_dest_phy_addr = 0;

const char *device = DEFAULT_DEVICE;
const char *output = DEFAULT_OUTPUT;
int width = DEFAULT_WIDTH;
int height = DEFAULT_HEIGHT;
int fps = DEFAULT_FRAME_RATE;
int bitrate = DEFAULT_BITRATE;

struct buffer_mapping buffer_mappings[BUFFER_COUNT];

int is_preview = 0;
int is_encoding = 0;

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
	codec_context.am_sysinfo.param = (void*)(0); //EXTERNAL_PTS | SYNC_OUTSIDE

	ret = codec_init(&codec_context);
	if (ret != 0)
	{
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

int write_codec_data(unsigned char* data, int data_length)
{
	int offset = 0;
	while (offset < data_length)
	{
		int count = codec_write(&codec_context, data + offset, data_length - offset);
		if (count > 0)
		{
			offset += count;
		}
		else
		{
			return errno;
		}
	}

	return 0;
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
	int format = 1;
	// Encode the video frames
	vl_frame_type_t type = FRAME_TYPE_AUTO;
	unsigned char* in = (unsigned char*)encode_buffer;
	int in_size = 0;
	unsigned char* out = encode_bitstream_buffer;

	int out_count = vl_video_encoder_encode(handle, type, in, in_size, out, format);

	if (out_count > 0)
	{
		ssize_t write_count = write(encoder_file_fd, encode_bitstream_buffer, out_count);
		if (write_count < 0)
		{
			printf("write bitstream failed.\n");
		}
	}
}

void write_to_file(const char* path, const char* value)
{
	int fd = open(path, O_RDWR | O_TRUNC, 0644);
	if (fd < 0)
	{
		printf("write_to_file open failed: %s = %s\n", path, value);
		return;
	}

	if (write(fd, value, strlen(value)) < 0)
	{
		printf("write_to_file write failed: %s = %s\n", path, value);
	}

	close(fd);
}

void set_vfm_state(void)
{
	write_to_file("/sys/class/vfm/map", "rm default");
	write_to_file("/sys/class/vfm/map", "add default decoder vfm_grabber");
}

void reset_vfm_state(void)
{
	write_to_file("/sys/class/vfm/map", "rm default");
	write_to_file("/sys/class/vfm/map", "add default decoder ppmgr deinterlace amvideo");
}

int open_device_node(const char *path, int *pfd)
{
	if (NULL == path || NULL == pfd)
		return -1;

	int fd = open(path, O_RDWR);
	if (fd < 0)
	{
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
	is_running = 0;
}

void* video_decoder_thread(void* argument)
{
	int ret;
	// Create codec
	ret = open_codec(width, height, 60);
	if (ret < 0) {
		printf("open codec failed.\n");
		return NULL;
	}

	// Start streaming
	int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	ret = ioctl(capture_fd, VIDIOC_STREAMON, &buffer_type);
	if (ret < 0)
	{
		printf("VIDIOC_STREAMON failed.\n");
		close_codec();
		return NULL;
	}

	char is_first_frame = 1;

	while (is_running)
	{
		char needs_mJpeg_dht = 1;
		// get buffer
		struct v4l2_buffer buffer = { 0 };
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;

		ret = ioctl(capture_fd, VIDIOC_DQBUF, &buffer);
		if (ret < 0)
		{
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
					needs_mJpeg_dht = 0;
					break;
				}
					++scan;
			}

			is_first_frame = 0;

			printf("needsMjpegDht = %d\n", needs_mJpeg_dht);
		}

retry:
		if (needs_mJpeg_dht)
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
			int ret;
			ret = write_codec_data(data, header_length);
			if (ret == EAGAIN) {
				reset_codec();
				goto retry;
			}

			// Send DHT
			ret = write_codec_data(mjpeg_dht, MJPEG_DHT_LENGTH);
			if (ret == EAGAIN) {
				reset_codec();
				goto retry;
			}

			// Send remaining data
			write_codec_data(sos, data_length - header_length);
			if (ret == EAGAIN) {
				reset_codec();
				goto retry;
			}
//			printf("data_length=%lu, found SOS @ %d\n", data_length, header_length);
		}
		else
		{
			int ret = write_codec_data(data, data_length);
			if (ret == EAGAIN) {
				reset_codec();
				goto retry;
			}
		}

		// return buffer
		ret = ioctl(capture_fd, VIDIOC_QBUF, &buffer);
		if (ret < 0)
		{
			printf("VIDIOC_QBUF failed.\n");
			close_codec();
			return NULL;
		}
	}

	close_codec();
	printf("%s exit!\n", __func__);
}


int main(int argc, char** argv)
{
	int ret;
	int frames = 0;
	float total_time = 0;
	struct timeval time_start, time_end;
	void *retval;

	int c;
	while ((c = getopt_long(argc, argv, "d:w:h:f:b:peo:H", longopts, NULL)) != -1)
	{
		switch (c)
		{
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
	if (ret < 0)
	{
		printf("capture device open failed.\n");
		exit(1);
	}

	struct v4l2_capability caps = { 0 };
	ret = ioctl(capture_fd, VIDIOC_QUERYCAP, &caps);
	if (ret < 0)
	{
		printf("VIDIOC_QUERYCAP failed.\n");
		close_device_node(capture_fd);
		exit(1);
	}

	printf("card = %s\n", (char*)caps.card);
	printf("\tbus_info = %s\n", (char*)caps.bus_info);
	printf("\tdriver = %s\n", (char*)caps.driver);

	if (!caps.capabilities & V4L2_CAP_VIDEO_CAPTURE)
	{
		printf("V4L2_CAP_VIDEO_CAPTURE not supported.\n");
		close_device_node(capture_fd);
		exit(1);
	}
	else
	{
		printf("V4L2_CAP_VIDEO_CAPTURE supported.\n");
	}

	struct v4l2_fmtdesc format_desc = { 0 };
	format_desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	printf("Supported formats:\n");
	while (1)
	{
		ret = ioctl(capture_fd, VIDIOC_ENUM_FMT, &format_desc);
		if (ret < 0)
		{
			break;
		}

		printf("\tdescription = %s, pixelformat=0x%x\n", format_desc.description, format_desc.pixelformat);

		struct v4l2_frmsizeenum format_size = { 0 };
		format_size.pixel_format = format_desc.pixelformat;

		while (1)
		{
			ret = ioctl(capture_fd, VIDIOC_ENUM_FRAMESIZES, &format_size);
			if (ret < 0)
			{
				break;
			}

			printf("\t\twidth = %d, height = %d\n", format_size.discrete.width, format_size.discrete.height);

			struct v4l2_frmivalenum frameInterval = { 0 };
			frameInterval.pixel_format = format_size.pixel_format;
			frameInterval.width = format_size.discrete.width;
			frameInterval.height = format_size.discrete.height;

			while (1)
			{
				ret = ioctl(capture_fd, VIDIOC_ENUM_FRAMEINTERVALS, &frameInterval);
				if (ret < 0)
				{
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
	if (ret < 0)
	{
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
	if (ret < 0)
	{
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
	if (ret < 0)
	{
		printf("VIDIOC_REQBUFS failed.\n");
		close_device_node(capture_fd);
		exit(1);
	}

	for (int i = 0; i < request_buffers.count; ++i)
	{
		struct v4l2_buffer buffer = { 0 };
		buffer.type = request_buffers.type;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;

		ret = ioctl(capture_fd, VIDIOC_QUERYBUF, &buffer);
		if (ret < 0)
		{
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
	for (int i = 0; i < request_buffers.count; ++i)
	{
		struct v4l2_buffer buffer = { 0 };
		buffer.index = i;
		buffer.type = request_buffers.type;
		buffer.memory = request_buffers.memory;

		ret = ioctl(capture_fd, VIDIOC_QBUF, &buffer);
		if (ret < 0)
		{
			printf("VIDIOC_QBUF failed.\n");
			close_device_node(capture_fd);
			exit(1);
		}
	}

	// GE2D
	ret = open_device_node(GE2D_DEVICE_NODE, &ge2d_fd);
	if (ret < 0)
	{
		printf("open ge2d failed.\n");
		close_device_node(capture_fd);
		exit(1);
	}

	// RGBA
	raw_data_dest_length = width * height * 4;

	// ION
	ion_fd = ion_mem_init();
	if (ion_fd < 0)
	{
		printf("open ion failed.\n");
		goto close_ge2d_fd;
	}
	printf("ion_fd: %d\n", ion_fd);

	ret = ion_mem_alloc(ion_fd, raw_data_dest_length, &ionmem_params, true);
	if (ret < 0)
	{
		printf("ion_mem_alloc: not enough memory.\n");
		goto close_ion_fd;
	}

	ionmem_handle_fd = ionmem_params.mImageFd;
	printf("ionmem_handle_fd: %d\n", ionmem_handle_fd);

	raw_data_dest_ptr = (char *)mmap(NULL, raw_data_dest_length,
			PROT_READ | PROT_WRITE, MAP_SHARED, ionmem_handle_fd, 0);
	if (!raw_data_dest_ptr)
	{
		printf("mmap failed,Not enough memory.\n");
		goto close_ion_fd;
	}

	printf("raw_data_dest_ptr: 0x%p, raw_data_dest_length: %d\n", raw_data_dest_ptr, raw_data_dest_length);

	// Get the physical address for the ion buffer
	struct meson_phys_data phys_data = { 0 };
	phys_data.handle = ionmem_params.mImageFd;

	struct ion_custom_data ion_custom_data = { 0 };
	ion_custom_data.cmd = ION_IOC_MESON_PHYS_ADDR;
	ion_custom_data.arg = (long unsigned int)&phys_data;

	ret = ioctl(ion_fd, ION_IOC_CUSTOM, &ion_custom_data);
	if (ret != 0)
	{
		printf("ION_IOC_CUSTOM failed (%d).\n", ret);
		goto unmap_ion_mem;
	}
	raw_data_dest_phy_addr = phys_data.phys_addr;

	printf("ion phys_addr=0x%lx\n", raw_data_dest_phy_addr);

	set_vfm_state();

	ret = open_device_node(VFM_GRABBER_NODE, &vfm_grabber_fd);
	if (ret < 0)
	{
		printf("open vfm_grabber failed.\n");
		goto unmap_ion_mem;
	}

	if (1 == is_encoding && 0 == is_preview)
	{
		int fd_out;
		if (strcmp(output, "-") == 0)
		{
			fd_out = 1; //stdout
		}
		else
		{
			mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;

			fd_out = open(output, O_CREAT | O_TRUNC | O_WRONLY, mode);
			if (fd_out < 0)
			{
				printf("open output failed.\n");
				goto close_vfmgrabber_fd;
			}
		}

		encoder_file_fd = fd_out;

		// encoder
		handle = open_encoder(format.fmt.pix.width, format.fmt.pix.height,
							(int)((double)stream_parm.parm.capture.timeperframe.denominator / (double)stream_parm.parm.capture.timeperframe.numerator),
							bitrate,
							10);

		if (handle < 0)
		{
			printf("open encoder failed.\n");
			goto close_encoder_file_fd;
		}

		encode_buffer = (unsigned char*)raw_data_dest_ptr;
		encode_bitstream_buffer = malloc(width * height * 4);
		if (NULL == encode_bitstream_buffer)
		{
			printf("malloc failed!\n");
			goto close_encoder;
		}
		printf("encode_bitstream_buffer: %p, %d\n", encode_bitstream_buffer, width * height * 4);
	}

	pthread_t thread;
	int result_code = pthread_create(&thread, NULL, video_decoder_thread, NULL);
	if (result_code != 0)
	{
		printf("pthread_create failed.\n");
		goto free;
	}

	ret = open_device_node(FB_DEVICE_NODE, &fb_fd);
	if (ret < 0)
	{
		printf("open fb failed.\n");
		goto join_thread;
	}

	struct fb_var_screeninfo var_info;
	ret = ioctl(fb_fd, FBIOGET_VSCREENINFO, &var_info);
	if (ret < 0)
	{
		printf("FBIOGET_VSCREENINFO failed.\n");
		goto close_fb_fd;
	}

	printf("var_info.xres = %d\n", var_info.xres);
	printf("var_info.yres = %d\n", var_info.yres);
	printf("var_info.xres_virtual = %d\n", var_info.xres_virtual);
	printf("var_info.yres_virtual = %d\n", var_info.yres_virtual);

	int current_buffer = 0;

	gettimeofday(&time_start, 0);
	while (is_running)
	{
		vfm_grabber_frameinfo vfmInfo;

		ret = ioctl(vfm_grabber_fd, VFM_GRABBER_GRAB_FRAME, &vfmInfo);
		if (ret < 0)
		{
			printf("ret=%d\n", ret);
			continue;
		}

		struct config_para_s config = { 0 };

		if (is_preview)
			config.src_dst_type = ALLOC_OSD0;
		else
			config.src_dst_type = ALLOC_ALLOC;
		config.alu_const_color = 0xffffffff;

		config.src_format = GE2D_FORMAT_M24_YUV420;
		config.src_planes[0].addr = vfmInfo.canvas0plane0.addr;
		config.src_planes[0].w = vfmInfo.canvas0plane0.width;
		config.src_planes[0].h = vfmInfo.canvas0plane0.height;
		config.src_planes[1].addr = vfmInfo.canvas0plane1.addr;
		config.src_planes[1].w = vfmInfo.canvas0plane1.width;
		config.src_planes[1].h = vfmInfo.canvas0plane1.height;
		config.src_planes[2].addr = vfmInfo.canvas0plane2.addr;
		config.src_planes[2].w = vfmInfo.canvas0plane2.width;
		config.src_planes[2].h = vfmInfo.canvas0plane2.height;

		if (1 == is_encoding && 0 == is_preview)
			config.dst_format = GE2D_LITTLE_ENDIAN | GE2D_FORMAT_M24_NV21;
		else
			config.dst_format = GE2D_LITTLE_ENDIAN | GE2D_FORMAT_S24_BGR; // FIXME RGB?
		config.dst_planes[0].addr = raw_data_dest_phy_addr;
		config.dst_planes[0].w = width;
		config.dst_planes[0].h = height;

		if (1 == is_encoding && 0 == is_preview)
		{
			config.dst_planes[1].addr = config.dst_planes[0].addr + (format.fmt.pix.width * format.fmt.pix.height);
			config.dst_planes[1].w = width;
			config.dst_planes[1].h = height / 2;
		}

		ret = ioctl(ge2d_fd, GE2D_CONFIG, &config);
		if (ret < 0)
		{
			printf("GE2D_CONFIG failed.: %d\n", ret);
			goto close_fb_fd;
		}

		// Perform the blit operation
		struct ge2d_para_s blit_rect = { 0 };

		blit_rect.src1_rect.x = 0;
		blit_rect.src1_rect.y = 0;
		blit_rect.src1_rect.w = width;
		blit_rect.src1_rect.h = height;

		blit_rect.dst_rect.x = 0;
		if (is_preview)
		{
			blit_rect.dst_rect.y = var_info.yres * current_buffer;
			blit_rect.dst_rect.w = var_info.xres;
			blit_rect.dst_rect.h = var_info.yres;
		}
		else
		{
			blit_rect.dst_rect.y = 0;
			blit_rect.dst_rect.w = width;
			blit_rect.dst_rect.h = height;
		}

		ret = ioctl(ge2d_fd, GE2D_STRETCHBLIT_NOALPHA, &blit_rect);
		if (ret < 0)
		{
			printf("GE2D_BLIT_NOALPHA failed.\n");
			goto close_fb_fd;
		}

		if (0 == is_preview && 0 == is_encoding)
		{
			// capture one RGB24 frame
			static int only_once = 1;
			if (only_once)
			{
				only_once = 0;
				int fd = open("test.rgb", O_RDWR | O_CREAT | O_TRUNC, 0666);
				if (fd)
				{
					ret = write(fd, raw_data_dest_ptr, width * height * 3);
					close(fd);
					printf("Save one RGBA image done!\n");
					// Exit
					is_running = 0;
				}
			}
		}

		ret = ioctl(vfm_grabber_fd, VFM_GRABBER_HINT_INVALIDATE, 0);
		if (ret < 0)
		{
			printf("VFM_GRABBER_HINT_INVALIDATE failed.\n");
			goto close_fb_fd;
		}

		if (is_preview)
		{
			var_info.yoffset = var_info.yres * current_buffer;
			ioctl(fb_fd, FBIOPAN_DISPLAY, &var_info);
			ioctl(fb_fd, FBIO_WAITFORVSYNC, 0);

			++current_buffer;
			current_buffer %= MAX_SCREEN_BUFFERS;
			if (current_buffer >= MAX_SCREEN_BUFFERS)
			{
				current_buffer = 0;
			}
		}

		// Syncronize the destination data
		ret = ion_sync_fd(ion_fd, ionmem_handle_fd);
		if (ret != 0)
		{
			printf("ion_sync_fd failed.\n");
			goto close_fb_fd;
		}

		if (1 == is_encoding && 0 == is_preview)
			encode_frame();

		// Measure FPS
		++frames;

		gettimeofday(&time_end, 0);
		total_time += (float)((time_end.tv_sec - time_start.tv_sec) + (time_end.tv_usec - time_start.tv_usec) / 1000.0f / 1000.0f);
		gettimeofday(&time_start, 0);

		if (total_time >= 1.0f)
		{
			int fps = (int)(frames / total_time);
			fprintf(stderr, "FPS: %i\n", fps);

			frames = 0;
			total_time = 0;
		}
	}

close_fb_fd:
	close_device_node(fb_fd);

join_thread:
	pthread_join(thread, &retval);

free:
	if (1 == is_encoding && 0 == is_preview)
		free(encode_bitstream_buffer);

close_encoder:
	if (1 == is_encoding && 0 == is_preview)
		close_encoder(handle);

close_encoder_file_fd:
	if (1 == is_encoding && 0 == is_preview)
		close(encoder_file_fd);

close_vfmgrabber_fd:
	close_device_node(vfm_grabber_fd);

unmap_ion_mem:
	munmap(raw_data_dest_ptr, raw_data_dest_length);

close_ion_fd:
	ion_mem_exit(ion_fd);

close_ge2d_fd:
	close_device_node(ge2d_fd);
	close_device_node(capture_fd);

	reset_vfm_state();

	printf("main exit ..\n");

	return ret;
}
