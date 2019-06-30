#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <assert.h>

#define CBUF_NUM 4

struct frame_info {
    int fd;
    void *start;
    uint32_t length;
    uint32_t width;
    uint32_t height;
    uint32_t bpp;
};

struct video_buffer {
    void *start;
    uint32_t length;
};

struct video_info {
    int fd;
    int width;
    int height;
    int pixelfmt;
    struct video_buffer *bufs;
};

/* rgb32 structure */
typedef struct {
    uint8_t r; // 红色分量
    uint8_t g; // 绿色分量
    uint8_t b; // 蓝色分量
    uint8_t rgbReserved; // 保留字节（用作Alpha通道或忽略）
} rgb32;

/* rgb32 format in frame buffer */
typedef struct {
    uint8_t b;
    uint8_t g;
    uint8_t r;
    uint8_t alpha;
} rgb32_frame;

int open_camera(const char* path) {
    int fd = 0;

    fd = open(path, O_RDWR);

    return fd;
}

static int g_exit = 0;

int init_framebuffer(const char* path, struct frame_info* fb_info) {
    int fr_fd = 0;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    uint32_t screen_size = 0;
    uint32_t frame_bpp = 0;
    void *buf = 0;

    if (path == NULL) {
        fprintf(stderr, "path is invalid...\n");
        return -1;
    }

    fr_fd = open(path, O_RDWR);
    if (fr_fd < 0) {
        fprintf(stderr, "open %s failed\n", path);
        return -1;
    }

    /* get fixed screen information */
    if (ioctl(fr_fd, FBIOGET_FSCREENINFO, &finfo) < 0) {
        fprintf(stderr, "read fixed screen info failed!\n");
        return -1;
    }

    /* get variable screen info */
    if (ioctl(fr_fd, FBIOGET_VSCREENINFO, &vinfo) < 0) {
        fprintf(stderr, "error read variable information!\n");
        return -1;
    }

    screen_size = vinfo.xres_virtual * vinfo.yres_virtual * vinfo.bits_per_pixel/8;
    frame_bpp = vinfo.bits_per_pixel;

    buf = mmap(0, screen_size, PROT_READ|PROT_WRITE, MAP_SHARED, fr_fd,
                            0);

    if (!buf) {
        fprintf(stderr, "mmap failed\n");
        return -1;
    }

    fb_info->fd = fr_fd;
    fb_info->start = buf;
    fb_info->length = screen_size;
    fb_info->width = vinfo.xres_virtual;
    fb_info->height = vinfo.yres_virtual;
    fb_info->bpp = frame_bpp;

    return 0;
}

int get_camera_info(struct video_info* v_info) {
    struct v4l2_capability caps;
    struct v4l2_format fmt;
    struct v4l2_fmtdesc fmtdesc;
    int fd;

    // printf("get_camera_info in\n");

    fd = v_info->fd;

    /* query capability */
    if (ioctl(fd, VIDIOC_QUERYCAP, &caps) < 0) {
        fprintf(stderr, "query capbility failed!\n");
        return -1;
    }

    if (!caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        fprintf(stderr, "device doesn't support capture feature...\n");
        return -1;
    }

    printf("device support capture!\n");

    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_CAP_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        fprintf(stderr, "get format failed\n");
        return -1;
    }

    fmtdesc.index = 0;
    fmtdesc.type = V4L2_CAP_VIDEO_CAPTURE;
    /* get current camera's support formats */
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        if (fmtdesc.pixelformat & fmt.fmt.pix.pixelformat) {
            printf("\tformat:%s\n", fmtdesc.description);
            break;
        }

        fmtdesc.index++;
    }

    // choose the default's format
    v_info->width = fmt.fmt.pix.width;
    v_info->height = fmt.fmt.pix.height;
    v_info->pixelfmt = fmt.fmt.pix.pixelformat;

    // printf("get_camera_info out\n");

    return 0;
}

int set_format(struct video_info* v_info) {
    int fd, ret;
    struct v4l2_format fmt;

    memset(&fmt, 0, sizeof(struct v4l2_format));

    fd = v_info->fd;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = v_info->width;
    fmt.fmt.pix.height = v_info->height;
    fmt.fmt.pix.pixelformat = v_info->pixelfmt;

    ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
    if (ret < 0) {
        printf("VIDIOC_S_FMT failed (%d)\n", ret);
        return -1;
    }

    printf("-------------------VIDIOC_S_FMT---------------------\n");
    printf("Stream Format Informations:\n");
    printf(" type: %d\n", fmt.type);
    printf(" width: %d\n", fmt.fmt.pix.width);
    printf(" height: %d\n", fmt.fmt.pix.height);

    return 0;
}

int req_bufs(struct video_info* v_info) {
    int fd;
    struct v4l2_requestbuffers req;

    fd = v_info->fd;
    memset(&req, 0, sizeof(req));
    req.count = CBUF_NUM;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "VIDIOC_REQBUFS failed! errno:%d(%m)\n", errno);
        return -1;
    }

    return 0;
}

void map_bufs(struct video_info* v_info) {
    int i = 0;
    struct v4l2_buffer tmp_buf;
    int fd;

    fd = v_info->fd;

    v_info->bufs = calloc(CBUF_NUM, sizeof(struct video_buffer));
    assert(v_info->bufs);

    for (i=0; i< CBUF_NUM; i++) {
        memset(&tmp_buf, 0, sizeof(tmp_buf));
        tmp_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        tmp_buf.memory = V4L2_MEMORY_MMAP;
        tmp_buf.index = i;

        // query buffer info
        if (ioctl(fd, VIDIOC_QUERYBUF, &tmp_buf) < 0) {
            fprintf(stderr, "VIDIOC_QUERYBUF (%d) error\n", i);
            return;
        }

        v_info->bufs[i].start = mmap(NULL, tmp_buf.length, PROT_READ|PROT_WRITE, MAP_SHARED,
                fd, tmp_buf.m.offset);
        v_info->bufs[i].length = tmp_buf.length;

        assert(v_info->bufs[i].start);

        if (ioctl(fd, VIDIOC_QBUF, &tmp_buf) < 0) {
            fprintf(stderr, "enqueue video buffer failed! errno: %d(%m)\n", errno);
            return;
        }
    }

    return;
}

void unmap_bufs(struct video_info* v_info) {
    int i;

    for (i=0; i< CBUF_NUM; i++) {
        munmap(v_info->bufs[i].start, v_info->bufs[i].length);
    }

    free(v_info->bufs);
}

int stream_on(struct video_info* v_info) {
    int fd, ret;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    fd = v_info->fd;

    // printf("stream on, fd = 0x%x\n", fd);

    ret = ioctl(fd, VIDIOC_STREAMON, &type);

    if (ret < 0) {
        fprintf(stderr, "stream on failed, error: %d(%m)\n", errno);
    }

    return ret;
}

int stream_off(struct video_info* v_info) {
    int fd, ret;
    int off = 1;

    fd = v_info->fd;

    // printf("stream off, fd = 0x%x\n", fd);

    ret = ioctl(fd, VIDIOC_STREAMOFF, &off);

    if (ret < 0) {
        fprintf(stderr, "stream off failed, error:%d(%m)\n", errno);
    }

    return ret;
}

int get_picture(struct video_info* v_info, uint8_t *buffer) {
    int ret;
    int fd;
    struct v4l2_buffer dequeue;
    struct v4l2_buffer enqueue;

    memset(&dequeue, 0, sizeof(struct v4l2_buffer));
    memset(&enqueue, 0, sizeof(struct v4l2_buffer));

    dequeue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    dequeue.memory = V4L2_MEMORY_MMAP;

    enqueue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    enqueue.memory = V4L2_MEMORY_MMAP;

    fd = v_info->fd;

    /* dequeue buffer */
    ret = ioctl(fd, VIDIOC_DQBUF, &dequeue);

    if (ret < 0) {
        fprintf(stderr, "dequeue video frame failed\n");
        return -1;
    }

    // printf("get picture[%d]\n", dequeue.index);

    // copy yuv data to picture
    memcpy(buffer, v_info->bufs[dequeue.index].start, v_info->bufs[dequeue.index].length);

    enqueue.index = dequeue.index;
    ret = ioctl(fd, VIDIOC_QBUF, &enqueue);

    if (ret < 0) {
        fprintf(stderr, "enqueue failed\n");
        return -2;
    }

    return 0;
}


/* yuv to rgb algorithm*/
static int sign3 = 1;
/*
YUV to RGB's calculation...
R = 1.164*(Y-16) + 1.159*(V-128);
G = 1.164*(Y-16) - 0.380*(U-128)+ 0.813*(V-128);
B = 1.164*(Y-16) + 2.018*(U-128));
*/
int yuvtorgb(int y, int u, int v) {
    uint32_t pixel32 = 0;
    uint8_t *pixel = (uint8_t *)&pixel32;
    int r, g, b;
    static long int ruv, guv, buv;

    if (1 == sign3) {
        sign3 = 0;
        ruv = 1159*(v-128);
        guv = -380*(u-128) + 813*(v-128);
        buv = 2018*(u-128);
    }

    r = (1164*(y-16) + ruv) / 1000;
    g = (1164*(y-16) - guv) / 1000;
    b = (1164*(y-16) + buv) / 1000;

    if(r > 255) r = 255;
    if(g > 255) g = 255;
    if(b > 255) b = 255;
    if(r < 0) r = 0;
    if(g < 0) g = 0;
    if(b < 0) b = 0;

    pixel[0] = r;
    pixel[1] = g;
    pixel[2] = b;

    return pixel32;
}

int yuv2rgb32(uint8_t* yuv, uint8_t* rgb, uint32_t width, uint32_t height) {
    uint32_t in, out;
    int y0, u, y1, v;
    uint32_t pixel32;
    uint8_t *pixel = (uint8_t *)&pixel32;
    /* yuyv, yuv 4:2:2*/
    uint32_t size = width*height*2;

    for (in=0, out=0; in < size; in+=4, out+=8) {
        y0 = yuv[in+0];
        u  = yuv[in+1];
        y1 = yuv[in+2];
        v  = yuv[in+3];

        sign3 = 1;
        pixel32 = yuvtorgb(y0, u, v);
        rgb[out+0] = pixel[0];
        rgb[out+1] = pixel[1];
        rgb[out+2] = pixel[2];
        rgb[out+3] = 0;

        pixel32 = yuvtorgb(y1, u, v);
        rgb[out+4] = pixel[0];
        rgb[out+5] = pixel[1];
        rgb[out+6] = pixel[2];
        rgb[out+7] = 0;
    }

    return 0;
}

void write_frames(uint8_t* img_buf, struct video_info* v_info,
                        struct frame_info* fb_info) {
    int row, column;
    int num = 0;
    rgb32_frame* rgb32_fbp = (rgb32_frame* )fb_info->start;
    rgb32* rgb32_img_buf = (rgb32 *)img_buf;
    uint32_t frame_x = fb_info->width;

    for (row = 0; row < v_info->height; row++) {
        for (column = 0; column < v_info->width; column++) {
            rgb32_fbp[row * frame_x + column].r = rgb32_img_buf[num].r;
            rgb32_fbp[row * frame_x + column].g = rgb32_img_buf[num].g;
            rgb32_fbp[row * frame_x + column].b = rgb32_img_buf[num].b;

            num++;
        }
    }
}

void exit_framebuffer(struct frame_info* fb_info) {
    /* unmap memory */
    munmap(fb_info->start, fb_info->length);

    /* close framebuffer device */
    close(fb_info->fd);
}

void close_camera(int fd) {
    if (fd) {
        close(fd);
    }
}

static void *key_monitor(void* arg) {
    /* 'q' for exit! */
    while (getchar() != 'q' && g_exit != 1) {
        sleep(1);
    }

    g_exit = 1;

    return (void *)0;
}

int main(int argc, char *argv[]) {
    int fd = 0;
    int ret = 0;
    struct frame_info fb_info;
    struct video_info video_info;
    static uint8_t *yuv, *rgb;
    uint64_t screen_size;
    pthread_t pid;
	pthread_attr_t attr;

    if (argc != 3) {
        fprintf(stderr, "usage: v4l2_cam video_device fb_device!\n");
        return -1;
    }

    /* 1. open v4l2 capture device */
    fd = open_camera(argv[1]);
    if (fd < 0) {
        fprintf(stderr, "open camera failed\n");
        return -1;
    }

    video_info.fd = fd;

    /* 2. init frame buffer */
    ret = init_framebuffer(argv[2], &fb_info);
    if (ret < 0) {
        fprintf(stderr, "init framebuffer failed...\n");
        goto fb_fail;
    }

    /* 3. get camera info */
    ret = get_camera_info(&video_info);
    if (ret < 0) {
        fprintf(stderr, "get camera info failed\n");
        goto query_fail;
    }

    /* 4. set camera format */
    set_format(&video_info);

    /* 5. request buffers */
    req_bufs(&video_info);

    /* 6. map memory to user space and enqueue video buffer */
    map_bufs(&video_info);

    /* 7. stream on */
    stream_on(&video_info);

    /* allocate yuv & rgb buffer */
    screen_size = video_info.width*video_info.height;

    printf("screen_size = %ld\n", screen_size);

    yuv = (uint8_t *)calloc(1, screen_size * 2);
    rgb = (uint8_t *)calloc(1, screen_size * 4);

    assert(yuv&&rgb);

    pthread_attr_init(&attr);
	if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) < 0) {
        fprintf(stderr, "set detach state failed\n");
        goto detach_fail;
    }
    /* create pthread to monitor keyboard input */
    if (pthread_create(&pid, &attr, key_monitor, NULL) < 0) {
        fprintf(stderr, "create thread failed! errno: %d(%m)\n", errno);
        goto thread_fail;
    }

    /* 8. while loop to get frame and show pictures... */
    while (!g_exit) {
        // get yuv data
        get_picture(&video_info, yuv);

        // convert yuv data to rgb
        yuv2rgb32(yuv, rgb, video_info.width, video_info.height);

        // write data to frame buffer device
        write_frames(rgb, &video_info, &fb_info);
    }

    /* 9. stream off */
    stream_off(&video_info);

    free(yuv);
    free(rgb);

    /* 10. unmaped the memory */
    unmap_bufs(&video_info);

    /* 11. close frame buffer */
    exit_framebuffer(&fb_info);

    /* 12. close v4l2 capture device */
    close_camera(fd);

    return 0;

thread_fail:
detach_fail:
    /* stream off */
    stream_off(&video_info);

    /* unmaped the memory */
    unmap_bufs(&video_info);

query_fail:
    exit_framebuffer(&fb_info);

fb_fail:

    close_camera(fd);

    return ret;
}
