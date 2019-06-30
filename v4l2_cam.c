#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <time.h>

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

/* video frame count */
#define VIDEO_COUNT 3

struct Buffer {
    void *start;
    unsigned int length;
};

static int kbhit(void) {
    fd_set rfds;
    struct timeval tv;
    int retval = 0;

    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    retval = select(1, &rfds, NULL, NULL, &tv);

    if (retval < 0) {
        fprintf(stderr, "select error!\n");
    }

    return retval;
}


int main(int argc, char *argv[]) {
    int fd = 0;
    int ret = 0;
    int i = 0;

    struct v4l2_capability caps;
    struct v4l2_fmtdesc fmtdesc;
    // struct v4l2_crop crop;
    struct v4l2_format fmt;
    // struct v4l2_streamparm parm;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    struct v4l2_frmsizeenum frmsize;
    enum v4l2_buf_type type;
    struct Buffer *buffers;
    struct timespec prev, tp1, tp2;
    long elapsed = 0, interval = 0;

    if (argc != 2) {
        fprintf(stderr, "usage: v4l2_cam device\n");
        return -1;
    }

    /* open video device */
    fd = open(argv[1], O_RDWR);

    if (fd < 0) {
        fprintf(stderr, "open device %s failed, error: %d(%m)",
                            argv[1], errno);
        return -1;
    }

    /* query device capabilities */
    ret = ioctl(fd, VIDIOC_QUERYCAP, &caps);
    if (ret < 0) {
        fprintf(stderr, "query caps failed, error: %d(%m)\n", errno);
        return -1;
    }

    printf("video device capabilities:\n");
    printf("driver: %s\n", caps.driver);
    printf("card: %s\n", caps.card);
    printf("bus_info: %s\n", caps.bus_info);
    printf("version: 0x%u\n", caps.version);
    printf("capabilities: 0x%x\n", caps.capabilities);

    if (V4L2_CAP_VIDEO_CAPTURE & caps.capabilities) {
        printf("video capture\n");
    }

    if (V4L2_CAP_VIDEO_OUTPUT & caps.capabilities) {
        printf("video output\n");
    }

    printf("device_caps: 0x%x\n", caps.device_caps);
    printf("reserved[1]: %u\n", caps.reserved[1]);
    printf("reserved[2]: %u\n", caps.reserved[2]);
    printf("reserved[3]: %u\n", caps.reserved[3]);

    if (!V4L2_CAP_VIDEO_CAPTURE & caps.capabilities) {
        fprintf(stderr, "device doesn't support video capture!! exit...\n");
        return -1;
    }

    printf("enum format:\n");
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    /* enum capture device supported format... */
    while(ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
        printf("\t%d.%s\n", fmtdesc.index+1, fmtdesc.description);
        fmtdesc.index++;
    }
    printf("enum format end:\n");

    printf("enum support frame size...\n");
    while(ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) != -1) {
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            printf("line:%d %dx%d\n",__LINE__, frmsize.discrete.width, frmsize.discrete.height);
        } else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
            printf("line:%d %dx%d\n",__LINE__, frmsize.discrete.width, frmsize.discrete.height);
        }
    }
    printf("enum support frame size end\n");

    /* set capture device parameter */
    // standard, such as pal, ntsc... no need...
    // ioctl(fd, VIDIOC_S_STD, &std_id);

    // no need to set crop...
    // ioctl(fd, VIDIOC_S_CROP, &crop);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.height = IMAGE_HEIGHT;
    fmt.fmt.pix.width = IMAGE_WIDTH;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    // fmt.fmt.pix.field = V4L2_FIELD_ANY;
    ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
    if (ret < 0) {
        fprintf(stderr, "v4l2 set format failed, error: %d(%m)\n", errno);
        return -1;
    }

    printf("======================================================================\n");


    ret = ioctl(fd, VIDIOC_G_FMT, &fmt);
    if (ret < 0) {
        fprintf(stderr, "v4l2 get format failed, error: %d(%m)\n", errno);
        return -1;
    }

    printf("v4l2 get format:\n");
    printf("type: %d\n", fmt.type);
    printf("pixelformat: %c%c%c%c\n", fmt.fmt.pix.pixelformat & 0xFF,
            (fmt.fmt.pix.pixelformat>>8) & 0xFF,
            (fmt.fmt.pix.pixelformat>>16) & 0xFF,
            (fmt.fmt.pix.pixelformat>>24) & 0xFF);
    printf("height: %d\n", fmt.fmt.pix.height);
    printf("width: %d\n", fmt.fmt.pix.width);
    printf("field:%d\n", fmt.fmt.pix.field);

    // set framerate
    // ioctl(fd, VIDIOC_S_PARM)

    // request buffer
    memset(&req, 0, sizeof(req));
    req.count = VIDEO_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // req.memory = V4L2_MEMORY_DMABUF;
    req.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(fd, VIDIOC_REQBUFS, &req);
    if (ret < 0) {
        printf("request buffer failed... errno:%d(%m)\n", errno);
        return -1;
    }

    // query buffer
    ret = ioctl(fd, VIDIOC_QUERYBUF, &buf);
    printf("buffer info:\n");
    printf("index: %d\n", buf.index);
    printf("memory: 0x%x\n", buf.memory);
    printf("offset: 0x%x\n", buf.m.offset);
    printf("length: %d\n", buf.length);

    // calloc video frame's addr
    buffers = calloc(req.count, sizeof(struct Buffer));

    if (!buffers) {
        fprintf(stderr, "Out Of Memory!\n");
        goto malloc_fail;
    }

    for (i=0; i<VIDEO_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            goto query_fail;
        }

        printf("buffer[%d] info:\n", i);
        printf("index: %d\n", buf.index);
        printf("memory: 0x%x\n", buf.memory);
        printf("offset: 0x%x\n", buf.m.offset);
        printf("length: %d\n", buf.length);

        buffers[i].length = buf.length;

        // mmap video buffer(vmalloced) to user space
        buffers[i].start = mmap(NULL, buf.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (MAP_FAILED == buffers[i].start) {
            fprintf(stderr, "mmap %d memory failed!", i);
            goto map_failed;
        }
    }

    // queue 3 buffers
    for (i=0; i<VIDEO_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "queue buffer[%d] failed\n", buf.index);
            goto queue_fail;
        }
    }

    // stream on
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "stream on failed, errno: %d(%m)\n", errno);
        goto streamon_fail;
    }

    memset(&prev, 0, sizeof(prev));
    // while loop to dequeue buffer and show it...
    while (1) {
#if 0
        /* check if stdin keyboard hit */
        if (kbhit() < 0) {
            // do nothing
            printf("keyboard not hit...\n");
        } else {
            printf("'q' for exit!\n");
            if (getchar() == 'q') {
                break;
            } else {
                // do nothing
            }
        }
#endif

        /* get time tp1 */
        clock_gettime(CLOCK_MONOTONIC, &tp1);

        interval = (tp1.tv_sec - prev.tv_sec)*1000
                        + (tp1.tv_nsec - prev.tv_nsec)/(1000*1000);

        if (prev.tv_sec == 0) {
            // do nothing
        } else {
            printf("get picture interval(frame rate?): %ld ms\n", interval);
        }

        prev.tv_sec = tp1.tv_sec;
        prev.tv_nsec = tp1.tv_nsec;

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            fprintf(stderr, "dequeue buffer failed, errno=%d(%m)\n", errno);
            goto dequeue_fail;
        }

        // process the yuv data...
        printf("todo -- process picture[%d] data...\n", buf.index);

        // queue buffer
        if(ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "queue buffer[%d] failed\n", buf.index);
            goto queue_fail;
        }

        clock_gettime(CLOCK_MONOTONIC, &tp2);

        elapsed = (tp2.tv_sec - tp1.tv_sec)*1000 + (tp2.tv_nsec - tp1.tv_nsec)/(1000*1000);

        printf("queue, dequeue totally cost %ld ms\n", elapsed);
    }

    // stream off
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    // unmap memory
    for (i=0; i<VIDEO_COUNT; i++) {
        if (munmap(buffers[i].start, buffers[i].length) < 0) {
            fprintf(stderr, "munmap failed, error=%d(%m)\n", errno);
        }
    }

    // close video device
    close(fd);

    printf("exit main...\n");
    return 0;

queue_fail:
    /* stream off */
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);
streamon_fail:
dequeue_fail:
    /* unmaped the data */
    for (i=0; i<VIDEO_COUNT; i++) {
        if (munmap(buffers[i].start, buffers[i].length) < 0) {
            fprintf(stderr, "munmap failed, error=%d(%m)\n", errno);
        }
    }

map_failed:
    if (buffers) {
        free(buffers);
    }
query_fail:
malloc_fail:
    close(fd);

    return -1;
}
