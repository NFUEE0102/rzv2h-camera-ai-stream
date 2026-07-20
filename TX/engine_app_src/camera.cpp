/***********************************************************************************************************************
* Copyright (C) 2023 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : camera.cpp
* Version      : 1.00
* Description  : for RZ/V2H DRP-AI Sample Application with MIPI/USB Camera
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "camera.h"
#include <errno.h>
#include <sys/mman.h>

Camera::Camera()
{
    camera_width    = CAM_IMAGE_WIDTH;
    camera_height   = CAM_IMAGE_HEIGHT;
    camera_color    = CAM_IMAGE_CHANNEL_YUY2;
}

Camera::~Camera()
{
}

/*****************************************
* Function Name : set_external_capture_buf
* Description   : Design B -- adopt a capture ring slot whose dma-buf is owned by
*                 the capture_encoder child (mmngr, non-cached). mmap gives the CPU view
*                 the app needs to copy the frame into drpai_buf for DRP-AI.
* Arguments     : index, dbuf_fd (received via SCM_RIGHTS), phys_addr, size
* Return value  : 0 ok, -1 otherwise
******************************************/
int8_t Camera::set_external_capture_buf(uint32_t index, int dbuf_fd,
                                        uint32_t phys_addr, uint32_t size)
{
    if (index >= CAP_BUF_NUM) return -1;
    dma_buf[index] = (camera_dma_buffer*)malloc(sizeof(camera_dma_buffer));
    if (!dma_buf[index]) return -1;
    dma_buf[index]->idx     = 0;            /* mmngr id owned by the child */
    dma_buf[index]->dbuf_fd = (uint32_t)dbuf_fd;
    dma_buf[index]->phy_addr = phys_addr;
    dma_buf[index]->size    = size;
    /* Try a CPU mapping so DRP-AI can use the proven copy-to-drpai_buf path. If
     * mmngr dma-bufs can't be mmap'd cross-process, fall back to DRP reading the
     * capture buffer's phys directly (mem == NULL). */
    dma_buf[index]->mem     = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dbuf_fd, 0);
    if (dma_buf[index]->mem == MAP_FAILED)
    {
        fprintf(stderr, "[WARN] mmap external capture buf %u failed: errno=%d -> DRP phys-direct fallback\n",
                index, errno);
        dma_buf[index]->mem = NULL;
    }
    else
    {
        fprintf(stderr, "[main] external capture buf %u mmap'd at %p (CPU readable)\n",
                index, dma_buf[index]->mem);
    }
    m_ext_bufs = true;
    return 0;
}

/*****************************************
* Function Name : format
* Description   : Format specification.
* Arguments     : fmt = an object that represents the format string.
*                 args = arguments to be formatted
* Return value  : A string object holding the formatted result.
******************************************/
template <typename ... Args>
std::string format(const std::string& fmt, Args ... args)
{
    size_t len = std::snprintf(nullptr, 0, fmt.c_str(), args ...);
    std::vector<char> buf(len + 1);
    std::snprintf(&buf[0], len + 1, fmt.c_str(), args ...);
    return std::string(&buf[0], &buf[0] + len);
}

/*****************************************
* Function Name : start_camera
* Description   : Function to initialize USB/MIPI camera capture
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::start_camera()
{
    int8_t ret = 0;
    int32_t n = 0;
    
#if INPUT_CAM_TYPE == 1
    int32_t i = 0;

std::string sw_cmd1 = format("media-ctl -d /dev/media0 -V \"'tevs 4-0048':0 [fmt:UYVY8_1X16/%s field:none]\"", MIPI_CAM_RES);
    std::string sw_cmd2 = format("media-ctl -d /dev/media0 -V \"'csi-16000400.csi20':0 [fmt:UYVY8_1X16/%s field:none]\"", MIPI_CAM_RES);
    std::string sw_cmd3 = format("media-ctl -d /dev/media0 -V \"'cru-ip-16000000.video0':0 [fmt:UYVY8_1X16/%s field:none]\"", MIPI_CAM_RES);
    const char* commands[3] =
    {
        sw_cmd1.c_str(),
        sw_cmd2.c_str(),
        sw_cmd3.c_str(),
    };
    int cmd_count = 3;

    // Execute commands
    for (i = 0; i < cmd_count; i++)
    {
        printf("%s\n", commands[i]);
        ret = system(commands[i]);
        printf("system ret = %d\n", ret);
        if (ret < 0)
        {
            printf("%s: failed media-ctl commands. index = %d\n", __func__, i);
            return -1;
        }
    }
#endif /* INPUT_CAM_TYPE */
    ret = open_camera_device();
    if (0 != ret) 
    {
        printf("failed to open_camera_device\n");
        return ret;
    }

    ret = init_camera_fmt();
    if (0 != ret) 
    {
        printf("failed to init_camera_fmt\n");
        return ret;
    }
    
    ret = init_buffer();
    if (0 != ret) 
    {
        printf("failed to init_buffer\n");
        return ret;
    }

    wayland_buf = (camera_dma_buffer*)malloc(sizeof(camera_dma_buffer));
    ret = video_buffer_alloc_dmabuf(wayland_buf,WAYLANDBUF);
    
    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to Allocate DMA buffer for the wayland_buf\n");
        return -1;
    }

    overlay_buf = (camera_dma_buffer*)malloc(sizeof(camera_dma_buffer));
    ret = video_buffer_alloc_dmabuf(overlay_buf,WAYLANDBUF);
    
    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to Allocate DMA buffer for the overlay_buf\n");
        return -1;
    }
    drpai_buf = (camera_dma_buffer*)malloc(sizeof(camera_dma_buffer));
    ret = video_buffer_alloc_dmabuf(drpai_buf,DRPAIBUF);

    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to Allocate DMA buffer for the drpai_buf\n");
        return -1;
    }
    
    for (n =0; n < CAP_BUF_NUM; n++)
    {
        if (!m_ext_bufs)   /* Design B: external slots already populated by set_external_capture_buf() */
        {
            dma_buf[n] = (camera_dma_buffer*)malloc(sizeof(camera_dma_buffer[n]));
            ret = video_buffer_alloc_dmabuf(dma_buf[n],CAPTUREBUF);
            if (-1 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to Allocate DMA buffer for the dma_buf\n");
                return ret;
            }
        }
        memset(&buf_capture, 0, sizeof(buf_capture));
        buf_capture.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf_capture.memory = V4L2_MEMORY_DMABUF;
        buf_capture.index = n;
        buf_capture.m.fd = (unsigned long) dma_buf[n]->dbuf_fd;
        buf_capture.length = dma_buf[n]->size;
        ret = xioctl(m_fd, VIDIOC_QBUF, &buf_capture);
        if (-1 == ret)
        {
            return -1;
        }
    }

    ret = start_capture();
    if (0 != ret) return ret;

    return 0;
}


/*****************************************
* Function Name : close_capture
* Description   : Close camera and free buffer
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::close_camera()
{
    int8_t ret = 0;
    int32_t i = 0;

    ret = stop_capture();
    if (0 != ret) return ret;

    video_buffer_free_dmabuf(wayland_buf);
    free(wayland_buf);
    wayland_buf = NULL;

    video_buffer_free_dmabuf(overlay_buf);
    free(overlay_buf);
    overlay_buf = NULL;

    video_buffer_free_dmabuf(drpai_buf);
    free(drpai_buf);
    drpai_buf = NULL;

    for (i = 0;i<CAP_BUF_NUM;i++)
    {
        if (m_ext_bufs)   /* child owns the mmngr buffer + fd; just unmap + free our struct */
        {
            if (dma_buf[i] && dma_buf[i]->mem) munmap(dma_buf[i]->mem, dma_buf[i]->size);
        }
        else
        {
            video_buffer_free_dmabuf(dma_buf[i]);
        }
        free(dma_buf[i]);
        dma_buf[i] = NULL;
    }

    close(m_fd);
    return 0;
}


/*****************************************
* Function Name : xioctl
* Description   : ioctl calling
* Arguments     : fd = V4L2 file descriptor
*                 request = V4L2 control ID defined in videodev2.h
*                 arg = set value
* Return value  : int = output parameter
******************************************/
int8_t Camera::xioctl(int8_t fd, int32_t request, void * arg)
{
    int8_t r;
    do r = ioctl(fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

/*****************************************
* Function Name : start_capture
* Description   : Set STREAMON
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::start_capture()
{
    int8_t ret = 0;
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ret = xioctl(m_fd, VIDIOC_STREAMON, &buf.type);
    if (-1 == ret)
    {
        return -1;
    }
    return 0;
}

/*****************************************
* Function Name : capture_qbuf
* Description   : Function to enqueue the buffer.
*                 (Call this function after capture_image() to restart filling image data into buffer)
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::capture_qbuf()
{
    int8_t ret = 0;

    ret = xioctl(m_fd, VIDIOC_QBUF, &buf_capture);
    if (-1 == ret)
    {
        return -1;
    }
    return 0;
}

/*****************************************
* Function Name : qbuf_index
* Description   : Re-enqueue a specific ring slot by index. Called from the
*                 GStreamer streaming thread (encode release notify) to recycle
*                 a v4l2 buffer once the encoder is done reading its dma-buf.
*                 Mirrors start_camera()'s V4L2_MEMORY_DMABUF QBUF exactly, and
*                 uses a LOCAL v4l2_buffer so it never clobbers buf_capture.
* Arguments     : index = ring slot index to requeue
* Return value  : 0 if succeeded, -1 otherwise
******************************************/
int8_t Camera::qbuf_index(uint32_t index)
{
    std::lock_guard<std::mutex> lk(m_ring_mtx);
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index  = index;
    buf.m.fd   = (unsigned long) dma_buf[index]->dbuf_fd;
    buf.length = dma_buf[index]->size;
    if (xioctl(m_fd, VIDIOC_QBUF, &buf) == -1)
    {
        fprintf(stderr, "[camera] qbuf_index VIDIOC_QBUF idx %u failed\n", index);
        return -1;
    }
    return 0;
}


/*****************************************
* Function Name : capture_image
* Description   : Function to capture image and return the physical memory address where the captured image stored.
*                 Must call capture_qbuf after calling this function.
* Arguments     : -
* Return value  : the physical memory address where the captured image stored.
******************************************/
uint64_t Camera::capture_image()
{
    int8_t ret = 0;
    /* app_m5 L4a (CPU optimisation, 2026-06-27): the redundant select()/pselect6
     * before DQBUF is removed. m_fd is opened O_RDWR (BLOCKING -- no O_NONBLOCK,
     * see open_camera_device()), so VIDIOC_DQBUF already blocks until a frame is
     * ready. The prior select() was a second wait on the same readiness, costing
     * one extra syscall (pselect6) per frame at 60fps for no benefit. DQBUF alone
     * is sufficient and equally blocking. EINTR on DQBUF is retried below. */
    for (;;)
    {
        /* Get buffer where camera stored data (blocks until a frame is ready). */
        ret = xioctl(m_fd, VIDIOC_DQBUF, &buf_capture);
        if (-1 == ret)
        {
            if (EINTR == errno) continue;   /* interrupted -> retry the DQBUF */
            return 0;
        }
        break;
    }

    if (!m_ext_bufs)   /* external bufs are non-cached (no flush) and the child owns the mmngr id */
    {
        ret = video_buffer_flush_dmabuf(dma_buf[buf_capture.index]->idx, dma_buf[buf_capture.index]->size);
        if (0 != ret)
        {
            return 0;
        }
    }
    return  dma_buf[buf_capture.index]->phy_addr;
}

/*****************************************
* Function Name : stop_capture
* Description   : Set STREAMOFF
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::stop_capture()
{
    int8_t ret = 0;
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;

    ret = xioctl(m_fd, VIDIOC_STREAMOFF, &buf.type);
    if (-1 == ret)
    {
        return -1;
    }
    return 0;
}

/*****************************************
* Function Name : open_camera_device
* Description   : Function to open camera *called by start_camera
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::open_camera_device()
{
    char dev_name[4096] = {0};
    int32_t i = 0;
    int8_t ret = 0;
    struct v4l2_capability fmt;

    for (i = 0; i < 15; i++)
    {
        snprintf(dev_name, sizeof(dev_name), "/dev/video%d", i);
        m_fd = open(dev_name, O_RDWR);
        if (-1 == m_fd)
        {
            continue;
        }

        /* Check device is valid (Query Device information) */
        memset(&fmt, 0, sizeof(fmt));
        ret = xioctl(m_fd, VIDIOC_QUERYCAP, &fmt);

        if (-1 == ret)
        {
            return -1;
        }

#if INPUT_CAM_TYPE == 1
        ret = strcmp((const char*)fmt.driver, "rzg2l_cru");
        if (0 == ret)
        {
            printf("[INFO] CSI2 Camera: %s\n", dev_name);
            break;
        }
#else /* INPUT_CAM_TYPE */
        /* Search USB camera */
        ret = strcmp((const char*)fmt.driver, "uvcvideo");
        if (0 == ret)
        {
            printf("[INFO] USB Camera: %s\n", dev_name);
            break;
        }
#endif /* INPUT_CAM_TYPE */
        close(m_fd);
    }

    if (15 <= i)
    {
        return -1;
    }
    return 0;
}

/*****************************************
* Function Name : init_camera_fmt
* Description   : Function to request format *called by start_camera
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::init_camera_fmt()
{
    int8_t ret = 0;
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = camera_width;
    fmt.fmt.pix.height = camera_height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    ret = xioctl(m_fd, VIDIOC_S_FMT, &fmt);
    if (-1 == ret)
    {
        printf("[ERROR] VIDIOC_S_FMT Failed: %d\n", ret);
        return -1;
    }
    struct v4l2_streamparm* setfps;
    setfps = (struct v4l2_streamparm*)calloc(1, sizeof(struct v4l2_streamparm));
    memset(setfps, 0, sizeof(struct v4l2_streamparm));
    setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps->parm.capture.timeperframe.numerator = 1;
    setfps->parm.capture.timeperframe.denominator = 30;
    if (ioctl(m_fd, VIDIOC_S_PARM, setfps) < 0)
    {
        perror("VIDIOC_S_PARM");
    }
    return 0;
}

/*****************************************
* Function Name : init_buffer
* Description   : Initialize camera buffer *called by start_camera
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::init_buffer()
{
    int8_t ret = 0;
    int32_t i = 0;
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = CAP_BUF_NUM;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_DMABUF;

    /*Request a buffer that will be kept in the device*/
    ret = xioctl(m_fd, VIDIOC_REQBUFS, &req);
    if (-1 == ret)
    {
        printf("[ERROR] VIDIOC_REQBUFS Failed: %d\n", ret);
        return -1;
    }

    struct v4l2_buffer buf;
    for (i =0; i < CAP_BUF_NUM; i++)
    {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_DMABUF;
        buf.index = i;

        /* Extract buffer information */
        ret = xioctl(m_fd, VIDIOC_QUERYBUF, &buf);
        if (-1 == ret)
        {
            printf("[ERROR] VIDIOC_QUERYBUF Failed: %d\n", ret);
            return -1;
        }

    }

    return 0;
}


/*****************************************
* Function Name : save_bin
* Description   : Get the capture image from buffer and save it into binary file
* Arguments     : filename = binary file name to be saved
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::save_bin(std::string filename)
{
    int8_t ret = 0;
    FILE * fp = fopen(filename.c_str(), "wb");
    if (!fp)
    {
        return -1;
    }

    /* Get data from buffer and write to binary file */
    ret = fwrite((uint8_t *)dma_buf[buf_capture.index]->mem, sizeof(uint8_t), dma_buf[buf_capture.index]->size, fp);

    if (!ret)
    {
        fclose(fp);
        return -1;
    }

    fclose(fp);
    return 0;
}

/*****************************************
* Function Name : video_buffer_alloc_dmabuf
* Description   : Allocate a DMA buffer for the camera
* Arguments     : buffer = pointer to the camera_dma_buffer struct
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Camera::video_buffer_alloc_dmabuf(struct camera_dma_buffer *buffer,int buf_size)
{
    MMNGR_ID id;
    uint32_t phard_addr;
    void *puser_virt_addr;
    int m_dma_fd;

    buffer->size = buf_size;
    mmngr_alloc_in_user_ext(&id, buffer->size, &phard_addr, &puser_virt_addr, MMNGR_VA_SUPPORT_CACHED, NULL);
    memset((void*)puser_virt_addr, 0, buffer->size);
    buffer->idx = id;
    buffer->mem = (void *)puser_virt_addr;
    buffer->phy_addr = phard_addr;
    if (!buffer->mem)
    {
        return -1;
    }
    mmngr_export_start_in_user_ext(&id, buffer->size, phard_addr, &m_dma_fd, NULL);
    buffer->dbuf_fd = m_dma_fd;
    return 0;
}

/*****************************************
* Function Name : video_buffer_free_dmabuf
* Description   : free a DMA buffer for the camera
* Arguments     : buffer = pointer to the camera_dma_buffer struct
* Return value  : -
******************************************/
void Camera::video_buffer_free_dmabuf(struct camera_dma_buffer *buffer)
{
    mmngr_free_in_user_ext(buffer->idx);
    return;
}


/*****************************************
* Function Name : video_buffer_flush_dmabuf
* Description   : flush a DMA buffer for the camera
* Arguments     : buffer = pointer to the camera_dma_buffer struct
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int Camera::video_buffer_flush_dmabuf(uint32_t idx, uint32_t size)
{
    int mm_ret = 0;
    
    /* Flush capture image area cache */
    mm_ret = mmngr_flush(idx, 0, size);
    
    return mm_ret;
}

/*****************************************
* Function Name : get_img
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : camera buffer
******************************************/
uint8_t * Camera::get_img()
{
    return (uint8_t *)dma_buf[buf_capture.index]->mem;
}


/*****************************************
* Function Name : get_size
* Description   : Function to return the camera buffer size (W x H x C)
* Arguments     : -
* Return value  : camera buffer size (W x H x C )
******************************************/
int32_t Camera::get_size()
{
    return dma_buf[buf_capture.index]->size;
}

/*****************************************
* Function Name : get_w
* Description   : Get camera_width. This function is currently NOT USED.
* Arguments     : -
* Return value  : camera_width = width of camera capture image.
******************************************/
int32_t Camera::get_w()
{
    return camera_width;
}

/*****************************************
* Function Name : set_w
* Description   : Set camera_width. This function is currently NOT USED.
* Arguments     : w = new camera capture image width
* Return value  : -
******************************************/
void Camera::set_w(int32_t w)
{
    camera_width= w;
    return;
}

/*****************************************
* Function Name : get_h
* Description   : Get camera_height. This function is currently NOT USED.
* Arguments     : -
* Return value  : camera_height = height of camera capture image.
******************************************/
int32_t Camera::get_h()
{
    return camera_height;
}

/*****************************************
* Function Name : set_h
* Description   : Set camera_height. This function is currently NOT USED.
* Arguments     : w = new camera capture image height
* Return value  : -
******************************************/
void Camera::set_h(int32_t h)
{
    camera_height = h;
    return;
}

/*****************************************
* Function Name : get_c
* Description   : Get camera_color. This function is currently NOT USED.
* Arguments     : -
* Return value  : camera_color = color channel of camera capture image.
******************************************/
int32_t Camera::get_c()
{
    return camera_color;
}

/*****************************************
* Function Name : set_c
* Description   : Set camera_color. This function is currently NOT USED.
* Arguments     : c = new camera capture image color channel
* Return value  : -
******************************************/
void Camera::set_c(int32_t c)
{
    camera_color= c;
    return;
}

/* =====================================================================
 * app_m5 SOFTWARE AUTO-EXPOSURE (AE)
 * ---------------------------------------------------------------------
 * Goal: HOLD a constant scene brightness (mean luma) automatically as the
 * lighting changes, while NEVER letting exposure exceed the 1/60s frame
 * period (so fps stays at 60) and preventing highlight clipping.
 *
 * Control model (board-measured 2026-06-27, YUYV 1920x1080):
 *   exp=500/g6 ->Y5.8  4000/g6 ->Y36  8000/g6 ->Y57.8  12000/g6 ->Y74
 *   16000/g6 ->Y84.6  16000/g10 ->Y108.6(clip3%)  16000/g24 ->Y162(clip22%)
 * => luma is ~ proportional to (exposure * gain): a MULTIPLICATIVE actuator.
 *    A correction factor f = TARGET/mean, damped, drives mean -> TARGET in a
 *    few steps. Gain is costly (noise + clipping) so we prefer EXPOSURE up to
 *    16666us, only adding GAIN above that, and removing GAIN before EXPOSURE
 *    when darkening. clip% rising with gain is exactly why the anti-clip rule
 *    forces a darken step before the proportional term.
 * ===================================================================== */

/* V4L2 control IDs (from the live camera: exposure 0x00980911,
 * gain 0x00980913, exposure_mode 0x009a0901 == V4L2_CID_EXPOSURE_AUTO). */
#ifndef V4L2_CID_EXPOSURE
#define V4L2_CID_EXPOSURE        0x00980911
#endif
#ifndef V4L2_CID_GAIN
#define V4L2_CID_GAIN            0x00980913
#endif

/* Tunables (overridable where sensible via env in ae_init). */
#define AE_EXP_MIN      100      /* us  */
#define AE_EXP_MAX      16666    /* us  -- HARD ceiling = 1/60s, NEVER exceeded */
#define AE_GAIN_MIN     1
#define AE_GAIN_MAX     64
#define AE_PERIOD       12       /* run the controller every Nth frame (~5Hz)  */
#define AE_SAMPLE_STRIDE 64      /* sample every 32nd LUMA byte (file off 64)   */
#define AE_DEADBAND     2.0      /* +/- luma: inside this -> do nothing         */
#define AE_KP           0.6      /* damping on the multiplicative correction    */
#define AE_CLIP_LIMIT   0.02     /* clip_frac above this -> force darken         */
#define AE_CLIP_VAL     250      /* luma >= this counts as a clipped/highlight  */
#define AE_DARKEN_STEP  0.85     /* multiplicative darken on a clip event        */
#define AE_FMAX         1.6      /* clamp the per-step factor (anti-overshoot)   */
#define AE_FMIN         0.55     /* (asymmetric: darken can be a bit harder)     */
#define AE_GAIN_EXP_REF 16666    /* exposure level at/above which we add gain    */

/* VIDIOC_S_CTRL helper. Silently ignores failure (a transient EBUSY on a
 * shared bus must not kill the capture thread). */
void Camera::ae_set_ctrl(uint32_t id, int32_t val)
{
    struct v4l2_control c;
    memset(&c, 0, sizeof(c));
    c.id    = id;
    c.value = val;
    xioctl(m_fd, VIDIOC_S_CTRL, &c);
}

/*****************************************
* Function Name : ae_init
* Description   : Initialise software AE. Reads env (AE_AUTO default ON,
*                 AE_TARGET override of the measured default), forces MANUAL
*                 exposure mode so we own exposure/gain, and seeds the actuator
*                 at the user-approved exposure=8000/gain=6 level.
******************************************/
void Camera::ae_init()
{
    const char* a = getenv("AE_AUTO");
    m_ae_on = !(a && atoi(a) == 0);          /* default ON; AE_AUTO=0 disables */

    const char* t = getenv("AE_TARGET");
    if (t) { double v = atof(t); if (v > 1.0 && v < 254.0) m_ae_target = v; }

    m_ae_inited = true;

    if (!m_ae_on)
    {
        fprintf(stderr, "[ae] AE_AUTO=0 -> software AE DISABLED (using whatever exposure/gain is set)\n");
        return;
    }

    /* Take manual control + seed the actuator at the user-approved level.
     * exposure_mode menu: 0 = Manual. */
    ae_set_ctrl(0x009a0901 /*exposure_mode/EXPOSURE_AUTO*/, 0);
    m_ae_exposure = 8000;
    m_ae_gain     = 6;
    ae_set_ctrl(V4L2_CID_EXPOSURE, m_ae_exposure);
    ae_set_ctrl(V4L2_CID_GAIN,     m_ae_gain);

    fprintf(stderr, "[ae] software AE ON: target_luma=%.1f  seed exp=%d gain=%d (exp_max=%d=1/60s)\n",
            m_ae_target, m_ae_exposure, m_ae_gain, AE_EXP_MAX);
}

/*****************************************
* Function Name : ae_update
* Description   : Per-frame AE entry. Cheap: returns immediately on the
*                 (AE_PERIOD-1)/AE_PERIOD frames; on the Nth frame it samples a
*                 few-thousand-point luma grid, computes mean + clip_frac, and
*                 drives a damped proportional + anti-clip controller that maps a
*                 single brightness change onto (exposure first, then gain).
* Arguments     : yuyv     = CPU-readable capture buffer (YUYV, Y at even bytes)
*                 buf_size = buffer size in bytes
******************************************/
void Camera::ae_update(const uint8_t* yuyv, uint32_t buf_size)
{
    if (!m_ae_on || !yuyv) return;
    if ((m_ae_framecnt++ % AE_PERIOD) != 0) return;   /* off the 60fps hot path */

    /* The capture slot is the gray-padded 1920x1920 square but the camera DMAs
     * only the top CAM_IMAGE_HEIGHT rows (the real frame); sample ONLY that
     * region so the gray padding (Y=114) never biases the measurement. */
    uint32_t real_bytes = (uint32_t)CAM_IMAGE_WIDTH * CAM_IMAGE_HEIGHT * CAM_IMAGE_CHANNEL_YUY2;
    if (real_bytes > buf_size) real_bytes = buf_size;

    uint64_t sum = 0; uint32_t cnt = 0, clip = 0;
    for (uint32_t i = 0; i < real_bytes; i += AE_SAMPLE_STRIDE)  /* even off => Y */
    {
        uint8_t y = yuyv[i];
        sum += y;
        if (y >= AE_CLIP_VAL) clip++;
        cnt++;
    }
    if (cnt == 0) return;
    double mean = (double)sum / (double)cnt;
    double clip_frac = (double)clip / (double)cnt;

    /* ---- decide a multiplicative brightness correction factor ---- */
    double f;
    if (clip_frac > AE_CLIP_LIMIT)
    {
        /* highlights blowing: force a darken step regardless of mean so we pull
         * gain/exposure down and recover clipped detail. */
        f = AE_DARKEN_STEP;
    }
    else
    {
        double err = m_ae_target - mean;
        if (err > -AE_DEADBAND && err < AE_DEADBAND) return;   /* hysteresis: settled */
        /* luma ~ linear in (exp*gain) => want factor = target/mean, damped. */
        f = 1.0 + AE_KP * (m_ae_target / (mean > 1.0 ? mean : 1.0) - 1.0);
        if (f > AE_FMAX) f = AE_FMAX;
        if (f < AE_FMIN) f = AE_FMIN;
    }

    /* ---- apply the factor via a SINGLE brightness "level" L = exposure*gain,
     * then DECOMPOSE L deterministically into (exposure, gain). Because luma is
     * ~proportional to exposure*gain (board-measured), scaling L by f scales the
     * luma by ~f. The decomposition is single-valued (no hysteresis between two
     * actuators -> no hunting), prefers EXPOSURE, and uses the MINIMUM gain:
     *     exposure = clamp(L, EXP_MIN, EXP_MAX)
     *     gain     = clamp(ceil(L / EXP_MAX), GAIN_MIN, GAIN_MAX)
     * i.e. gain stays 1 until exposure hits its 1/60s ceiling, then grows just
     * enough to cover the overflow; when the scene brightens again exposure
     * drops back below max and gain automatically returns toward 1 (low noise).
     * Exposure can NEVER exceed EXP_MAX by construction -> 60fps is guaranteed. */
    double L = (double)m_ae_exposure * (double)m_ae_gain;   /* current level */
    double Lf = L * f;                                       /* desired level */
    double L_min = (double)AE_EXP_MIN * (double)AE_GAIN_MIN;
    double L_max = (double)AE_EXP_MAX * (double)AE_GAIN_MAX;
    if (Lf < L_min) Lf = L_min;
    if (Lf > L_max) Lf = L_max;

    int32_t new_gain = (int32_t)((Lf / (double)AE_EXP_MAX) + 0.999);  /* ceil */
    if (new_gain < AE_GAIN_MIN) new_gain = AE_GAIN_MIN;
    if (new_gain > AE_GAIN_MAX) new_gain = AE_GAIN_MAX;
    int32_t new_exp  = (int32_t)((Lf / (double)new_gain) + 0.5);
    if (new_exp < AE_EXP_MIN) new_exp = AE_EXP_MIN;

    /* hard clamps (exposure ceiling = 1/60s is INVIOLABLE) */
    if (new_exp  < AE_EXP_MIN)  new_exp  = AE_EXP_MIN;
    if (new_exp  > AE_EXP_MAX)  new_exp  = AE_EXP_MAX;
    if (new_gain < AE_GAIN_MIN) new_gain = AE_GAIN_MIN;
    if (new_gain > AE_GAIN_MAX) new_gain = AE_GAIN_MAX;

    /* push only what changed (avoid redundant ioctls) */
    if (new_exp != m_ae_exposure)
    {
        m_ae_exposure = new_exp;
        ae_set_ctrl(V4L2_CID_EXPOSURE, m_ae_exposure);
    }
    if (new_gain != m_ae_gain)
    {
        m_ae_gain = new_gain;
        ae_set_ctrl(V4L2_CID_GAIN, m_ae_gain);
    }

    /* observable trace, throttled (~ every 12th control tick ~= 2.4s) */
    if ((m_ae_logcnt++ % 12) == 0)
    {
        fprintf(stderr, "[ae] luma=%.1f target=%.1f clip=%.2f%% exp=%d gain=%d%s\n",
                mean, m_ae_target, clip_frac * 100.0, m_ae_exposure, m_ae_gain,
                (clip_frac > AE_CLIP_LIMIT) ? " (anti-clip darken)" : "");
    }
}
