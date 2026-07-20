/***********************************************************************************************************************
* Copyright (C) 2023 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : camera.h
* Version      : 1.00-github
* Description  : for RZ/V2H DRP-AI Sample Application with MIPI/USB Camera
***********************************************************************************************************************/

#ifndef CAMERA_H
#define CAMERA_H

#include <linux/videodev2.h>
#include <mutex>
#include "define.h"
/* This block of code is only accessible from C code. */
#ifdef __cplusplus
extern "C" {
#endif
#include "mmngr_user_public.h"
#include "mmngr_buf_user_public.h"
#ifdef __cplusplus
}
#endif

class Camera
{
    public:
        Camera();
        ~Camera();

        struct camera_dma_buffer
        {
         /* The index of the buffer. */
         uint32_t idx;
         /* The file descriptor for the DMA buffer. */
         uint32_t dbuf_fd;
         /* The size of the buffer in bytes. */
         uint32_t size;
         /* The physical address of DMA buffer. */
         uint32_t phy_addr;
         /* The pointer to the memory for the buffer. */
         void *mem;           
        };
        struct camera_dma_buffer *wayland_buf;
        struct camera_dma_buffer *overlay_buf;
        struct camera_dma_buffer *drpai_buf;
        int8_t start_camera();
        int8_t capture_qbuf();
        /* zero-copy encode support (re-QBUF a slot from the gst streaming thread) */
        int8_t   qbuf_index(uint32_t index);
        uint32_t get_capture_index() const { return buf_capture.index; }
        struct camera_dma_buffer* get_dma_buffer(uint32_t index) { return dma_buf[index]; }
        /* Design B: use externally-owned (capture_encoder child) capture buffers. Call
         * once per ring slot BEFORE start_camera(); the camera then QBUFs these
         * dma-buf fds (V4L2_MEMORY_DMABUF kernel import) instead of allocating. */
        int8_t set_external_capture_buf(uint32_t index, int dbuf_fd,
                                        uint32_t phys_addr, uint32_t size);
        uint64_t capture_image();
        int8_t close_camera();
        int8_t save_bin(std::string filename);
        int video_buffer_flush_dmabuf(uint32_t idx, uint32_t size);

        /* =====================================================================
         * app_m5 SOFTWARE AUTO-EXPOSURE (AE) -- 60fps-safe brightness hold
         * ---------------------------------------------------------------------
         * The camera's built-in Auto extends exposure past the 1/60s frame period
         * (drops fps) and its AGC target clips highlights. Instead we hold a fixed
         * MANUAL exposure/gain and run our OWN cheap proportional+anti-clip loop on
         * the captured YUYV luma, inside the capture thread (which already DQBUFs
         * every frame -- a separate process can't, V4L2 is single-reader).
         *
         * Exposure is kept <= AE_EXP_MAX (16666us = 1/60s) so 60fps never drops;
         * GAIN is only added once exposure is maxed (and dropped before exposure
         * when darkening) to minimise sensor noise / clipping. Measured default
         * TARGET_LUMA = the user-approved exposure=8000 gain=6 level (mean Y ~57).
         *
         *   ae_init()   -- read env (AE_AUTO default 1, AE_TARGET override), take
         *                  manual control of exposure_mode/exposure/gain, seed state.
         *   ae_update() -- call once per captured frame with the YUYV buffer; it
         *                  samples + runs the controller only every AE_PERIOD frames
         *                  (~5Hz) so it is off the 60fps critical path.
         * ===================================================================== */
        void ae_init();
        void ae_update(const uint8_t* yuyv, uint32_t buf_size);

        uint8_t * get_img();
        int32_t get_size();
        int32_t get_w();
        void set_w(int32_t w);
        int32_t get_h();
        void set_h(int32_t h);
        int32_t get_c();
        void set_c(int32_t c);

    private:
        std::string device;
        int32_t camera_width;
        int32_t camera_height;
        int32_t camera_color;
        int m_fd;
        uint8_t *buffer[CAP_BUF_NUM];

        #define WAYLANDBUF      (IMAGE_OUTPUT_WIDTH * IMAGE_OUTPUT_HEIGHT * IMAGE_CHANNEL_BGRA * WL_BUF_NUM)
        #define CAPTUREBUF      (CAM_IMAGE_WIDTH * CAM_IMAGE_HEIGHT * CAM_IMAGE_CHANNEL_YUY2)
/* DRP-AI input buffer holds the PADDED SQUARE (DRPAI_IN_WIDTH x DRPAI_IN_HEIGHT
 * = 1920x1920) YUYV image: the capture thread gray-fills the whole square then
 * copies the 1920x1080 camera frame into the top rows, and PreRuntimeV2H::Pre()
 * reads the full square (in_param shape = DRPAI_IN_WIDTH). MUST be the square
 * size or the gray-fill / memcpy overflow the heap (the 640x640 value did). */
#define DRPAIBUF        (DRPAI_IN_WIDTH * DRPAI_IN_HEIGHT * DRPAI_IN_CHANNEL_YUY2)


        struct v4l2_buffer buf_capture;
        struct camera_dma_buffer *dma_buf[CAP_BUF_NUM];
        bool m_ext_bufs = false; /* Design B: capture buffers owned by capture_encoder child */
        std::mutex m_ring_mtx;   /* serializes QBUF (gst thread) vs ring bookkeeping */

        /* --- app_m5 software AE state (see ae_init/ae_update) --- */
        bool     m_ae_on        = false; /* AE_AUTO=1 (default) -> loop active        */
        bool     m_ae_inited    = false; /* ae_init() ran                             */
        double   m_ae_target    = 57.0;  /* desired mean luma (measured @8000/6)       */
        int32_t  m_ae_exposure  = 8000;  /* current exposure (us), clamped [100,16666] */
        int32_t  m_ae_gain      = 6;     /* current analog gain, clamped [1,64]        */
        uint32_t m_ae_framecnt  = 0;     /* frame counter for the AE_PERIOD gate       */
        uint32_t m_ae_logcnt    = 0;     /* throttle the [ae] print                    */
        void     ae_set_ctrl(uint32_t id, int32_t val);  /* VIDIOC_S_CTRL helper       */
        int8_t xioctl(int8_t fd, int32_t request, void *arg);
        int8_t start_capture();
        int8_t stop_capture();
        int8_t open_camera_device();
        int8_t init_camera_fmt();
        int8_t init_buffer();
        int8_t video_buffer_alloc_dmabuf(struct camera_dma_buffer *buffer,int buf_size);
        void video_buffer_free_dmabuf(struct camera_dma_buffer *buffer);

};

#endif
