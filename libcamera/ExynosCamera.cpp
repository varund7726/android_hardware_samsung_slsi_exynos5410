/*
 * Copyright (C) 2013, The CyanogenMod Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>

#include <sys/mman.h>
#include <sys/types.h>

#include <utils/Log.h>

#include "ion.h"

#include "ExynosCamera.h"

//#define LOG_NDEBUG 0
#define LOG_TAG "ExynosCamera"

namespace android {

int cam_int_get_pixel_depth(uint32_t fmt)
{
    int depth = 0;

    switch (fmt) {
    case V4L2_PIX_FMT_JPEG:
        depth = 8;
        break;
    case V4L2_PIX_FMT_NV12:
    case V4L2_PIX_FMT_NV21:
    case V4L2_PIX_FMT_YUV420:
    case V4L2_PIX_FMT_YVU420M:
    case V4L2_PIX_FMT_NV12M:
    case V4L2_PIX_FMT_NV12MT:
        depth = 12;
        break;
    case V4L2_PIX_FMT_RGB565:
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_YVYU:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_VYUY:
    case V4L2_PIX_FMT_NV16:
    case V4L2_PIX_FMT_NV61:
    case V4L2_PIX_FMT_YUV422P:
    case V4L2_PIX_FMT_SBGGR10:
    case V4L2_PIX_FMT_SBGGR12:
    case V4L2_PIX_FMT_SBGGR16:
        depth = 16;
        break;
    case V4L2_PIX_FMT_RGB32:
        depth = 32;
        break;
    default:
        ALOGE("Get depth failed(format : %d)", fmt);
        break;
    }

    return depth;
}

int cam_int_s_fmt(node_info_t *node)
{
    struct v4l2_format v4l2_fmt;
    unsigned int framesize;
    int ret;

    memset(&v4l2_fmt, 0, sizeof(struct v4l2_format));

    v4l2_fmt.type = node->type;
    framesize = (node->width * node->height *
            cam_int_get_pixel_depth(node->format)) / 8;

    if (node->planes >= 1) {
        v4l2_fmt.fmt.pix_mp.width       = node->width;
        v4l2_fmt.fmt.pix_mp.height      = node->height;
        v4l2_fmt.fmt.pix_mp.pixelformat = node->format;
        v4l2_fmt.fmt.pix_mp.field       = V4L2_FIELD_ANY;
    } else {
        ALOGE("%s:S_FMT, Out of bound : Number of element plane",__FUNCTION__);
    }

    /* Set up for capture */
    ret = exynos_v4l2_s_fmt(node->fd, &v4l2_fmt);

    if (ret < 0)
        ALOGE("%s: exynos_v4l2_s_fmt fail (%d)",__FUNCTION__, ret);


    return ret;
}

int cam_int_reqbufs(node_info_t *node)
{
    struct v4l2_requestbuffers req;
    int ret;

    req.count = node->buffers;
    req.type = node->type;
    req.memory = node->memory;

    ret = exynos_v4l2_reqbufs(node->fd, &req);

    if (ret < 0)
        ALOGE("%s: VIDIOC_REQBUFS (fd:%d) failed (%d)",__FUNCTION__,node->fd, ret);

    return req.count;
}

int cam_int_qbuf(node_info_t *node, int index)
{
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    int i;
    int ret = 0;

    v4l2_buf.m.planes   = planes;
    v4l2_buf.type       = node->type;
    v4l2_buf.memory     = node->memory;
    v4l2_buf.index      = index;
    v4l2_buf.length     = node->planes;

    for(i = 0; i < node->planes; i++){
        v4l2_buf.m.planes[i].m.fd = (int)(node->buffer[index].buf.fd.extFd[i]);
        v4l2_buf.m.planes[i].length  = (unsigned long)(node->buffer[index].buf.size.extS[i]);
    }

    ret = exynos_v4l2_qbuf(node->fd, &v4l2_buf);

    if (ret < 0)
        ALOGE("%s: cam_int_qbuf failed (index:%d)(ret:%d)",__FUNCTION__, index, ret);

    return ret;
}

int cam_int_streamon(node_info_t *node)
{
    enum v4l2_buf_type type = node->type;
    int ret;

    ret = exynos_v4l2_streamon(node->fd, type);

    if (ret < 0)
        ALOGE("%s: VIDIOC_STREAMON failed [%d] (%d)",__FUNCTION__, node->fd,ret);

    ALOGV("On streaming I/O... ... fd(%d)", node->fd);

    return ret;
}

int cam_int_streamoff(node_info_t *node)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    int ret;

    ALOGV("Off streaming I/O... fd(%d)", node->fd);
    ret = exynos_v4l2_streamoff(node->fd, type);

    if (ret < 0)
        ALOGE("%s: VIDIOC_STREAMOFF failed (%d)",__FUNCTION__, ret);

    return ret;
}

int isp_int_streamoff(node_info_t *node)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    int ret;

    ALOGV("Off streaming I/O... fd(%d)", node->fd);
    ret = exynos_v4l2_streamoff(node->fd, type);

    if (ret < 0)
        ALOGE("%s: VIDIOC_STREAMOFF failed (%d)",__FUNCTION__, ret);

    return ret;
}

int cam_int_dqbuf(node_info_t *node)
{
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    int ret;

    v4l2_buf.type       = node->type;
    v4l2_buf.memory     = node->memory;
    v4l2_buf.m.planes   = planes;
    v4l2_buf.length     = node->planes;

    ret = exynos_v4l2_dqbuf(node->fd, &v4l2_buf);
    if (ret < 0)
        ALOGE("%s: VIDIOC_DQBUF failed (%d)",__FUNCTION__, ret);

    return v4l2_buf.index;
}

int cam_int_dqbuf(node_info_t *node, int num_plane)
{
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    int ret;

    v4l2_buf.type       = node->type;
    v4l2_buf.memory     = node->memory;
    v4l2_buf.m.planes   = planes;
    v4l2_buf.length     = num_plane;

    ret = exynos_v4l2_dqbuf(node->fd, &v4l2_buf);
    if (ret < 0)
        ALOGE("%s: VIDIOC_DQBUF failed (%d)",__FUNCTION__, ret);

    return v4l2_buf.index;
}

int cam_int_s_input(node_info_t *node, int index)
{
    int ret;

    ret = exynos_v4l2_s_input(node->fd, index);
    if (ret < 0)
        ALOGE("%s: VIDIOC_S_INPUT failed (%d)",__FUNCTION__, ret);

    return ret;
}

int cam_int_open_node(node_info_t *node, int node_num)
{
    char node_name[30];
    int fd = 0;

    memset(&node_name, 0x00, sizeof(char[30]));
    sprintf(node_name, "%s%d", NODE_PREFIX, node_num);
    fd = exynos_v4l2_open(node_name, O_RDWR, 0);

    if (fd < 0) {
        ALOGE("ERR(%s): failed to open node (%s) fd (%d)",
                __FUNCTION__, node_name, fd);
        return fd;
    } else {
        ALOGV("DEBUG(%s): node opened(%s) fd (%d)",
                __FUNCTION__, node_name, fd);
    }
    node->fd = fd;

    return 0;
}

void cam_int_init_memory(ExynosBuffer *buf, int memory_num)
{
    for (int i = 0; i < memory_num; i++) {
        buf->virt.extP[i] = (char *) MAP_FAILED;
        buf->fd.extFd[i] = -1;
        buf->size.extS[i] = 0;
    }
}

void cam_int_free_memory(ExynosBuffer *buf, int memory_num)
{
    int ret = 0;

    for (int i = 0; i < memory_num; i++) {
        if (buf->fd.extFd[i] != -1) {
            if (buf->virt.extP[i] != (char *) MAP_FAILED) {
                ret = ion_unmap(buf->virt.extP[i], buf->size.extS[i]);
                if (ret < 0)
                    ALOGE("ERR(%s)", __FUNCTION__);
            }
            ion_free(buf->fd.extFd[i]);
            ALOGV("freeCameraMemory : [%d][0x%08x] size(%d)",
                    i, (unsigned int)(buf->virt.extP[i]), buf->size.extS[i]);
        }
        buf->fd.extFd[i] = -1;
        buf->virt.extP[i] = (char *)MAP_FAILED;
        buf->size.extS[i] = 0;
    }
}

int cam_int_alloc_memory(ion_client client, ExynosBuffer *buf,
        int memory_num, int cache_flag)
{
    int ret = 0;
    int flag = 0;

    if (client == 0) {
        ALOGE("[%s] ionClient is zero (%d)\n", __FUNCTION__, client);
        return -1;
    }

    for (int i = 0 ; i < memory_num; i++) {
        if (buf->size.extS[i] == 0) {
            break;
        }
        if (1 << i & cache_flag)
            flag = ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC;
        else
            flag = 0;
        buf->fd.extFd[i] = ion_alloc(client, buf->size.extS[i], 0,
                ION_HEAP_SYSTEM_MASK, flag);
        if ((buf->fd.extFd[i] == -1) ||(buf->fd.extFd[i] == 0)) {
            ALOGE("[%s]ion_alloc(%d) failed\n",
                    __FUNCTION__, buf->size.extS[i]);
            buf->fd.extFd[i] = -1;
            cam_int_free_memory(buf, memory_num);
            return -1;
        }

        buf->virt.extP[i] = (char *) ion_map(buf->fd.extFd[i],
                buf->size.extS[i], 0);
        if ((buf->virt.extP[i] == (char *) MAP_FAILED) ||
                (buf->virt.extP[i] == NULL)) {
            ALOGE("[%s]src ion map failed(%d)\n",
                    __FUNCTION__, buf->size.extS[i]);
            buf->virt.extP[i] = (char *) MAP_FAILED;
            cam_int_free_memory(buf, memory_num);
            return -1;
        }
        ALOGV("%s: [%d][0x%08x] size(%d) flag(%d)", __FUNCTION__, i,
                (unsigned int)(buf->virt.extP[i]), buf->size.extS[i], flag);
    }

    return ret;
}

int cam_int_init_sensor(node_info_t *sensor, ion_client client)
{
    int res = 0;
    for (int i = 0; i < sensor->buffers; i++) {
        cam_int_init_memory(&sensor->buffer[i].buf, sensor->planes);
        sensor->buffer[i].buf.size.extS[0] = sensor->width * sensor->height * 2;
        sensor->buffer[i].buf.size.extS[1] = 8 * 1024;
        res = cam_int_alloc_memory(client, &sensor->buffer[i].buf,
                sensor->planes, 1 << 1);
        if (res) {
            ALOGE("ERROR(%s): cam_int_alloc_memory failed for sensor buffer %d",
                    __FUNCTION__, i);
            // Free already allocated sensor buffers
            for (int j = 0; j < i; j++) {
                cam_int_free_memory(&sensor->buffer[i].buf, sensor->planes);
            }
            break;
        }
    }

    return res;
}

void cam_int_init_isp(node_info_t *isp, node_info_t *sensor)
{
    isp->width = sensor->width;
    isp->height = sensor->height;
    isp->format = sensor->format;
    isp->planes = sensor->planes;
    isp->buffers = sensor->buffers;
    isp->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    isp->memory = V4L2_MEMORY_DMABUF;

    for (int i = 0; i < isp->buffers; i++) {
        cam_int_init_memory(&isp->buffer[i].buf, isp->planes);
        isp->buffer[i].buf.size.extS[0] = sensor->buffer[i].buf.size.extS[0];
        isp->buffer[i].buf.size.extS[1] = sensor->buffer[i].buf.size.extS[1];
        isp->buffer[i].buf.fd.extFd[0] = sensor->buffer[i].buf.fd.extFd[0];
        isp->buffer[i].buf.fd.extFd[1] = sensor->buffer[i].buf.fd.extFd[1];
        isp->buffer[i].buf.virt.extP[0] = sensor->buffer[i].buf.virt.extP[0];
        isp->buffer[i].buf.virt.extP[1] = sensor->buffer[i].buf.virt.extP[1];
    }
}

int cam_int_start_node(node_info_t *node, int sensor_id)
{
    int ret = 0;

    ret = cam_int_s_input(node, sensor_id);
    if (ret < 0) {
        ALOGE("ERR(%s): cam_int_s_input(%d) failed!!!! err = %d",
                __FUNCTION__, sensor_id, ret);
        return ret;
    }
    ret = cam_int_s_fmt(node);
    if (ret < 0) {
        ALOGE("ERR(%s): cam_int_s_fmt(%d) failed!!!! err = %d",
                __FUNCTION__, sensor_id, ret);
        return ret;
    }
    cam_int_reqbufs(node);

    return 0;
}

ExynosCamera::ExynosCamera()
    :
        m_cameraId(CAMERA_ID_BACK),
        m_ionClient(0)
{
    memset(&m_streamInfo, 0, sizeof(camera_hw_info_t));

    m_ionClient = ion_client_create();
    if (m_ionClient < 0) {
        ALOGE("ERR(%s): Fail on ion_client_create, err = %d",
                __FUNCTION__, m_ionClient);
    }
}

ExynosCamera::~ExynosCamera()
{
    delete m_cameraInfo;
}

bool ExynosCamera::create(int cameraId)
{
    m_cameraId = cameraId;

    if (m_cameraInfo) {
        delete m_cameraInfo;
    }

    if (cameraId == CAMERA_ID_BACK) {
        m_streamInfo.sensor_id = SENSOR_NAME_IMX135;
        m_cameraInfo = new ExynosCameraInfoIMX135();
    } else {
        m_streamInfo.sensor_id = SENSOR_NAME_S5K6B2;
        m_cameraInfo = new ExynosCameraInfoS5K6B2();
    }

    // TODO: setup ISP chain here?

    return true;
}

bool ExynosCamera::destroy()
{
    return true;
}

int ExynosCamera::getCameraId()
{
    return m_cameraId;
}

int ExynosCamera::getPreviewWidth(void)
{
    return m_previewWidth;
}

int ExynosCamera::getPreviewHeight(void)
{
    return m_previewHeight;
}

int ExynosCamera::getPreviewColorFormat(void)
{
    return m_previewColorFormat;
}

int ExynosCamera::getHardwarePreviewWidth()
{
    return m_cameraInfo->previewSize.width;
}

int ExynosCamera::getHardwarePreviewHeight()
{
    return m_cameraInfo->previewSize.height;
}

int ExynosCamera::getHardwarePreviewColorFormat()
{
    return m_cameraInfo->previewFormat;
}

int ExynosCamera::getPreviewNumBuffers()
{
    return m_streamInfo.scp.buffers;
}

bool ExynosCamera::setPreviewBuffer(ExynosBuffer *buf)
{
    node_info_t *previewStream = &m_streamInfo.scp;
    int idx = buf->reserved.p;

    if (previewStream->buffers <= idx) {
        ALOGE("ERR(%s): index out of range (%d/%d)",
                __FUNCTION__, idx, previewStream->buffers);
        return false;
    }

    previewStream->buffer[idx].buf = *buf;

    return true;
}

bool ExynosCamera::getPreviewBuffer(ExynosBuffer *buf)
{
    node_info_t *previewStream = &m_streamInfo.scp;
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[previewStream->planes];
    int ret;

    v4l2_buf.m.planes = planes;
    v4l2_buf.type = previewStream->type;
    v4l2_buf.memory = previewStream->memory;
    v4l2_buf.length = 0;

    for (int i = 0; i < 3; i++) {
        if (previewStream->buffer[0].buf.size.extS[i] != 0) {
            v4l2_buf.length++;
        }
    }

    ret = exynos_v4l2_dqbuf(previewStream->fd, &v4l2_buf);
    if (ret != 0) {
        ALOGE("ERR(%s): exynos_v4l2_dqbuf failed, err = %d",
                __FUNCTION__, ret);
        return false;
    }

    if ((unsigned int) previewStream->buffers <= v4l2_buf.index) {
        ALOGE("ERR(%s): index out of range (%d/%d)",
                __FUNCTION__, v4l2_buf.index, previewStream->buffers);
        return false;
    }

    *buf = previewStream->buffer[v4l2_buf.index].buf;

    return true;
}

bool ExynosCamera::putPreviewBuffer(ExynosBuffer *buf)
{
    node_info_t *previewStream = &m_streamInfo.scp;
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[previewStream->planes];
    int idx = buf->reserved.p;
    int ret;

    if (!previewStream->buffer[idx].valid) {
        ALOGE("ERR(%s): invalid buffer index %d",
                __FUNCTION__, idx);
    }

    v4l2_buf.m.planes = planes;
    v4l2_buf.type = previewStream->type;
    v4l2_buf.memory = previewStream->memory;
    v4l2_buf.index = idx;
    v4l2_buf.length = 0;

    for (int i = 0; i < 3; i++) {
        v4l2_buf.m.planes[i].m.fd = previewStream->buffer[idx].buf.fd.extFd[i];
        v4l2_buf.m.planes[i].length = previewStream->buffer[idx].buf.size.extS[i];

        if (previewStream->buffer[idx].buf.size.extS[i] != 0) {
            v4l2_buf.length++;
        }
    }

    ret = exynos_v4l2_qbuf(previewStream->fd, &v4l2_buf);
    if (ret != 0) {
        ALOGE("ERR(%s): exynos_v4l2_qbuf failed, err = %d",
                __FUNCTION__, ret);
        return false;
    }

    return true;
}

void ExynosCamera::setParameters(const CameraParameters *p)
{
    m_cameraInfo->fromParameters(p);
}

void ExynosCamera::getParameters(CameraParameters *p)
{
    m_cameraInfo->toParameters(p);
}

bool ExynosCamera::initializeIspChain(void)
{
    int ret;

    ret = cam_int_open_node(&m_streamInfo.sensor0, VIDEO_NODE_SENSOR0);
    ret = cam_int_open_node(&m_streamInfo.is3a1, VIDEO_NODE_IS3A1);
    ret = cam_int_open_node(&m_streamInfo.isp, VIDEO_NODE_ISP);
    ret = cam_int_open_node(&m_streamInfo.scc, VIDEO_NODE_SCALERC);
    ret = cam_int_open_node(&m_streamInfo.scp, VIDEO_NODE_SCALERP);

    /* initialize sensor0 */
    m_streamInfo.sensor0.width = m_cameraInfo->pictureSize.width;
    m_streamInfo.sensor0.height = m_cameraInfo->pictureSize.height;
    m_streamInfo.sensor0.format = V4L2_PIX_FMT_SBGGR16;
    m_streamInfo.sensor0.planes = 2;
    m_streamInfo.sensor0.buffers = NUM_SENSOR_BUFFERS;
    m_streamInfo.sensor0.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    m_streamInfo.sensor0.memory = V4L2_MEMORY_DMABUF;
    m_streamInfo.sensor0.status = false;

    ret = cam_int_init_sensor(&m_streamInfo.sensor0, m_ionClient);
    if (ret < 0) {
        ALOGE("ERR(%s): cam_int_sensor failed!!!!", __FUNCTION__);
        goto init_isp_chain_err;
    }

    // TODO: initialize IS3A1 ?

    /* initialize ISP with sensor0 */
    cam_int_init_isp(&m_streamInfo.isp, &m_streamInfo.sensor0);

    /* initialize ScalerC */
    m_streamInfo.scc.width = m_cameraInfo->videoSize.width;
    m_streamInfo.scc.height = m_cameraInfo->videoSize.height;
    m_streamInfo.scc.format = V4L2_PIX_FMT_YUYV;
    m_streamInfo.scc.planes = 2;
    m_streamInfo.scc.buffers = NUM_SCC_BUFFERS;
    m_streamInfo.scc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    m_streamInfo.scc.memory = V4L2_MEMORY_DMABUF;
    m_streamInfo.scc.status = false;

    return true;

init_isp_chain_err:
    close(m_streamInfo.sensor0.fd);
    close(m_streamInfo.is3a1.fd);
    close(m_streamInfo.isp.fd);
    close(m_streamInfo.scc.fd);
    close(m_streamInfo.scp.fd);

    return false;
}

}; // namespace android
