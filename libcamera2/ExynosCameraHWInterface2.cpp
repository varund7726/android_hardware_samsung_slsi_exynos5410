/*
**
** Copyright 2008, The Android Open Source Project
** Copyright 2012, Samsung Electronics Co. LTD
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

/*!
 * \file      ExynosCameraHWInterface2.cpp
 * \brief     source file for Android Camera API 2.0 HAL
 * \author    Sungjoong Kang(sj3.kang@samsung.com)
 * \date      2012/07/10
 *
 * <b>Revision History: </b>
 * - 2012/05/31 : Sungjoong Kang(sj3.kang@samsung.com) \n
 *   Initial Release
 *
 * - 2012/07/10 : Sungjoong Kang(sj3.kang@samsung.com) \n
 *   2nd Release
 *
 */

//#define LOG_NDEBUG 0
#define LOG_TAG "ExynosCameraHAL2"
#include <utils/Log.h>
#include <math.h>

#include "ExynosCameraHWInterface2.h"
#include "exynos_format.h"

//#define DEBUG_AF_ENABLED
#ifdef DEBUG_AF_ENABLED
#define AF_LOG(...) ALOGD(__VA_ARGS__)
#define AF_LOGV(...) ALOGV(__VA_ARGS__)
#else
#define AF_LOG(...) ((void)0)
#define AF_LOGV(...) ((void)0)
#endif

//#define DEBUG_FLASH_ENABLED
#ifdef DEBUG_FLASH_ENABLED
#define FLASH_LOG(...) ALOGD(__VA_ARGS__)
#else
#define FLASH_LOG(...) ((void)0)
#endif

namespace android {

enum {
    NO_TRANSITION                   = 0,
    HAL_AFSTATE_INACTIVE            = 1,
    HAL_AFSTATE_TRIGGERED           = 2,
    HAL_AFSTATE_STARTED             = 3,
    HAL_AFSTATE_SCANNING            = 4,
    HAL_AFSTATE_FOCUSED             = 5,
    HAL_AFSTATE_FAILED              = 6,
    HAL_AFSTATE_MAX,
};

#ifdef DEBUG_AF_ENABLED
const char HAL_AFState_Name[HAL_AFSTATE_MAX][50] = {
    {"NO TRANSITION"},
    {"AFSTATE_INACTIVE"},
    {"AFSTATE_TRIGGERED"},
    {"AFSTATE_STARTED"},
    {"AFSTATE_SCANNING"},
    {"AFSTATE_FOCUSED"},
    {"AFSTATE_FAILED"},
};
#endif

enum is_set_flash_command_state {
    FLASH_STATE_NO_TRANSITION = 0,
    FLASH_STATE_NONE,
    FLASH_STATE_ON,
    FLASH_STATE_ON_WAIT,
    FLASH_STATE_ON_DONE,
    FLASH_STATE_AUTO_AE_AWB_LOCK,
    FLASH_STATE_AE_AWB_LOCK_WAIT,
    FLASH_STATE_AUTO_WAIT,
    FLASH_STATE_AUTO_DONE,
    FLASH_STATE_AUTO_OFF,
    FLASH_STATE_CAPTURE,
    FLASH_STATE_CAPTURE_WAIT,
    FLASH_STATE_CAPTURE_JPEG,
    FLASH_STATE_CAPTURE_END,
    FLASH_STATE_MAX
};

#ifdef DEBUG_FLASH_ENABLED
const char HAL_FlashState_Name[FLASH_STATE_MAX][50] = {
    {"NO TRANSITION"},
    {"FLASH_STATE_NONE"},
    {"FLASH_STATE_ON"},
    {"FLASH_STATE_ON_WAIT"},
    {"FLASH_STATE_ON_DONE"},
    {"FLASH_STATE_AUTO_AE_AWB_LOCK"},
    {"FLASH_STATE_AE_AWB_LOCK_WAIT"},
    {"FLASH_STATE_AUTO_WAIT"},
    {"FLASH_STATE_AUTO_DONE"},
    {"FLASH_STATE_AUTO_OFF"},
    {"FLASH_STATE_CAPTURE"},
    {"FLASH_STATE_CAPTURE_WAIT"},
    {"FLASH_STATE_CAPTURE_JPEG"},
    {"FLASH_STATE_CAPTURE_END"},
};
#endif

void m_savePostView(const char *fname, uint8_t *buf, uint32_t size)
{
    int nw;
    int cnt = 0;
    uint32_t written = 0;

    ALOGV("opening file [%s], address[%x], size(%d)", fname, (unsigned int)buf, size);
    int fd = open(fname, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        ALOGE("failed to create file [%s]: %s", fname, strerror(errno));
        return;
    }

    ALOGV("writing %d bytes to file [%s]", size, fname);
    while (written < size) {
        nw = ::write(fd, buf + written, size - written);
        if (nw < 0) {
            ALOGE("failed to write to file %d [%s]: %s",written,fname, strerror(errno));
            break;
        }
        written += nw;
        cnt++;
    }
    ALOGV("done writing %d bytes to file [%s] in %d passes",size, fname, cnt);
    ::close(fd);
}

int get_pixel_depth(uint32_t fmt)
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
    framesize = (node->width * node->height * get_pixel_depth(node->format)) / 8;

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
        v4l2_buf.m.planes[i].m.fd = (int)(node->buffer[index].fd.extFd[i]);
        v4l2_buf.m.planes[i].length  = (unsigned long)(node->buffer[index].size.extS[i]);
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


gralloc_module_t const* ExynosCameraHWInterface2::m_grallocHal;

RequestManager::RequestManager(SignalDrivenThread* main_thread):
    m_sensorPipelineSkipCnt(0),
    m_cropX(0),
    m_lastCompletedFrameCnt(-1),
    m_lastAeMode(0),
    m_lastAaMode(0),
    m_lastAwbMode(0),
    m_lastAeComp(0),
    m_vdisBubbleEn(false)
{
    m_metadataConverter = new MetadataConverter;
    m_mainThread = main_thread;
    ResetEntry();
    return;
}

RequestManager::~RequestManager()
{
    ALOGV("%s", __FUNCTION__);
    if (m_metadataConverter != NULL) {
        delete m_metadataConverter;
        m_metadataConverter = NULL;
    }

    releaseSensorQ();
    return;
}

void RequestManager::ResetEntry()
{
    Mutex::Autolock lock(m_requestMutex);
    for (int i=0 ; i<NUM_MAX_REQUEST_MGR_ENTRY; i++) {
        memset(&(entries[i]), 0x00, sizeof(request_manager_entry_t));
        entries[i].internal_shot.shot.ctl.request.frameCount = -1;
    }
    m_numOfEntries = 0;
    m_entryInsertionIndex = -1;
    m_entryProcessingIndex = -1;
    m_entryFrameOutputIndex = -1;
}

int RequestManager::GetNumEntries()
{
    return m_numOfEntries;
}

void RequestManager::SetDefaultParameters(int cropX)
{
    m_cropX = cropX;
}

bool RequestManager::IsRequestQueueFull()
{
    Mutex::Autolock lock(m_requestMutex);
    if (m_numOfEntries>=NUM_MAX_REQUEST_MGR_ENTRY)
        return true;
    else
        return false;
}

void RequestManager::RegisterRequest(camera_metadata_t * new_request, int * afMode, uint32_t * afRegion)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);

    Mutex::Autolock lock(m_requestMutex);

    request_manager_entry * newEntry = NULL;
    int newInsertionIndex = GetNextIndex(m_entryInsertionIndex);
    ALOGV("DEBUG(%s): got lock, new insertIndex(%d), cnt before reg(%d)", __FUNCTION__,newInsertionIndex, m_numOfEntries );


    newEntry = &(entries[newInsertionIndex]);

    if (newEntry->status!=EMPTY) {
        ALOGV("DEBUG(%s): Circular buffer abnormal ", __FUNCTION__);
        return;
    }

    newEntry->status = REGISTERED;
    newEntry->original_request = new_request;
    memset(&(newEntry->internal_shot), 0, sizeof(struct camera2_shot_ext));
    m_metadataConverter->ToInternalShot(new_request, &(newEntry->internal_shot));

    newEntry->output_stream_count = 0;

    if (newEntry->internal_shot.shot.ctl.request.outputStreams[0] & MASK_OUTPUT_SCP)
        newEntry->output_stream_count++;

    if (newEntry->internal_shot.shot.ctl.request.outputStreams[0] & MASK_OUTPUT_SCC)
        newEntry->output_stream_count++;

    m_numOfEntries++;
    m_entryInsertionIndex = newInsertionIndex;

    /* Don't change AF mode in still shot mode */
    if (newEntry->internal_shot.shot.ctl.aa.captureIntent != AA_CAPTURE_INTENT_STILL_CAPTURE) {
        *afMode = (int)(newEntry->internal_shot.shot.ctl.aa.afMode);
        afRegion[0] = newEntry->internal_shot.shot.ctl.aa.afRegions[0];
        afRegion[1] = newEntry->internal_shot.shot.ctl.aa.afRegions[1];
        afRegion[2] = newEntry->internal_shot.shot.ctl.aa.afRegions[2];
        afRegion[3] = newEntry->internal_shot.shot.ctl.aa.afRegions[3];
    } else {
        *afMode = -1;
        ALOGD("DEBUG(%s): AA_CAPTURE_INTENT_STILL_CAPTURE Mode ", __FUNCTION__);
    }

    ALOGV("## RegisterReq DONE num(%d), insert(%d), processing(%d), frame(%d), (frameCnt(%d))",
    m_numOfEntries,m_entryInsertionIndex,m_entryProcessingIndex, m_entryFrameOutputIndex, newEntry->internal_shot.shot.ctl.request.frameCount);
}

void RequestManager::DeregisterRequest(camera_metadata_t ** deregistered_request)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    int frame_index;
    request_manager_entry * currentEntry;

    Mutex::Autolock lock(m_requestMutex);

    frame_index = GetCompletedIndex();
    currentEntry =  &(entries[frame_index]);
    if (currentEntry->status != COMPLETED) {
        ALOGW("DBG(%s): Circular buffer abnormal. processing(%d), frame(%d), status(%d) ", __FUNCTION__,
                       m_entryProcessingIndex, frame_index,(int)(currentEntry->status));
        return;
    }
    if (deregistered_request)  *deregistered_request = currentEntry->original_request;

    m_lastCompletedFrameCnt = currentEntry->internal_shot.shot.ctl.request.frameCount;

    currentEntry->status = EMPTY;
    currentEntry->original_request = NULL;
    memset(&(currentEntry->internal_shot), 0, sizeof(struct camera2_shot_ext));
    currentEntry->internal_shot.shot.ctl.request.frameCount = -1;
    currentEntry->output_stream_count = 0;
    m_numOfEntries--;
    ALOGV("## DeRegistReq DONE num(%d), insert(%d), processing(%d), frame(%d)",
     m_numOfEntries,m_entryInsertionIndex,m_entryProcessingIndex, m_entryFrameOutputIndex);

    CheckCompleted(GetNextIndex(frame_index));
    return;
}

bool RequestManager::PrepareFrame(size_t* num_entries, size_t* frame_size,
                camera_metadata_t ** prepared_frame, int afState)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    Mutex::Autolock lock(m_requestMutex);
    status_t res = NO_ERROR;
    int tempFrameOutputIndex = GetCompletedIndex();
    request_manager_entry * currentEntry =  &(entries[tempFrameOutputIndex]);
    ALOGV("DEBUG(%s): processing(%d), frameOut(%d), insert(%d) recentlycompleted(%d)", __FUNCTION__,
        m_entryProcessingIndex, m_entryFrameOutputIndex, m_entryInsertionIndex, m_completedIndex);

    if (currentEntry->status != COMPLETED) {
        ALOGV("DBG(%s): Circular buffer abnormal status(%d)", __FUNCTION__, (int)(currentEntry->status));

        return false;
    }
    m_entryFrameOutputIndex = tempFrameOutputIndex;
    m_tempFrameMetadata = place_camera_metadata(m_tempFrameMetadataBuf, 2000, 35, 500); //estimated
    add_camera_metadata_entry(m_tempFrameMetadata, ANDROID_CONTROL_AF_STATE, &afState, 1);
    res = m_metadataConverter->ToDynamicMetadata(&(currentEntry->internal_shot),
                m_tempFrameMetadata);
    if (res!=NO_ERROR) {
        ALOGE("ERROR(%s): ToDynamicMetadata (%d) ", __FUNCTION__, res);
        return false;
    }
    *num_entries = get_camera_metadata_entry_count(m_tempFrameMetadata);
    *frame_size = get_camera_metadata_size(m_tempFrameMetadata);
    *prepared_frame = m_tempFrameMetadata;
    ALOGV("## PrepareFrame DONE: frameOut(%d) frameCnt-req(%d) timestamp(%lld)", m_entryFrameOutputIndex,
        currentEntry->internal_shot.shot.ctl.request.frameCount, currentEntry->internal_shot.shot.dm.sensor.timeStamp);
    // Dump();
    return true;
}

int RequestManager::MarkProcessingRequest(ExynosBuffer* buf)
{
    struct camera2_shot_ext * shot_ext;
    struct camera2_shot_ext * request_shot;
    int targetStreamIndex = 0;
    request_manager_entry * newEntry = NULL;

    Mutex::Autolock lock(m_requestMutex);
    if (m_numOfEntries == 0)  {
        ALOGV("DEBUG(%s): Request Manager Empty ", __FUNCTION__);
        return -1;
    }

    shot_ext = (struct camera2_shot_ext *)buf->virt.extP[1];
    if ((m_entryProcessingIndex == m_entryInsertionIndex)
        && (entries[m_entryProcessingIndex].status == REQUESTED || entries[m_entryProcessingIndex].status == CAPTURED || entries[m_entryProcessingIndex].status == COMPLETED)) {
        ALOGV("## MarkProcReq skipping(request underrun) -  num(%d), insert(%d), processing(%d), frame(%d)",
         m_numOfEntries,m_entryInsertionIndex,m_entryProcessingIndex, m_entryFrameOutputIndex);
        shot_ext->isReprocessing = 0;
        return -1;
    }

    int newProcessingIndex = GetNextIndex(m_entryProcessingIndex);

    newEntry = &(entries[newProcessingIndex]);
    request_shot = &(newEntry->internal_shot);
    ALOGV("DEBUG(%s): index(%d) framecount(%d)", __FUNCTION__, newProcessingIndex, request_shot->shot.ctl.request.frameCount);
    if (newEntry->status != REGISTERED) {
        ALOGW("DEBUG(%s)(%d): Circular buffer abnormal, numOfEntries(%d), status(%d)", __FUNCTION__,
                                                newProcessingIndex, m_numOfEntries, newEntry->status);
        for (int i = 0; i < NUM_MAX_REQUEST_MGR_ENTRY; i++)
            ALOGV("DBG: entrie[%d].stream output cnt = %d, framecnt(%d)", i,
                        entries[i].output_stream_count, entries[i].internal_shot.shot.ctl.request.frameCount);
        return -1;
    }

    newEntry->status = REQUESTED;

    memset(shot_ext, 0x00, sizeof(struct camera2_shot_ext));
    shot_ext->shot.ctl.request.frameCount = request_shot->shot.ctl.request.frameCount;
    shot_ext->request_sensor = 1;
    shot_ext->dis_bypass = 1;
    shot_ext->dnr_bypass = 1;
    shot_ext->fd_bypass = 1;
    shot_ext->setfile = 0;

    targetStreamIndex = request_shot->shot.ctl.request.outputStreams[0];
    shot_ext->shot.ctl.request.outputStreams[0] = targetStreamIndex;
    if (targetStreamIndex & MASK_OUTPUT_SCP)
        shot_ext->request_scp = 1;

    if (targetStreamIndex & MASK_OUTPUT_SCC)
        shot_ext->request_scc = 1;

    if (shot_ext->shot.ctl.stats.faceDetectMode != FACEDETECT_MODE_OFF)
        shot_ext->fd_bypass = 0;

    shot_ext->shot.ctl.aa.mode = AA_CONTROL_NONE;

    shot_ext->shot.ctl.request.metadataMode = METADATA_MODE_FULL;
    shot_ext->shot.ctl.stats.faceDetectMode = FACEDETECT_MODE_FULL;
    shot_ext->shot.magicNumber = 0x23456789;
    shot_ext->shot.ctl.sensor.exposureTime = 0;
    shot_ext->shot.ctl.sensor.frameDuration = 33*1000*1000;
    shot_ext->shot.ctl.sensor.sensitivity = 0;

    shot_ext->isReprocessing = request_shot->isReprocessing;
    shot_ext->reprocessInput = request_shot->reprocessInput;
    if (shot_ext->isReprocessing) {
        ALOGD("(%s): reprocess request - framecnt(%d)",
                     __FUNCTION__, shot_ext->shot.ctl.request.frameCount);
        shot_ext->request_scp = 0;
        shot_ext->request_scc = 0;
    }

    shot_ext->shot.ctl.scaler.cropRegion[0] = request_shot->shot.ctl.scaler.cropRegion[0];
    shot_ext->shot.ctl.scaler.cropRegion[1] = request_shot->shot.ctl.scaler.cropRegion[1];
    shot_ext->shot.ctl.scaler.cropRegion[2] = request_shot->shot.ctl.scaler.cropRegion[2];

    m_entryProcessingIndex = newProcessingIndex;
    return newProcessingIndex;
}

void RequestManager::NotifyStreamOutput(uint32_t frameCnt)
{
    int index;

    Mutex::Autolock lock(m_requestMutex);
    ALOGV("DEBUG(%s): frameCnt(%d)", __FUNCTION__, frameCnt);

    index = FindEntryIndexByFrameCnt(frameCnt);
    if (index == -1) {
        ALOGE("ERR(%s): Cannot find entry for frameCnt(%d)", __FUNCTION__, frameCnt);
        return;
    }
    ALOGV("DEBUG(%s): frameCnt(%d), last cnt (%d)", __FUNCTION__, frameCnt,   entries[index].output_stream_count);

    entries[index].output_stream_count--;  //TODO : match stream id also
    CheckCompleted(index);
}

void RequestManager::CheckCompleted(int index)
{
    if ((entries[index].status == METADONE || entries[index].status == COMPLETED)
        || (entries[index].status == REQUESTED && entries[index].internal_shot.isReprocessing)) {
        if (entries[index].output_stream_count <= 0) {
            ALOGV("(%s): Completed(index:%d)(frameCnt:%d)(reprocess:%d)", __FUNCTION__,
                    index, entries[index].internal_shot.shot.ctl.request.frameCount, entries[index].internal_shot.isReprocessing);
            entries[index].status = COMPLETED;
            if (m_lastCompletedFrameCnt + 1 == entries[index].internal_shot.shot.ctl.request.frameCount)
                m_mainThread->SetSignal(SIGNAL_MAIN_STREAM_OUTPUT_DONE);
        }
    }
}

int RequestManager::GetCompletedIndex()
{
    return FindEntryIndexByFrameCnt(m_lastCompletedFrameCnt + 1);
}

void  RequestManager::pushSensorQ(int index)
{
    Mutex::Autolock lock(m_requestMutex);
    m_sensorQ.push_back(index);
}

int RequestManager::popSensorQ()
{
   List<int>::iterator sensor_token;
   int index;

    Mutex::Autolock lock(m_requestMutex);

    if(m_sensorQ.size() == 0)
        return -1;

    sensor_token = m_sensorQ.begin()++;
    index = *sensor_token;
    m_sensorQ.erase(sensor_token);

    return (index);
}

void RequestManager::releaseSensorQ()
{
    List<int>::iterator r;

    Mutex::Autolock lock(m_requestMutex);
    ALOGV("(%s)m_sensorQ.size : %d", __FUNCTION__, m_sensorQ.size());

    while(m_sensorQ.size() > 0){
        r  = m_sensorQ.begin()++;
        m_sensorQ.erase(r);
    }
    return;
}

void RequestManager::ApplyDynamicMetadata(struct camera2_shot_ext *shot_ext)
{
    int index;
    struct camera2_shot_ext * request_shot;
    nsecs_t timeStamp;
    int i;

    Mutex::Autolock lock(m_requestMutex);
    ALOGV("DEBUG(%s): frameCnt(%d)", __FUNCTION__, shot_ext->shot.ctl.request.frameCount);

    for (i = 0 ; i < NUM_MAX_REQUEST_MGR_ENTRY ; i++) {
        if((entries[i].internal_shot.shot.ctl.request.frameCount == shot_ext->shot.ctl.request.frameCount)
            && (entries[i].status == CAPTURED)){
            entries[i].status = METADONE;
            break;
        }
    }

    if (i == NUM_MAX_REQUEST_MGR_ENTRY){
        ALOGE("[%s] no entry found(framecount:%d)", __FUNCTION__, shot_ext->shot.ctl.request.frameCount);
        return;
    }

    request_manager_entry * newEntry = &(entries[i]);
    request_shot = &(newEntry->internal_shot);

    timeStamp = request_shot->shot.dm.sensor.timeStamp;
    memcpy(&(request_shot->shot.dm), &(shot_ext->shot.dm), sizeof(struct camera2_dm));
    request_shot->shot.dm.sensor.timeStamp = timeStamp;
    m_lastTimeStamp = timeStamp;
    CheckCompleted(i);
}

void    RequestManager::UpdateIspParameters(struct camera2_shot_ext *shot_ext, int frameCnt, ctl_request_info_t *ctl_info)
{
    int index, targetStreamIndex;
    struct camera2_shot_ext * request_shot;

    ALOGV("DEBUG(%s): updating info with frameCnt(%d)", __FUNCTION__, frameCnt);
    if (frameCnt < 0)
        return;

    index = FindEntryIndexByFrameCnt(frameCnt);
    if (index == -1) {
        ALOGE("ERR(%s): Cannot find entry for frameCnt(%d)", __FUNCTION__, frameCnt);
        return;
    }

    request_manager_entry * newEntry = &(entries[index]);
    request_shot = &(newEntry->internal_shot);
    memcpy(&(shot_ext->shot.ctl), &(request_shot->shot.ctl), sizeof(struct camera2_ctl));

    /* Default Settings */
    shot_ext->request_sensor = 1;
    shot_ext->dis_bypass = 1;
    shot_ext->dnr_bypass = 1;
    shot_ext->fd_bypass = 1;
    shot_ext->drc_bypass = 1;
    shot_ext->setfile = 0;
    shot_ext->request_scc = 0;
    shot_ext->request_scp = 0;

    /* Set Frame Count */
    shot_ext->shot.ctl.request.frameCount = frameCnt;

    /* Set Reprocessing */
    shot_ext->isReprocessing = request_shot->isReprocessing;
    shot_ext->reprocessInput = request_shot->reprocessInput;

    shot_ext->awb_mode_dm = request_shot->awb_mode_dm;

    /* Set Crop Region */
    shot_ext->shot.ctl.scaler.cropRegion[0] = request_shot->shot.ctl.scaler.cropRegion[0];
    shot_ext->shot.ctl.scaler.cropRegion[1] = request_shot->shot.ctl.scaler.cropRegion[1];
    shot_ext->shot.ctl.scaler.cropRegion[2] = request_shot->shot.ctl.scaler.cropRegion[2];

    // mapping flash UI mode from aeMode
    if (request_shot->shot.ctl.aa.aeMode >= AA_AEMODE_ON) {
        if (request_shot->shot.ctl.aa.captureIntent == AA_CAPTURE_INTENT_PREVIEW)
            ctl_info->flash.i_flashMode = request_shot->shot.ctl.aa.aeMode;
        else if (request_shot->shot.ctl.aa.captureIntent == AA_CAPTURE_INTENT_VIDEO_RECORD)
            ctl_info->flash.i_flashMode = request_shot->shot.ctl.aa.aeMode;
        request_shot->shot.ctl.aa.aeMode = AA_AEMODE_ON;
    }
    /* Apply metering mode */
    if (request_shot->metering_mode > AA_AEMODE_METERING_NONE) {
        request_shot->shot.ctl.aa.aeMode = (enum aa_aemode)request_shot->metering_mode;
        ctl_info->ae.aeMeteringMode = request_shot->metering_mode;
    }

    // Apply ae/awb lock or unlock
    if (request_shot->ae_lock == AEMODE_LOCK_ON)
            request_shot->shot.ctl.aa.aeMode = AA_AEMODE_LOCKED;
    if (request_shot->awb_lock == AWBMODE_LOCK_ON)
            request_shot->shot.ctl.aa.awbMode = AA_AWBMODE_LOCKED;

    /* Set AA Mode */
    if (m_lastAaMode == request_shot->shot.ctl.aa.mode) {
        shot_ext->shot.ctl.aa.mode = (enum aa_mode)(0);
    } else {
        shot_ext->shot.ctl.aa.mode = request_shot->shot.ctl.aa.mode;
        m_lastAaMode = (int)(shot_ext->shot.ctl.aa.mode);
    }

    /* Set AE Mode */
    if (m_lastAeMode == request_shot->shot.ctl.aa.aeMode) {
        shot_ext->shot.ctl.aa.aeMode = (enum aa_aemode)(0);
    } else {
        shot_ext->shot.ctl.aa.aeMode = request_shot->shot.ctl.aa.aeMode;
        m_lastAeMode = (int)(shot_ext->shot.ctl.aa.aeMode);
    }

    /* Set AWB Mode */
    if (m_lastAwbMode == request_shot->shot.ctl.aa.awbMode) {
        shot_ext->shot.ctl.aa.awbMode = (enum aa_awbmode)(0);
    } else {
        shot_ext->shot.ctl.aa.awbMode = request_shot->shot.ctl.aa.awbMode;
        m_lastAwbMode = (int)(shot_ext->shot.ctl.aa.awbMode);
    }

    /* Set EV Value */
    if (m_lastAeComp == request_shot->shot.ctl.aa.aeExpCompensation) {
        shot_ext->shot.ctl.aa.aeExpCompensation = 0;
    } else {
        shot_ext->shot.ctl.aa.aeExpCompensation = request_shot->shot.ctl.aa.aeExpCompensation;
        m_lastAeComp = (int)(shot_ext->shot.ctl.aa.aeExpCompensation);
    }

    /* Set Effect */
    shot_ext->shot.ctl.color.mode = request_shot->shot.ctl.color.mode;

    /* Set VDIS */
    if (request_shot->shot.ctl.aa.videoStabilizationMode) {
        m_vdisBubbleEn = true;
        shot_ext->dis_bypass = 0;
        shot_ext->dnr_bypass = 0;
    } else {
        m_vdisBubbleEn = false;
        shot_ext->dis_bypass = 1;
        shot_ext->dnr_bypass = 1;
    }

    shot_ext->shot.ctl.aa.afTrigger = 0;
    /* Set Output Streams */
    targetStreamIndex = newEntry->internal_shot.shot.ctl.request.outputStreams[0];
    shot_ext->shot.ctl.request.outputStreams[0] = targetStreamIndex;
    if (targetStreamIndex & MASK_OUTPUT_SCP)
        shot_ext->request_scp = 1;

    if (targetStreamIndex & MASK_OUTPUT_SCC)
        shot_ext->request_scc = 1;

    if (shot_ext->shot.ctl.stats.faceDetectMode != FACEDETECT_MODE_OFF)
        shot_ext->fd_bypass = 0;

    /* Set FPS Range */
    shot_ext->shot.ctl.aa.aeTargetFpsRange[0] = request_shot->shot.ctl.aa.aeTargetFpsRange[0];
    shot_ext->shot.ctl.aa.aeTargetFpsRange[1] = request_shot->shot.ctl.aa.aeTargetFpsRange[1];

    ALOGV("(%s): applied aa(%d) aemode(%d) expComp(%d), awb(%d) afmode(%d), ", __FUNCTION__,
    (int)(shot_ext->shot.ctl.aa.mode), (int)(shot_ext->shot.ctl.aa.aeMode),
    (int)(shot_ext->shot.ctl.aa.aeExpCompensation), (int)(shot_ext->shot.ctl.aa.awbMode),
    (int)(shot_ext->shot.ctl.aa.afMode));
}

bool    RequestManager::IsVdisEnable(void)
{
        return m_vdisBubbleEn;
}

int     RequestManager::FindEntryIndexByFrameCnt(uint32_t frameCnt)
{
    for (int i = 0 ; i < NUM_MAX_REQUEST_MGR_ENTRY ; i++) {
        if (entries[i].internal_shot.shot.ctl.request.frameCount == frameCnt)
            return i;
    }
    return -1;
}

void    RequestManager::RegisterTimestamp(uint32_t frameCnt, nsecs_t * frameTime)
{
    int index = FindEntryIndexByFrameCnt(frameCnt);
    if (index == -1) {
        ALOGE("ERR(%s): Cannot find entry for frameCnt(%d)", __FUNCTION__, frameCnt);
        return;
    }

    request_manager_entry * currentEntry = &(entries[index]);
    if (currentEntry->internal_shot.isReprocessing == 1) {
        ALOGV("DEBUG(%s): REPROCESSING : preserving timestamp for reqIndex(%d) frameCnt(%d) (%lld)", __FUNCTION__,
        index, frameCnt, currentEntry->internal_shot.shot.dm.sensor.timeStamp);
    } else {
        currentEntry->internal_shot.shot.dm.sensor.timeStamp = *((uint64_t*)frameTime);
        ALOGV("DEBUG(%s): applied timestamp for reqIndex(%d) frameCnt(%d) (%lld)", __FUNCTION__,
            index, frameCnt, currentEntry->internal_shot.shot.dm.sensor.timeStamp);
    }
}


nsecs_t  RequestManager::GetTimestampByFrameCnt(uint32_t frameCnt)
{
    int index = FindEntryIndexByFrameCnt(frameCnt);
    if (index == -1) {
        ALOGE("ERR(%s): Cannot find entry for frameCnt(%d) returning saved time(%lld)", __FUNCTION__, frameCnt, m_lastTimeStamp);
        return m_lastTimeStamp;
    }
    else
        return GetTimestamp(index);
}

nsecs_t  RequestManager::GetTimestamp(int index)
{
    Mutex::Autolock lock(m_requestMutex);
    if (index < 0 || index >= NUM_MAX_REQUEST_MGR_ENTRY) {
        ALOGE("ERR(%s): Request entry outside of bounds (%d)", __FUNCTION__, index);
        return 0;
    }

    request_manager_entry * currentEntry = &(entries[index]);
    nsecs_t frameTime = currentEntry->internal_shot.shot.dm.sensor.timeStamp;
    if (frameTime == 0) {
        ALOGV("DEBUG(%s): timestamp null,  returning saved value", __FUNCTION__);
        frameTime = m_lastTimeStamp;
    }
    ALOGV("DEBUG(%s): Returning timestamp for reqIndex(%d) (%lld)", __FUNCTION__, index, frameTime);
    return frameTime;
}

uint8_t  RequestManager::GetOutputStreamByFrameCnt(uint32_t frameCnt)
{
    int index = FindEntryIndexByFrameCnt(frameCnt);
    if (index == -1) {
        ALOGE("ERR(%s): Cannot find entry for frameCnt(%d)", __FUNCTION__, frameCnt);
        return 0;
    }
    else
        return GetOutputStream(index);
}

uint8_t  RequestManager::GetOutputStream(int index)
{
    Mutex::Autolock lock(m_requestMutex);
    if (index < 0 || index >= NUM_MAX_REQUEST_MGR_ENTRY) {
        ALOGE("ERR(%s): Request entry outside of bounds (%d)", __FUNCTION__, index);
        return 0;
    }

    request_manager_entry * currentEntry = &(entries[index]);
    return currentEntry->internal_shot.shot.ctl.request.outputStreams[0];
}

camera2_shot_ext *  RequestManager::GetInternalShotExtByFrameCnt(uint32_t frameCnt)
{
    int index = FindEntryIndexByFrameCnt(frameCnt);
    if (index == -1) {
        ALOGE("ERR(%s): Cannot find entry for frameCnt(%d)", __FUNCTION__, frameCnt);
        return 0;
    }
    else
        return GetInternalShotExt(index);
}

camera2_shot_ext *  RequestManager::GetInternalShotExt(int index)
{
    Mutex::Autolock lock(m_requestMutex);
    if (index < 0 || index >= NUM_MAX_REQUEST_MGR_ENTRY) {
        ALOGE("ERR(%s): Request entry outside of bounds (%d)", __FUNCTION__, index);
        return 0;
    }

    request_manager_entry * currentEntry = &(entries[index]);
    return &currentEntry->internal_shot;
}

int     RequestManager::FindFrameCnt(struct camera2_shot_ext * shot_ext)
{
    Mutex::Autolock lock(m_requestMutex);
    int i;

    if (m_numOfEntries == 0) {
        ALOGV("DBG(%s): No Entry found", __FUNCTION__);
        return -1;
    }

    for (i = 0 ; i < NUM_MAX_REQUEST_MGR_ENTRY ; i++) {
        if(entries[i].internal_shot.shot.ctl.request.frameCount != shot_ext->shot.ctl.request.frameCount)
            continue;

        if (entries[i].status == REQUESTED) {
            entries[i].status = CAPTURED;
            return entries[i].internal_shot.shot.ctl.request.frameCount;
        }
    }
    ALOGV("(%s): No Entry found frame count(%d)", __FUNCTION__, shot_ext->shot.ctl.request.frameCount);

    return -1;
}

void     RequestManager::SetInitialSkip(int count)
{
    ALOGV("(%s): Pipeline Restarting. setting cnt(%d) - current(%d)", __FUNCTION__, count, m_sensorPipelineSkipCnt);
    if (count > m_sensorPipelineSkipCnt)
        m_sensorPipelineSkipCnt = count;
}

int     RequestManager::GetSkipCnt()
{
    ALOGV("(%s): skip cnt(%d)", __FUNCTION__, m_sensorPipelineSkipCnt);
    return m_sensorPipelineSkipCnt;
}

void RequestManager::DecreaseSkipCnt()
{
    m_sensorPipelineSkipCnt--;
    ALOGV("(%s): decreased skip cnt(%d)", __FUNCTION__, m_sensorPipelineSkipCnt);
}

void RequestManager::Dump(void)
{
    int i = 0;
    request_manager_entry * currentEntry;
    ALOGD("## Dump  totalentry(%d), insert(%d), processing(%d), frame(%d)",
    m_numOfEntries,m_entryInsertionIndex,m_entryProcessingIndex, m_entryFrameOutputIndex);

    for (i = 0 ; i < NUM_MAX_REQUEST_MGR_ENTRY ; i++) {
        currentEntry =  &(entries[i]);
        ALOGD("[%2d] status[%d] frameCnt[%3d] numOutput[%d] outstream[0]-%x ", i,
        currentEntry->status, currentEntry->internal_shot.shot.ctl.request.frameCount,
            currentEntry->output_stream_count,
            currentEntry->internal_shot.shot.ctl.request.outputStreams[0]);
    }
}

int     RequestManager::GetNextIndex(int index)
{
    index++;
    if (index >= NUM_MAX_REQUEST_MGR_ENTRY)
        index = 0;

    return index;
}

int     RequestManager::GetPrevIndex(int index)
{
    index--;
    if (index < 0)
        index = NUM_MAX_REQUEST_MGR_ENTRY-1;

    return index;
}

#define AF_MODE_CAF(x) ((((x) == AA_AFMODE_CONTINUOUS_VIDEO) || \
                         ((x) == AA_AFMODE_CONTINUOUS_PICTURE) )? \
                         true : false)

#define AF_MODE_NORMAL(x) ((((x) == AA_AFMODE_AUTO) || \
                            ((x) == AA_AFMODE_MACRO))? true : false)

#define AF_MODE_NORMAL_MANUAL(x) ((((x) == AA_AFMODE_AUTO) || \
                                   ((x) == AA_AFMODE_MACRO) || \
                                   ((x) == AA_AFMODE_MANUAL))? true : false)

ExynosCameraHWInterface2::ExynosCameraHWInterface2(int cameraId, camera2_device_t *dev, ExynosCamera2 * camera, int *openInvalid):
            m_requestQueueOps(NULL),
            m_frameQueueOps(NULL),
            m_callbackCookie(NULL),
            m_isRequestQueueNull(true),
            m_halDevice(dev),
            m_ionCameraClient(0),
            m_isIspStarted(false),
            m_sccLocalBufferValid(false),
            m_cameraId(cameraId),
            m_wideAspect(false),
            m_zoomRatio(1),
            m_vdisBubbleCnt(0),
            m_vdisDupFrame(0),
            m_isAlreadyRegistered(false),
            m_scpForceSuspended(false),
            m_afState(HAL_AFSTATE_INACTIVE),
            m_afTriggerId(0),
            m_afMode(NO_CHANGE),
            m_afMode2(NO_CHANGE),
            m_IsAfModeUpdateRequired(false),
            m_IsAfLockRequired(false),
            m_nightCaptureCnt(0),
            m_nightCaptureFrameCnt(0),
            m_lastSceneMode(0)
{
    ALOGD("(%s): ENTER", __FUNCTION__);
    int ret = 0;
    int res = 0;

    m_exynosPictureCSC = NULL;
    m_exynosVideoCSC = NULL;

    if (!m_grallocHal) {
        ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t **)&m_grallocHal);
        if (ret)
            ALOGE("ERR(%s):Fail on loading gralloc HAL", __FUNCTION__);
    }

    m_camera2 = camera;
    m_ionCameraClient = createIonClient(m_ionCameraClient);
    if(m_ionCameraClient == 0)
        ALOGE("ERR(%s):Fail on ion_client_create", __FUNCTION__);

    m_mainThread    = new MainThread(this);
    m_requestManager = new RequestManager((SignalDrivenThread*)(m_mainThread.get()));
    *openInvalid = InitializeISPChain();
    if (*openInvalid < 0) {
        ALOGD("(%s): ISP chain init failed. exiting", __FUNCTION__);
        // clean process
        // 1. close video nodes
        // SCP
        res = exynos_v4l2_close(m_camera_info.scp.fd);
        if (res != NO_ERROR ) {
            ALOGE("ERR(%s): exynos_v4l2_close failed(%d)",__FUNCTION__ , res);
        }
        // SCC
        res = exynos_v4l2_close(m_camera_info.capture.fd);
        if (res != NO_ERROR ) {
            ALOGE("ERR(%s): exynos_v4l2_close failed(%d)",__FUNCTION__ , res);
        }
        // Sensor
        res = exynos_v4l2_close(m_camera_info.sensor.fd);
        if (res != NO_ERROR ) {
            ALOGE("ERR(%s): exynos_v4l2_close failed(%d)",__FUNCTION__ , res);
        }
        // ISP
        res = exynos_v4l2_close(m_camera_info.isp.fd);
        if (res != NO_ERROR ) {
            ALOGE("ERR(%s): exynos_v4l2_close failed(%d)",__FUNCTION__ , res);
        }
    } else {
        m_sensorThread  = new SensorThread(this);
        m_mainThread->Start("MainThread", PRIORITY_DEFAULT, 0);
        m_sensorThread->Start("SensorThread", PRIORITY_DEFAULT, 0);
        ALOGV("DEBUG(%s): created sensorthread ", __FUNCTION__);

        m_jpegEncThread = new JpegEncThread(this);
        m_jpegEncThread->Start("JpegEncThread", PRIORITY_DEFAULT, 0);

        m_miscThread = new MiscThread(this);
        m_miscThread->Start("MiscThread", PRIORITY_DEFAULT, 0);

        for (int i = 0 ; i < STREAM_ID_LAST+1 ; i++)
            m_subStreams[i].type =  SUBSTREAM_TYPE_NONE;
        CSC_METHOD cscMethod = CSC_METHOD_HW;
        m_exynosPictureCSC = csc_init(cscMethod);
        if (m_exynosPictureCSC == NULL)
            ALOGE("ERR(%s): csc_init() fail", __FUNCTION__);
        csc_set_hw_property(m_exynosPictureCSC, CSC_HW_PROPERTY_FIXED_NODE, PICTURE_GSC_NODE_NUM);

        m_exynosVideoCSC = csc_init(cscMethod);
        if (m_exynosVideoCSC == NULL)
            ALOGE("ERR(%s): csc_init() fail", __FUNCTION__);
        csc_set_hw_property(m_exynosVideoCSC, CSC_HW_PROPERTY_FIXED_NODE, VIDEO_GSC_NODE_NUM);

        m_setExifFixedAttribute();

        // contol information clear
        // flash
        m_ctlInfo.flash.i_flashMode = AA_AEMODE_ON;
        m_ctlInfo.flash.m_afFlashFired = false;
        m_ctlInfo.flash.m_flashEnableFlg = false;
        m_ctlInfo.flash.m_flashFrameCount = 0;
        m_ctlInfo.flash.m_flashState = FLASH_STATE_NONE;
        m_ctlInfo.flash.m_flashTimeOut = 0;
        m_ctlInfo.flash.m_flashDecisionResult = false;
        m_ctlInfo.flash.m_flashTorchMode = false;
        m_ctlInfo.flash.m_precaptureState = 0;
        m_ctlInfo.flash.m_precaptureTriggerId = 0;
        // ae
        m_ctlInfo.ae.aeStateNoti = AE_STATE_INACTIVE;
        // scene
        m_ctlInfo.scene.prevSceneMode = AA_SCENE_MODE_MAX;
    }
    ALOGD("(%s): EXIT", __FUNCTION__);
}

ExynosCameraHWInterface2::~ExynosCameraHWInterface2()
{
    ALOGD("(%s): ENTER", __FUNCTION__);
    this->release();
    ALOGD("(%s): EXIT", __FUNCTION__);
}

void ExynosCameraHWInterface2::release()
{
    int i, res;
    ALOGD("(HAL2::release): ENTER");

    if (m_streamThreads[1] != NULL) {
        m_streamThreads[1]->release();
        m_streamThreads[1]->SetSignal(SIGNAL_THREAD_TERMINATE);
    }

    if (m_streamThreads[0] != NULL) {
        m_streamThreads[0]->release();
        m_streamThreads[0]->SetSignal(SIGNAL_THREAD_TERMINATE);
    }

    if (m_sensorThread != NULL) {
        m_sensorThread->release();
    }

    if (m_mainThread != NULL) {
        m_mainThread->release();
    }

    if (m_jpegEncThread != NULL) {
        m_jpegEncThread->release();
    }

    if (m_miscThread != NULL)
        m_miscThread->release();

    if (m_exynosPictureCSC)
        csc_deinit(m_exynosPictureCSC);
    m_exynosPictureCSC = NULL;

    if (m_exynosVideoCSC)
        csc_deinit(m_exynosVideoCSC);
    m_exynosVideoCSC = NULL;

    if (m_streamThreads[1] != NULL) {
        ALOGD("(HAL2::release): START Waiting for (indirect) stream thread 1 termination");
        while (!m_streamThreads[1]->IsTerminated())
            usleep(SIG_WAITING_TICK);
        ALOGD("(HAL2::release): END   Waiting for (indirect) stream thread 1 termination");
        m_streamThreads[1] = NULL;
    }

    if (m_streamThreads[0] != NULL) {
        ALOGD("(HAL2::release): START Waiting for (indirect) stream thread 0 termination");
        while (!m_streamThreads[0]->IsTerminated())
            usleep(SIG_WAITING_TICK);
        ALOGD("(HAL2::release): END   Waiting for (indirect) stream thread 0 termination");
        m_streamThreads[0] = NULL;
    }

    if (m_sensorThread != NULL) {
        ALOGD("(HAL2::release): START Waiting for (indirect) sensor thread termination");
        while (!m_sensorThread->IsTerminated())
            usleep(SIG_WAITING_TICK);
        ALOGD("(HAL2::release): END   Waiting for (indirect) sensor thread termination");
        m_sensorThread = NULL;
    }

    if (m_mainThread != NULL) {
        ALOGD("(HAL2::release): START Waiting for (indirect) main thread termination");
        while (!m_mainThread->IsTerminated())
            usleep(SIG_WAITING_TICK);
        ALOGD("(HAL2::release): END   Waiting for (indirect) main thread termination");
        m_mainThread = NULL;
    }

    if (m_requestManager != NULL) {
        delete m_requestManager;
        m_requestManager = NULL;
    }

    for (i = 0; i < NUM_BAYER_BUFFERS; i++)
        freeCameraMemory(&m_camera_info.sensor.buffer[i], m_camera_info.sensor.planes);

    if (m_sccLocalBufferValid) {
        for (i = 0; i < NUM_SCC_BUFFERS; i++)
            freeCameraMemory(&m_sccLocalBuffer[i], 2);
    }
    else {
        for (i = 0; i < NUM_SCC_BUFFERS; i++)
            freeCameraMemory(&m_camera_info.capture.buffer[i], m_camera_info.capture.planes);
    }

    ALOGV("DEBUG(%s): calling exynos_v4l2_close - sensor", __FUNCTION__);
    res = exynos_v4l2_close(m_camera_info.sensor.fd);
    if (res != NO_ERROR ) {
        ALOGE("ERR(%s): exynos_v4l2_close failed(%d)",__FUNCTION__ , res);
    }

    ALOGV("DEBUG(%s): calling exynos_v4l2_close - isp", __FUNCTION__);
    res = exynos_v4l2_close(m_camera_info.isp.fd);
    if (res != NO_ERROR ) {
        ALOGE("ERR(%s): exynos_v4l2_close failed(%d)",__FUNCTION__ , res);
    }

    ALOGV("DEBUG(%s): calling exynos_v4l2_close - capture", __FUNCTION__);
    res = exynos_v4l2_close(m_camera_info.capture.fd);
    if (res != NO_ERROR ) {
        ALOGE("ERR(%s): exynos_v4l2_close failed(%d)",__FUNCTION__ , res);
    }

    ALOGV("DEBUG(%s): calling exynos_v4l2_close - scp", __FUNCTION__);
    res = exynos_v4l2_close(m_camera_info.scp.fd);
    if (res != NO_ERROR ) {
        ALOGE("ERR(%s): exynos_v4l2_close failed(%d)",__FUNCTION__ , res);
    }
    ALOGV("DEBUG(%s): calling deleteIonClient", __FUNCTION__);
    deleteIonClient(m_ionCameraClient);

    ALOGD("(HAL2::release): EXIT");
}

int ExynosCameraHWInterface2::InitializeISPChain()
{
    char node_name[30];
    int fd = 0;
    int i;
    int ret = 0;

    /* Open Sensor */
    memset(&node_name, 0x00, sizeof(char[30]));
    sprintf(node_name, "%s%d", NODE_PREFIX, 40);
    fd = exynos_v4l2_open(node_name, O_RDWR, 0);

    if (fd < 0) {
        ALOGE("ERR(%s): failed to open sensor video node (%s) fd (%d)", __FUNCTION__,node_name, fd);
    }
    else {
        ALOGV("DEBUG(%s): sensor video node opened(%s) fd (%d)", __FUNCTION__,node_name, fd);
    }
    m_camera_info.sensor.fd = fd;

    /* Open ISP */
    memset(&node_name, 0x00, sizeof(char[30]));
    sprintf(node_name, "%s%d", NODE_PREFIX, 41);
    fd = exynos_v4l2_open(node_name, O_RDWR, 0);

    if (fd < 0) {
        ALOGE("ERR(%s): failed to open isp video node (%s) fd (%d)", __FUNCTION__,node_name, fd);
    }
    else {
        ALOGV("DEBUG(%s): isp video node opened(%s) fd (%d)", __FUNCTION__,node_name, fd);
    }
    m_camera_info.isp.fd = fd;

    /* Open ScalerC */
    memset(&node_name, 0x00, sizeof(char[30]));
    sprintf(node_name, "%s%d", NODE_PREFIX, 42);
    fd = exynos_v4l2_open(node_name, O_RDWR, 0);

    if (fd < 0) {
        ALOGE("ERR(%s): failed to open capture video node (%s) fd (%d)", __FUNCTION__,node_name, fd);
    }
    else {
        ALOGV("DEBUG(%s): capture video node opened(%s) fd (%d)", __FUNCTION__,node_name, fd);
    }
    m_camera_info.capture.fd = fd;

    /* Open ScalerP */
    memset(&node_name, 0x00, sizeof(char[30]));
    sprintf(node_name, "%s%d", NODE_PREFIX, 44);
    fd = exynos_v4l2_open(node_name, O_RDWR, 0);
    if (fd < 0) {
        ALOGE("DEBUG(%s): failed to open preview video node (%s) fd (%d)", __FUNCTION__,node_name, fd);
    }
    else {
        ALOGV("DEBUG(%s): preview video node opened(%s) fd (%d)", __FUNCTION__,node_name, fd);
    }
    m_camera_info.scp.fd = fd;

    if(m_cameraId == 0)
        m_camera_info.sensor_id = SENSOR_NAME_S5K4E5;
    else
        m_camera_info.sensor_id = SENSOR_NAME_S5K6A3;

    memset(&m_camera_info.dummy_shot, 0x00, sizeof(struct camera2_shot_ext));
    m_camera_info.dummy_shot.shot.ctl.request.metadataMode = METADATA_MODE_FULL;
    m_camera_info.dummy_shot.shot.magicNumber = 0x23456789;

    m_camera_info.dummy_shot.dis_bypass = 1;
    m_camera_info.dummy_shot.dnr_bypass = 1;
    m_camera_info.dummy_shot.fd_bypass = 1;

    /*sensor setting*/
    m_camera_info.dummy_shot.shot.ctl.sensor.exposureTime = 0;
    m_camera_info.dummy_shot.shot.ctl.sensor.frameDuration = 33 * 1000 * 1000;
    m_camera_info.dummy_shot.shot.ctl.sensor.sensitivity = 0;

    m_camera_info.dummy_shot.shot.ctl.request.frameCount = -1;

    m_camera_info.dummy_shot.shot.ctl.scaler.cropRegion[0] = 0;
    m_camera_info.dummy_shot.shot.ctl.scaler.cropRegion[1] = 0;

    /*request setting*/
    m_camera_info.dummy_shot.request_sensor = 1;
    m_camera_info.dummy_shot.request_scc = 0;
    m_camera_info.dummy_shot.request_scp = 0;
    m_camera_info.dummy_shot.shot.ctl.request.outputStreams[0] = 0;

    m_camera_info.dummy_shot.shot.ctl.aa.aeTargetFpsRange[0] = 30;
    m_camera_info.dummy_shot.shot.ctl.aa.aeTargetFpsRange[1] = 30;

    m_camera_info.sensor.width = m_camera2->getSensorRawW();
    m_camera_info.sensor.height = m_camera2->getSensorRawH();

    m_camera_info.sensor.format = V4L2_PIX_FMT_SBGGR16;
    m_camera_info.sensor.planes = 2;
    m_camera_info.sensor.buffers = NUM_BAYER_BUFFERS;
    m_camera_info.sensor.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    m_camera_info.sensor.memory = V4L2_MEMORY_DMABUF;

    for(i = 0; i < m_camera_info.sensor.buffers; i++){
        initCameraMemory(&m_camera_info.sensor.buffer[i], m_camera_info.sensor.planes);
        m_camera_info.sensor.buffer[i].size.extS[0] = m_camera_info.sensor.width*m_camera_info.sensor.height*2;
        m_camera_info.sensor.buffer[i].size.extS[1] = 8*1024; // HACK, driver use 8*1024, should be use predefined value
        allocCameraMemory(m_ionCameraClient, &m_camera_info.sensor.buffer[i], m_camera_info.sensor.planes, 1<<1);
    }

    m_camera_info.isp.width = m_camera_info.sensor.width;
    m_camera_info.isp.height = m_camera_info.sensor.height;
    m_camera_info.isp.format = m_camera_info.sensor.format;
    m_camera_info.isp.planes = m_camera_info.sensor.planes;
    m_camera_info.isp.buffers = m_camera_info.sensor.buffers;
    m_camera_info.isp.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    m_camera_info.isp.memory = V4L2_MEMORY_DMABUF;

    for(i = 0; i < m_camera_info.isp.buffers; i++){
        initCameraMemory(&m_camera_info.isp.buffer[i], m_camera_info.isp.planes);
        m_camera_info.isp.buffer[i].size.extS[0]    = m_camera_info.sensor.buffer[i].size.extS[0];
        m_camera_info.isp.buffer[i].size.extS[1]    = m_camera_info.sensor.buffer[i].size.extS[1];
        m_camera_info.isp.buffer[i].fd.extFd[0]     = m_camera_info.sensor.buffer[i].fd.extFd[0];
        m_camera_info.isp.buffer[i].fd.extFd[1]     = m_camera_info.sensor.buffer[i].fd.extFd[1];
        m_camera_info.isp.buffer[i].virt.extP[0]    = m_camera_info.sensor.buffer[i].virt.extP[0];
        m_camera_info.isp.buffer[i].virt.extP[1]    = m_camera_info.sensor.buffer[i].virt.extP[1];
    };

    /* init ISP */
    ret = cam_int_s_input(&(m_camera_info.isp), m_camera_info.sensor_id);
    if (ret < 0) {
        ALOGE("ERR(%s): cam_int_s_input(%d) failed!!!! ",  __FUNCTION__, m_camera_info.sensor_id);
        return false;
    }
    cam_int_s_fmt(&(m_camera_info.isp));
    ALOGV("DEBUG(%s): isp calling reqbuf", __FUNCTION__);
    cam_int_reqbufs(&(m_camera_info.isp));
    ALOGV("DEBUG(%s): isp calling querybuf", __FUNCTION__);
    ALOGV("DEBUG(%s): isp mem alloc done",  __FUNCTION__);

    /* init Sensor */
    cam_int_s_input(&(m_camera_info.sensor), m_camera_info.sensor_id);
    ALOGV("DEBUG(%s): sensor s_input done",  __FUNCTION__);
    if (cam_int_s_fmt(&(m_camera_info.sensor))< 0) {
        ALOGE("ERR(%s): sensor s_fmt fail",  __FUNCTION__);
    }
    ALOGV("DEBUG(%s): sensor s_fmt done",  __FUNCTION__);
    cam_int_reqbufs(&(m_camera_info.sensor));
    ALOGV("DEBUG(%s): sensor reqbuf done",  __FUNCTION__);
    for (i = 0; i < m_camera_info.sensor.buffers; i++)
        memcpy( m_camera_info.sensor.buffer[i].virt.extP[1], &(m_camera_info.dummy_shot),
                sizeof(struct camera2_shot_ext));
    for (i = 0; i < NUM_MIN_SENSOR_QBUF; i++) {
        ALOGV("(%s): sensor initial QBUF [%d]",  __FUNCTION__, i);
        cam_int_qbuf(&(m_camera_info.sensor), i);
    }
    for (i = NUM_MIN_SENSOR_QBUF; i < m_camera_info.sensor.buffers; i++)
        m_requestManager->pushSensorQ(i);

    ALOGV("== stream_on :: sensor");
    cam_int_streamon(&(m_camera_info.sensor));
    m_camera_info.sensor.status = true;

    /* init Capture */
    m_camera_info.capture.width = m_camera2->getSensorW();
    m_camera_info.capture.height = m_camera2->getSensorH();
    m_camera_info.capture.format = V4L2_PIX_FMT_YUYV;
    m_camera_info.capture.planes = 2;
    m_camera_info.capture.buffers = NUM_SCC_BUFFERS;
    m_camera_info.capture.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    m_camera_info.capture.memory = V4L2_MEMORY_DMABUF;

    m_camera_info.capture.status = false;

    return true;
}

void ExynosCameraHWInterface2::StartSCCThread(bool threadExists)
{
    ALOGV("(%s)", __FUNCTION__);
    StreamThread *AllocatedStream;
    stream_parameters_t newParameters;
    uint32_t format_actual;


    if (!threadExists) {
        m_streamThreads[1]  = new StreamThread(this, 1);
    }
    AllocatedStream = (StreamThread*)(m_streamThreads[1].get());
    if (!threadExists) {
        AllocatedStream->Start("StreamThread", PRIORITY_DEFAULT, 0);
        m_streamThreadInitialize((SignalDrivenThread*)AllocatedStream);
        AllocatedStream->m_numRegisteredStream = 1;
    }
    AllocatedStream->m_index        = 1;

    format_actual                   = HAL_PIXEL_FORMAT_YCbCr_422_I; // YUYV

    newParameters.width             = m_camera2->getSensorW();
    newParameters.height            = m_camera2->getSensorH();
    newParameters.format            = format_actual;
    newParameters.streamOps         = NULL;
    newParameters.numHwBuffers      = NUM_SCC_BUFFERS;
    newParameters.planes            = 2;

    newParameters.numSvcBufsInHal   = 0;

    newParameters.node              = &m_camera_info.capture;

    AllocatedStream->streamType     = STREAM_TYPE_INDIRECT;
    ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, AllocatedStream->m_numRegisteredStream);

    if (!threadExists) {
        if (!m_sccLocalBufferValid) {
            for (int i = 0; i < m_camera_info.capture.buffers; i++){
                initCameraMemory(&m_camera_info.capture.buffer[i], newParameters.node->planes);
                m_camera_info.capture.buffer[i].size.extS[0] = m_camera_info.capture.width*m_camera_info.capture.height*2;
                m_camera_info.capture.buffer[i].size.extS[1] = 4*1024; // HACK, driver use 4*1024, should be use predefined value
                allocCameraMemory(m_ionCameraClient, &m_camera_info.capture.buffer[i], m_camera_info.capture.planes, 1<<1);
                m_sccLocalBuffer[i] = m_camera_info.capture.buffer[i];
            }
            m_sccLocalBufferValid = true;
        }
    } else {
        if (m_sccLocalBufferValid) {
             for (int i = 0; i < m_camera_info.capture.buffers; i++)
                m_camera_info.capture.buffer[i] = m_sccLocalBuffer[i];
        } else {
            ALOGE("(%s): SCC Thread starting with no buffer", __FUNCTION__);
        }
    }
    cam_int_s_input(newParameters.node, m_camera_info.sensor_id);
    m_camera_info.capture.buffers = NUM_SCC_BUFFERS;
    cam_int_s_fmt(newParameters.node);
    ALOGV("DEBUG(%s): capture calling reqbuf", __FUNCTION__);
    cam_int_reqbufs(newParameters.node);
    ALOGV("DEBUG(%s): capture calling querybuf", __FUNCTION__);

    for (int i = 0; i < newParameters.node->buffers; i++) {
        ALOGV("DEBUG(%s): capture initial QBUF [%d]",  __FUNCTION__, i);
        cam_int_qbuf(newParameters.node, i);
        newParameters.svcBufStatus[i] = ON_DRIVER;
    }

    ALOGV("== stream_on :: capture");
    if (cam_int_streamon(newParameters.node) < 0) {
        ALOGE("ERR(%s): capture stream on fail", __FUNCTION__);
    } else {
        m_camera_info.capture.status = true;
    }

    AllocatedStream->setParameter(&newParameters);
    AllocatedStream->m_activated    = true;
    AllocatedStream->m_isBufferInit = true;
}

void ExynosCameraHWInterface2::StartISP()
{
    ALOGV("== stream_on :: isp");
    cam_int_streamon(&(m_camera_info.isp));
    exynos_v4l2_s_ctrl(m_camera_info.sensor.fd, V4L2_CID_IS_S_STREAM, IS_ENABLE_STREAM);
}

int ExynosCameraHWInterface2::getCameraId() const
{
    return m_cameraId;
}

int ExynosCameraHWInterface2::setRequestQueueSrcOps(const camera2_request_queue_src_ops_t *request_src_ops)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    if ((NULL != request_src_ops) && (NULL != request_src_ops->dequeue_request)
            && (NULL != request_src_ops->free_request) && (NULL != request_src_ops->request_count)) {
        m_requestQueueOps = (camera2_request_queue_src_ops_t*)request_src_ops;
        return 0;
    }
    else {
        ALOGE("DEBUG(%s):setRequestQueueSrcOps : NULL arguments", __FUNCTION__);
        return 1;
    }
}

int ExynosCameraHWInterface2::notifyRequestQueueNotEmpty()
{
    int i = 0;

    ALOGV("(%s): START current num of entries(%d)", __FUNCTION__, m_requestManager->GetNumEntries());
    if ((NULL==m_frameQueueOps)|| (NULL==m_requestQueueOps)) {
        ALOGE("DEBUG(%s):queue ops NULL. ignoring request", __FUNCTION__);
        return 0;
    }
    m_isRequestQueueNull = false;
    if (m_requestManager->GetNumEntries() == 0)
        m_requestManager->SetInitialSkip(0);

    if (m_isIspStarted == false) {
        /* isp */
        m_camera_info.sensor.buffers = NUM_BAYER_BUFFERS;
        m_camera_info.isp.buffers = m_camera_info.sensor.buffers;
        cam_int_s_fmt(&(m_camera_info.isp));
        cam_int_reqbufs(&(m_camera_info.isp));

        /* sensor */
        if (m_camera_info.sensor.status == false) {
            cam_int_s_fmt(&(m_camera_info.sensor));
            cam_int_reqbufs(&(m_camera_info.sensor));

            for (i = 0; i < m_camera_info.sensor.buffers; i++)
                memcpy( m_camera_info.sensor.buffer[i].virt.extP[1], &(m_camera_info.dummy_shot),
                        sizeof(struct camera2_shot_ext));
            for (i = 0; i < NUM_MIN_SENSOR_QBUF; i++) {
                ALOGV("(%s): sensor initial QBUF [%d]",  __FUNCTION__, i);
                cam_int_qbuf(&(m_camera_info.sensor), i);
            }
            for (i = NUM_MIN_SENSOR_QBUF; i < m_camera_info.sensor.buffers; i++)
                m_requestManager->pushSensorQ(i);
            ALOGV("DEBUG(%s): calling sensor streamon", __FUNCTION__);
            cam_int_streamon(&(m_camera_info.sensor));
            m_camera_info.sensor.status = true;
        }
    }
    if (!(m_streamThreads[1].get())) {
        ALOGV("DEBUG(%s): stream thread 1 not exist. starting without stream", __FUNCTION__);
        StartSCCThread(false);
    } else {
        if (m_streamThreads[1]->m_activated ==  false) {
            ALOGV("DEBUG(%s): stream thread 1 suspended. restarting", __FUNCTION__);
            StartSCCThread(true);
        } else {
            if (m_camera_info.capture.status == false) {
                m_camera_info.capture.buffers = NUM_SCC_BUFFERS;
                cam_int_s_fmt(&(m_camera_info.capture));
                ALOGV("DEBUG(%s): capture calling reqbuf", __FUNCTION__);
                cam_int_reqbufs(&(m_camera_info.capture));
                ALOGV("DEBUG(%s): capture calling querybuf", __FUNCTION__);

                if (m_streamThreads[1]->streamType == STREAM_TYPE_DIRECT) {
                    StreamThread *          targetStream = m_streamThreads[1].get();
                    stream_parameters_t     *targetStreamParms = &(targetStream->m_parameters);
                    node_info_t             *currentNode = targetStreamParms->node;

                    struct v4l2_buffer v4l2_buf;
                    struct v4l2_plane  planes[VIDEO_MAX_PLANES];

                    for (i = 0 ; i < targetStreamParms->numSvcBuffers ; i++) {
                        v4l2_buf.m.planes   = planes;
                        v4l2_buf.type       = currentNode->type;
                        v4l2_buf.memory     = currentNode->memory;

                        v4l2_buf.length     = currentNode->planes;
                        v4l2_buf.index      = i;
                        ExynosBuffer metaBuf = targetStreamParms->metaBuffers[i];

                        if (i < currentNode->buffers) {
                            v4l2_buf.m.planes[0].m.fd = targetStreamParms->svcBuffers[i].fd.extFd[0];
                            v4l2_buf.m.planes[2].m.fd = targetStreamParms->svcBuffers[i].fd.extFd[1];
                            v4l2_buf.m.planes[1].m.fd = targetStreamParms->svcBuffers[i].fd.extFd[2];
                            v4l2_buf.length += targetStreamParms->metaPlanes;
                            v4l2_buf.m.planes[v4l2_buf.length-1].m.fd = metaBuf.fd.extFd[0];
                            v4l2_buf.m.planes[v4l2_buf.length-1].length = metaBuf.size.extS[0];

                            ALOGV("Qbuf metaBuf: fd(%d), length(%d) plane(%d)", metaBuf.fd.extFd[0], metaBuf.size.extS[0], v4l2_buf.length);
                            if (exynos_v4l2_qbuf(currentNode->fd, &v4l2_buf) < 0) {
                                ALOGE("ERR(%s): exynos_v4l2_qbuf() fail fd(%d)", __FUNCTION__, currentNode->fd);
                            }
                            ALOGV("DEBUG(%s): exynos_v4l2_qbuf() success fd(%d)", __FUNCTION__, currentNode->fd);
                            targetStreamParms->svcBufStatus[i]  = REQUIRES_DQ_FROM_SVC;
                        }
                        else {
                            targetStreamParms->svcBufStatus[i]  = ON_SERVICE;
                        }

                    }

                } else {
                    for (int i = 0; i < m_camera_info.capture.buffers; i++) {
                        ALOGV("DEBUG(%s): capture initial QBUF [%d]",  __FUNCTION__, i);
                        cam_int_qbuf(&(m_camera_info.capture), i);
                    }
                }
                ALOGV("== stream_on :: capture");
                if (cam_int_streamon(&(m_camera_info.capture)) < 0) {
                    ALOGE("ERR(%s): capture stream on fail", __FUNCTION__);
                } else {
                    m_camera_info.capture.status = true;
                }
            }
            if (m_scpForceSuspended) {
                m_scpForceSuspended = false;
            }
        }
    }
    if (m_isIspStarted == false) {
        StartISP();
        ALOGV("DEBUG(%s):starting sensor thread", __FUNCTION__);
        m_requestManager->SetInitialSkip(3);
        m_sensorThread->Start("SensorThread", PRIORITY_DEFAULT, 0);
        m_isIspStarted = true;
    }
    m_sensorThread->SetSignal(SIGNAL_SENSOR_START_REQ_PROCESSING);
    return 0;
}

int ExynosCameraHWInterface2::setFrameQueueDstOps(const camera2_frame_queue_dst_ops_t *frame_dst_ops)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    if ((NULL != frame_dst_ops) && (NULL != frame_dst_ops->dequeue_frame)
            && (NULL != frame_dst_ops->cancel_frame) && (NULL !=frame_dst_ops->enqueue_frame)) {
        m_frameQueueOps = (camera2_frame_queue_dst_ops_t *)frame_dst_ops;
        return 0;
    }
    else {
        ALOGE("DEBUG(%s):setFrameQueueDstOps : NULL arguments", __FUNCTION__);
        return 1;
    }
}

int ExynosCameraHWInterface2::getInProgressCount()
{
    int inProgressCount = m_requestManager->GetNumEntries();
    int jpegEncodingCount = m_jpegEncThread->getInProgressCount();
    ALOGV("DEBUG(%s): # of dequeued req (%d) jpeg(%d) = (%d)", __FUNCTION__,
        inProgressCount, jpegEncodingCount, (inProgressCount +jpegEncodingCount));
    return (inProgressCount + jpegEncodingCount);
}

int ExynosCameraHWInterface2::flushCapturesInProgress()
{
    return 0;
}

int ExynosCameraHWInterface2::constructDefaultRequest(int request_template, camera_metadata_t **request)
{
    ALOGV("DEBUG(%s): making template (%d) ", __FUNCTION__, request_template);

    if (request == NULL) return BAD_VALUE;
    if (request_template < 0 || request_template >= CAMERA2_TEMPLATE_COUNT) {
        return BAD_VALUE;
    }
    status_t res;
    // Pass 1, calculate size and allocate
    res = m_camera2->constructDefaultRequest(request_template,
            request,
            true);
    if (res != OK) {
        return res;
    }
    // Pass 2, build request
    res = m_camera2->constructDefaultRequest(request_template,
            request,
            false);
    if (res != OK) {
        ALOGE("Unable to populate new request for template %d",
                request_template);
    }

    return res;
}

int ExynosCameraHWInterface2::allocateStream(uint32_t width, uint32_t height, int format, const camera2_stream_ops_t *stream_ops,
                                    uint32_t *stream_id, uint32_t *format_actual, uint32_t *usage, uint32_t *max_buffers)
{
    ALOGD("(%s): stream width(%d) height(%d) format(%x)", __FUNCTION__,  width, height, format);
    bool useDirectOutput = false;
    StreamThread *AllocatedStream;
    stream_parameters_t newParameters;
    substream_parameters_t *subParameters;
    StreamThread *parentStream;
    status_t res;
    int allocCase = 0;

    if ((format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED || format == CAMERA2_HAL_PIXEL_FORMAT_OPAQUE)  &&
            m_camera2->isSupportedResolution(width, height)) {
        if (!(m_streamThreads[0].get())) {
            ALOGV("DEBUG(%s): stream 0 not exist", __FUNCTION__);
            allocCase = 0;
        }
        else {
            if ((m_streamThreads[0].get())->m_activated == true) {
                ALOGV("DEBUG(%s): stream 0 exists and activated.", __FUNCTION__);
                allocCase = 1;
            }
            else {
                ALOGV("DEBUG(%s): stream 0 exists and deactivated.", __FUNCTION__);
                allocCase = 2;
            }
        }

        // TODO : instead of that, use calculate aspect ratio and selection with calculated ratio.
        if ((width == 1920 && height == 1080) || (width == 1280 && height == 720)
                    || (width == 720 && height == 480) || (width == 1440 && height == 960)
                    || (width == 1344 && height == 896)) {
            m_wideAspect = true;
        } else {
            m_wideAspect = false;
        }
        ALOGV("DEBUG(%s): m_wideAspect (%d)", __FUNCTION__, m_wideAspect);

        if (allocCase == 0 || allocCase == 2) {
            *stream_id = STREAM_ID_PREVIEW;

            m_streamThreads[0]  = new StreamThread(this, *stream_id);

            AllocatedStream = (StreamThread*)(m_streamThreads[0].get());
            AllocatedStream->Start("StreamThread", PRIORITY_DEFAULT, 0);
            m_streamThreadInitialize((SignalDrivenThread*)AllocatedStream);

            *format_actual                      = HAL_PIXEL_FORMAT_EXYNOS_YV12;
            *usage                              = 0;
            *max_buffers                        = 6;

            newParameters.width                 = width;
            newParameters.height                = height;
            newParameters.format                = *format_actual;
            newParameters.streamOps             = stream_ops;
            newParameters.usage                 = *usage;
            newParameters.numHwBuffers          = NUM_SCP_BUFFERS;
            newParameters.numOwnSvcBuffers      = *max_buffers;
            newParameters.planes                = NUM_PLANES(*format_actual);
            newParameters.metaPlanes            = 1;
            newParameters.numSvcBufsInHal       = 0;
            newParameters.minUndequedBuffer     = 3;
            newParameters.needsIonMap           = true;

            newParameters.node                  = &m_camera_info.scp;
            newParameters.node->type            = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            newParameters.node->memory          = V4L2_MEMORY_DMABUF;

            AllocatedStream->streamType         = STREAM_TYPE_DIRECT;
            AllocatedStream->m_index            = 0;
            AllocatedStream->setParameter(&newParameters);
            AllocatedStream->m_activated = true;
            AllocatedStream->m_numRegisteredStream = 1;
            ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, AllocatedStream->m_numRegisteredStream);
            m_requestManager->SetDefaultParameters(m_camera2->getSensorW());
            m_camera_info.dummy_shot.shot.ctl.scaler.cropRegion[2] = m_camera2->getSensorW();
            if (m_subStreams[STREAM_ID_RECORD].type != SUBSTREAM_TYPE_NONE)
                AllocatedStream->attachSubStream(STREAM_ID_RECORD, 10);
            if (m_subStreams[STREAM_ID_PRVCB].type != SUBSTREAM_TYPE_NONE)
                AllocatedStream->attachSubStream(STREAM_ID_PRVCB, 70);
            return 0;
        } else if (allocCase == 1) {
            *stream_id = STREAM_ID_RECORD;

            subParameters = &m_subStreams[STREAM_ID_RECORD];
            memset(subParameters, 0, sizeof(substream_parameters_t));

            parentStream = (StreamThread*)(m_streamThreads[0].get());
            if (!parentStream) {
                return 1;
            }

            *format_actual = HAL_PIXEL_FORMAT_YCbCr_420_SP; // NV12M
            *usage = 0;
            *max_buffers = 6;

            subParameters->type         = SUBSTREAM_TYPE_RECORD;
            subParameters->width        = width;
            subParameters->height       = height;
            subParameters->format       = *format_actual;
            subParameters->svcPlanes     = NUM_PLANES(*format_actual);
            subParameters->streamOps     = stream_ops;
            subParameters->usage         = *usage;
            subParameters->numOwnSvcBuffers = *max_buffers;
            subParameters->numSvcBufsInHal  = 0;
            subParameters->needBufferInit    = false;
            subParameters->minUndequedBuffer = 2;

            res = parentStream->attachSubStream(STREAM_ID_RECORD, 20);
            if (res != NO_ERROR) {
                ALOGE("(%s): substream attach failed. res(%d)", __FUNCTION__, res);
                return 1;
            }
            ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, parentStream->m_numRegisteredStream);
            ALOGV("(%s): Enabling Record", __FUNCTION__);
            return 0;
        }
    }
    else if ((format == CAMERA2_HAL_PIXEL_FORMAT_ZSL)
            && (width == m_camera2->getSensorW()) && (height == m_camera2->getSensorH())) {

        if (!(m_streamThreads[1].get())) {
            ALOGV("DEBUG(%s): stream thread 1 not exist", __FUNCTION__);
            useDirectOutput = true;
        }
        else {
            ALOGV("DEBUG(%s): stream thread 1 exists and deactivated.", __FUNCTION__);
            useDirectOutput = false;
        }
        if (useDirectOutput) {
            *stream_id = STREAM_ID_ZSL;

            m_streamThreads[1]  = new StreamThread(this, *stream_id);
            AllocatedStream = (StreamThread*)(m_streamThreads[1].get());
            AllocatedStream->Start("StreamThread", PRIORITY_DEFAULT, 0);
            m_streamThreadInitialize((SignalDrivenThread*)AllocatedStream);

            *format_actual = HAL_PIXEL_FORMAT_YCbCr_422_I; // YUYV
            *usage = 0;
            *max_buffers = 6;

            newParameters.width                 = width;
            newParameters.height                = height;
            newParameters.format                = *format_actual;
            newParameters.streamOps             = stream_ops;
            newParameters.usage                 = *usage;
            newParameters.numHwBuffers          = NUM_SCC_BUFFERS;
            newParameters.numOwnSvcBuffers      = *max_buffers;
            newParameters.planes                = NUM_PLANES(*format_actual);
            newParameters.metaPlanes            = 1;

            newParameters.numSvcBufsInHal       = 0;
            newParameters.minUndequedBuffer     = 2;
            newParameters.needsIonMap           = false;

            newParameters.node                  = &m_camera_info.capture;
            newParameters.node->type            = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            newParameters.node->memory          = V4L2_MEMORY_DMABUF;

            AllocatedStream->streamType         = STREAM_TYPE_DIRECT;
            AllocatedStream->m_index            = 1;
            AllocatedStream->setParameter(&newParameters);
            AllocatedStream->m_activated = true;
            AllocatedStream->m_numRegisteredStream = 1;
            ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, AllocatedStream->m_numRegisteredStream);
            return 0;
        } else {
            bool bJpegExists = false;
            AllocatedStream = (StreamThread*)(m_streamThreads[1].get());
            subParameters = &m_subStreams[STREAM_ID_JPEG];
            if (subParameters->type == SUBSTREAM_TYPE_JPEG) {
                ALOGD("(%s): jpeg stream exists", __FUNCTION__);
                bJpegExists = true;
                AllocatedStream->detachSubStream(STREAM_ID_JPEG);
            }
            AllocatedStream->m_releasing = true;
            ALOGD("START stream thread 1 release %d", __LINE__);
            do {
                AllocatedStream->release();
                usleep(SIG_WAITING_TICK);
            } while (AllocatedStream->m_releasing);
            ALOGD("END   stream thread 1 release %d", __LINE__);

            *stream_id = STREAM_ID_ZSL;

            m_streamThreadInitialize((SignalDrivenThread*)AllocatedStream);

            *format_actual = HAL_PIXEL_FORMAT_YCbCr_422_I; // YUYV
            *usage = 0;
            *max_buffers = 6;

            newParameters.width                 = width;
            newParameters.height                = height;
            newParameters.format                = *format_actual;
            newParameters.streamOps             = stream_ops;
            newParameters.usage                 = *usage;
            newParameters.numHwBuffers          = NUM_SCC_BUFFERS;
            newParameters.numOwnSvcBuffers      = *max_buffers;
            newParameters.planes                = NUM_PLANES(*format_actual);
            newParameters.metaPlanes            = 1;

            newParameters.numSvcBufsInHal       = 0;
            newParameters.minUndequedBuffer     = 2;
            newParameters.needsIonMap           = false;

            newParameters.node                  = &m_camera_info.capture;
            newParameters.node->type            = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            newParameters.node->memory          = V4L2_MEMORY_DMABUF;

            AllocatedStream->streamType         = STREAM_TYPE_DIRECT;
            AllocatedStream->m_index            = 1;
            AllocatedStream->setParameter(&newParameters);
            AllocatedStream->m_activated = true;
            AllocatedStream->m_numRegisteredStream = 1;
            if (bJpegExists) {
                AllocatedStream->attachSubStream(STREAM_ID_JPEG, 10);
            }
            ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, AllocatedStream->m_numRegisteredStream);
            return 0;

        }
    }
    else if (format == HAL_PIXEL_FORMAT_BLOB
            && m_camera2->isSupportedJpegResolution(width, height)) {
        *stream_id = STREAM_ID_JPEG;

        subParameters = &m_subStreams[*stream_id];
        memset(subParameters, 0, sizeof(substream_parameters_t));

        if (!(m_streamThreads[1].get())) {
            ALOGV("DEBUG(%s): stream thread 1 not exist", __FUNCTION__);
            StartSCCThread(false);
        }
        else if (m_streamThreads[1]->m_activated ==  false) {
            ALOGV("DEBUG(%s): stream thread 1 suspended. restarting", __FUNCTION__);
            StartSCCThread(true);
        }
        parentStream = (StreamThread*)(m_streamThreads[1].get());

        *format_actual = HAL_PIXEL_FORMAT_BLOB;
        *usage = GRALLOC_USAGE_SW_READ_OFTEN | GRALLOC_USAGE_SW_WRITE_OFTEN;
        *max_buffers = 4;

        subParameters->type          = SUBSTREAM_TYPE_JPEG;
        subParameters->width         = width;
        subParameters->height        = height;
        subParameters->format        = *format_actual;
        subParameters->svcPlanes     = 1;
        subParameters->streamOps     = stream_ops;
        subParameters->usage         = *usage;
        subParameters->numOwnSvcBuffers = *max_buffers;
        subParameters->numSvcBufsInHal  = 0;
        subParameters->needBufferInit    = false;
        subParameters->minUndequedBuffer = 2;

        res = parentStream->attachSubStream(STREAM_ID_JPEG, 10);
        if (res != NO_ERROR) {
            ALOGE("(%s): substream attach failed. res(%d)", __FUNCTION__, res);
            return 1;
        }
        ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, parentStream->m_numRegisteredStream);
        ALOGV("(%s): Enabling Jpeg", __FUNCTION__);
        return 0;
    }
    else if (format == HAL_PIXEL_FORMAT_YCrCb_420_SP || format == HAL_PIXEL_FORMAT_YV12) {
        *stream_id = STREAM_ID_PRVCB;

        subParameters = &m_subStreams[STREAM_ID_PRVCB];
        memset(subParameters, 0, sizeof(substream_parameters_t));

        parentStream = (StreamThread*)(m_streamThreads[0].get());
        if (!parentStream) {
            return 1;
        }

        *format_actual = format;
        *usage = GRALLOC_USAGE_SW_READ_OFTEN | GRALLOC_USAGE_SW_WRITE_OFTEN;
        *max_buffers = 6;

        subParameters->type         = SUBSTREAM_TYPE_PRVCB;
        subParameters->width        = width;
        subParameters->height       = height;
        subParameters->format       = *format_actual;
        subParameters->svcPlanes     = NUM_PLANES(*format_actual);
        subParameters->streamOps     = stream_ops;
        subParameters->usage         = *usage;
        subParameters->numOwnSvcBuffers = *max_buffers;
        subParameters->numSvcBufsInHal  = 0;
        subParameters->needBufferInit    = false;
        subParameters->minUndequedBuffer = 2;

        if (format == HAL_PIXEL_FORMAT_YCrCb_420_SP) {
            subParameters->internalFormat = HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP;
            subParameters->internalPlanes = NUM_PLANES(HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP);
        }
        else {
            subParameters->internalFormat = HAL_PIXEL_FORMAT_EXYNOS_YV12;
            subParameters->internalPlanes = NUM_PLANES(HAL_PIXEL_FORMAT_EXYNOS_YV12);
        }

        res = parentStream->attachSubStream(STREAM_ID_PRVCB, 20);
        if (res != NO_ERROR) {
            ALOGE("(%s): substream attach failed. res(%d)", __FUNCTION__, res);
            return 1;
        }
        ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, parentStream->m_numRegisteredStream);
        ALOGV("(%s): Enabling previewcb", __FUNCTION__);
        return 0;
    }
    ALOGE("(%s): Unsupported Pixel Format", __FUNCTION__);
    return 1;
}

int ExynosCameraHWInterface2::registerStreamBuffers(uint32_t stream_id,
        int num_buffers, buffer_handle_t *registeringBuffers)
{
    int                     i,j;
    void                    *virtAddr[3];
    int                     plane_index = 0;
    StreamThread *          targetStream;
    stream_parameters_t     *targetStreamParms;
    node_info_t             *currentNode;

    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane  planes[VIDEO_MAX_PLANES];

    ALOGD("(%s): stream_id(%d), num_buff(%d), handle(%x) ", __FUNCTION__,
        stream_id, num_buffers, (uint32_t)registeringBuffers);

    if (stream_id == STREAM_ID_PREVIEW && m_streamThreads[0].get()) {
        targetStream = m_streamThreads[0].get();
        targetStreamParms = &(m_streamThreads[0]->m_parameters);

    }
    else if (stream_id == STREAM_ID_JPEG || stream_id == STREAM_ID_RECORD || stream_id == STREAM_ID_PRVCB) {
        substream_parameters_t  *targetParms;
        targetParms = &m_subStreams[stream_id];

        targetParms->numSvcBuffers = num_buffers;

        for (i = 0 ; i < targetParms->numSvcBuffers ; i++) {
            ALOGV("(%s): registering substream(%d) Buffers[%d] (%x) ", __FUNCTION__,
                i, stream_id, (uint32_t)(registeringBuffers[i]));
            if (m_grallocHal) {
                if (m_grallocHal->lock(m_grallocHal, registeringBuffers[i],
                       targetParms->usage, 0, 0,
                       targetParms->width, targetParms->height, virtAddr) != 0) {
                    ALOGE("ERR(%s): could not obtain gralloc buffer", __FUNCTION__);
                }
                else {
                    ExynosBuffer currentBuf;
                    const private_handle_t *priv_handle = reinterpret_cast<const private_handle_t *>(registeringBuffers[i]);
                    if (targetParms->svcPlanes == 1) {
                        currentBuf.fd.extFd[0] = priv_handle->fd;
                        currentBuf.size.extS[0] = priv_handle->uiAllocSize[0];
                        currentBuf.size.extS[1] = 0;
                        currentBuf.size.extS[2] = 0;
                    } else if (targetParms->svcPlanes == 2) {
                        currentBuf.fd.extFd[0] = priv_handle->fd;
                        currentBuf.fd.extFd[1] = priv_handle->fd1;

                    } else if (targetParms->svcPlanes == 3) {
                        currentBuf.fd.extFd[0] = priv_handle->fd;
                        currentBuf.fd.extFd[1] = priv_handle->fd1;
                        currentBuf.fd.extFd[2] = priv_handle->fd2;
                    }
                    for (plane_index = 0 ; plane_index < targetParms->svcPlanes ; plane_index++) {
                        currentBuf.virt.extP[plane_index] = (char *)virtAddr[plane_index];
                        ALOGV("DEBUG(%s): plane(%d): fd(%d) addr(%x) size(%d)",
                             __FUNCTION__, plane_index, currentBuf.fd.extFd[plane_index],
                             (unsigned int)currentBuf.virt.extP[plane_index], currentBuf.size.extS[plane_index]);
                    }
                    targetParms->svcBufStatus[i]  = ON_SERVICE;
                    targetParms->svcBuffers[i]    = currentBuf;
                    targetParms->svcBufHandle[i]  = registeringBuffers[i];
                }
            }
        }
        targetParms->needBufferInit = true;
        return 0;
    }
    else if (stream_id == STREAM_ID_ZSL && m_streamThreads[1].get()) {
        targetStream = m_streamThreads[1].get();
        targetStreamParms = &(m_streamThreads[1]->m_parameters);
    }
    else {
        ALOGE("(%s): unregistered stream id (%d)", __FUNCTION__, stream_id);
        return 1;
    }

    if (targetStream->streamType == STREAM_TYPE_DIRECT) {
        if (num_buffers < targetStreamParms->numHwBuffers) {
            ALOGE("ERR(%s) registering insufficient num of buffers (%d) < (%d)",
                __FUNCTION__, num_buffers, targetStreamParms->numHwBuffers);
            return 1;
        }
    }
    ALOGV("DEBUG(%s): format(%x) width(%d), height(%d) svcPlanes(%d)",
            __FUNCTION__, targetStreamParms->format, targetStreamParms->width,
            targetStreamParms->height, targetStreamParms->planes);
    targetStreamParms->numSvcBuffers = num_buffers;
    currentNode = targetStreamParms->node;
    currentNode->width      = targetStreamParms->width;
    currentNode->height     = targetStreamParms->height;
    currentNode->format     = HAL_PIXEL_FORMAT_2_V4L2_PIX(targetStreamParms->format);
    currentNode->planes     = targetStreamParms->planes;
    currentNode->buffers    = targetStreamParms->numHwBuffers;
    cam_int_s_input(currentNode, m_camera_info.sensor_id);
    cam_int_s_fmt(currentNode);
    cam_int_reqbufs(currentNode);
    for (i = 0 ; i < targetStreamParms->numSvcBuffers ; i++) {
        ALOGV("DEBUG(%s): registering Stream Buffers[%d] (%x) ", __FUNCTION__,
            i, (uint32_t)(registeringBuffers[i]));
                v4l2_buf.m.planes   = planes;
                v4l2_buf.type       = currentNode->type;
                v4l2_buf.memory     = currentNode->memory;
                v4l2_buf.index      = i;
                v4l2_buf.length     = currentNode->planes;

                ExynosBuffer currentBuf;
                ExynosBuffer metaBuf;
                const private_handle_t *priv_handle = reinterpret_cast<const private_handle_t *>(registeringBuffers[i]);

                m_getAlignedYUVSize(currentNode->format,
                    currentNode->width, currentNode->height, &currentBuf);

                ALOGV("DEBUG(%s):  ion_size(%d), stride(%d), ", __FUNCTION__, priv_handle->size, priv_handle->stride);
                if (currentNode->planes == 1) {
                    v4l2_buf.m.planes[0].m.fd = priv_handle->fd;
                    currentBuf.fd.extFd[0] = priv_handle->fd;
                    currentBuf.size.extS[0] = priv_handle->uiAllocSize[0];
                    currentBuf.size.extS[1] = 0;
                    currentBuf.size.extS[2] = 0;
                } else if (currentNode->planes == 2) {
                    v4l2_buf.m.planes[0].m.fd = priv_handle->fd;
                    v4l2_buf.m.planes[1].m.fd = priv_handle->fd1;
                    currentBuf.fd.extFd[0] = priv_handle->fd;
                    currentBuf.fd.extFd[1] = priv_handle->fd1;

                } else if (currentNode->planes == 3) {
                    v4l2_buf.m.planes[0].m.fd = priv_handle->fd;
                    v4l2_buf.m.planes[2].m.fd = priv_handle->fd1;
                    v4l2_buf.m.planes[1].m.fd = priv_handle->fd2;
                    currentBuf.fd.extFd[0] = priv_handle->fd;
                    currentBuf.fd.extFd[2] = priv_handle->fd1;
                    currentBuf.fd.extFd[1] = priv_handle->fd2;
                }

                for (plane_index = 0 ; plane_index < (int)v4l2_buf.length ; plane_index++) {
                    if (targetStreamParms->needsIonMap)
                        currentBuf.virt.extP[plane_index] = (char *)ion_map(currentBuf.fd.extFd[plane_index], currentBuf.size.extS[plane_index], 0);
                    v4l2_buf.m.planes[plane_index].length  = currentBuf.size.extS[plane_index];
                    ALOGV("(%s): MAPPING plane(%d): fd(%d) addr(%x), length(%d)",
                         __FUNCTION__, plane_index, v4l2_buf.m.planes[plane_index].m.fd,
                         (unsigned int)currentBuf.virt.extP[plane_index],
                         v4l2_buf.m.planes[plane_index].length);
                }

                if (i < currentNode->buffers) {

                    /* add plane for metadata*/
                    metaBuf.size.extS[0] = 4*1024;
                    allocCameraMemory(m_ionCameraClient , &metaBuf, 1, 1<<0);

                    v4l2_buf.length += targetStreamParms->metaPlanes;
                    v4l2_buf.m.planes[v4l2_buf.length-1].m.fd = metaBuf.fd.extFd[0];
                    v4l2_buf.m.planes[v4l2_buf.length-1].length = metaBuf.size.extS[0];

                    ALOGV("Qbuf metaBuf: fd(%d), length(%d) plane(%d)", metaBuf.fd.extFd[0], metaBuf.size.extS[0], v4l2_buf.length);

                    if (exynos_v4l2_qbuf(currentNode->fd, &v4l2_buf) < 0) {
                        ALOGE("ERR(%s): stream id(%d) exynos_v4l2_qbuf() fail fd(%d)",
                            __FUNCTION__, stream_id, currentNode->fd);
                    }
                    ALOGV("DEBUG(%s): stream id(%d) exynos_v4l2_qbuf() success fd(%d)",
                            __FUNCTION__, stream_id, currentNode->fd);
                    targetStreamParms->svcBufStatus[i]  = REQUIRES_DQ_FROM_SVC;
                }
                else {
                    targetStreamParms->svcBufStatus[i]  = ON_SERVICE;
                }

                targetStreamParms->svcBuffers[i]       = currentBuf;
                targetStreamParms->metaBuffers[i] = metaBuf;
                targetStreamParms->svcBufHandle[i]     = registeringBuffers[i];
            }

    ALOGV("DEBUG(%s): calling  streamon stream id = %d", __FUNCTION__, stream_id);
    cam_int_streamon(targetStreamParms->node);
    ALOGV("DEBUG(%s): calling  streamon END", __FUNCTION__);
    currentNode->status = true;
    ALOGV("DEBUG(%s): END registerStreamBuffers", __FUNCTION__);

    return 0;
}

int ExynosCameraHWInterface2::releaseStream(uint32_t stream_id)
{
    StreamThread *targetStream;
    status_t res = NO_ERROR;
    ALOGD("(%s): stream_id(%d)", __FUNCTION__, stream_id);
    bool releasingScpMain = false;

    if (stream_id == STREAM_ID_PREVIEW) {
        targetStream = (StreamThread*)(m_streamThreads[0].get());
        if (!targetStream) {
            ALOGW("(%s): Stream Not Exists", __FUNCTION__);
            return NO_ERROR;
        }
        targetStream->m_numRegisteredStream--;
        ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, targetStream->m_numRegisteredStream);
        releasingScpMain = true;
        if (targetStream->m_parameters.needsIonMap) {
            for (int i = 0; i < targetStream->m_parameters.numSvcBuffers; i++) {
                for (int j = 0; j < targetStream->m_parameters.planes; j++) {
                    ion_unmap(targetStream->m_parameters.svcBuffers[i].virt.extP[j],
                                    targetStream->m_parameters.svcBuffers[i].size.extS[j]);
                    ALOGV("(%s) ummap stream buffer[%d], plane(%d), fd %d vaddr %x", __FUNCTION__, i, j,
                                  targetStream->m_parameters.svcBuffers[i].fd.extFd[j], (unsigned int)(targetStream->m_parameters.svcBuffers[i].virt.extP[j]));
                }
            }
        }
    } else if (stream_id == STREAM_ID_JPEG) {
        if (m_resizeBuf.size.s != 0) {
            freeCameraMemory(&m_resizeBuf, 1);
        }
        memset(&m_subStreams[stream_id], 0, sizeof(substream_parameters_t));

        targetStream = (StreamThread*)(m_streamThreads[1].get());
        if (!targetStream) {
            ALOGW("(%s): Stream Not Exists", __FUNCTION__);
            return NO_ERROR;
        }

        if (targetStream->detachSubStream(stream_id) != NO_ERROR) {
            ALOGE("(%s): substream detach failed. res(%d)", __FUNCTION__, res);
            return 1;
        }
        ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, targetStream->m_numRegisteredStream);
        return 0;
    } else if (stream_id == STREAM_ID_RECORD) {
        memset(&m_subStreams[stream_id], 0, sizeof(substream_parameters_t));

        targetStream = (StreamThread*)(m_streamThreads[0].get());
        if (!targetStream) {
            ALOGW("(%s): Stream Not Exists", __FUNCTION__);
            return NO_ERROR;
        }

        if (targetStream->detachSubStream(stream_id) != NO_ERROR) {
            ALOGE("(%s): substream detach failed. res(%d)", __FUNCTION__, res);
            return 1;
        }

        if (targetStream->m_numRegisteredStream != 0)
            return 0;
    } else if (stream_id == STREAM_ID_PRVCB) {
        if (m_previewCbBuf.size.s != 0) {
            freeCameraMemory(&m_previewCbBuf, m_subStreams[stream_id].internalPlanes);
        }
        memset(&m_subStreams[stream_id], 0, sizeof(substream_parameters_t));

        targetStream = (StreamThread*)(m_streamThreads[0].get());
        if (!targetStream) {
            ALOGW("(%s): Stream Not Exists", __FUNCTION__);
            return NO_ERROR;
        }

        if (targetStream->detachSubStream(stream_id) != NO_ERROR) {
            ALOGE("(%s): substream detach failed. res(%d)", __FUNCTION__, res);
            return 1;
        }

        if (targetStream->m_numRegisteredStream != 0)
            return 0;
    } else if (stream_id == STREAM_ID_ZSL) {
        targetStream = (StreamThread*)(m_streamThreads[1].get());
        if (!targetStream) {
            ALOGW("(%s): Stream Not Exists", __FUNCTION__);
            return NO_ERROR;
        }

        targetStream->m_numRegisteredStream--;
        ALOGV("(%s): m_numRegisteredStream = %d", __FUNCTION__, targetStream->m_numRegisteredStream);
        if (targetStream->m_parameters.needsIonMap) {
            for (int i = 0; i < targetStream->m_parameters.numSvcBuffers; i++) {
                for (int j = 0; j < targetStream->m_parameters.planes; j++) {
                    ion_unmap(targetStream->m_parameters.svcBuffers[i].virt.extP[j],
                                    targetStream->m_parameters.svcBuffers[i].size.extS[j]);
                    ALOGV("(%s) ummap stream buffer[%d], plane(%d), fd %d vaddr %x", __FUNCTION__, i, j,
                                  targetStream->m_parameters.svcBuffers[i].fd.extFd[j], (unsigned int)(targetStream->m_parameters.svcBuffers[i].virt.extP[j]));
                }
            }
        }
    } else {
        ALOGE("ERR:(%s): wrong stream id (%d)", __FUNCTION__, stream_id);
        return 1;
    }

    if (m_sensorThread != NULL && releasingScpMain) {
        m_sensorThread->release();
        ALOGD("(%s): START Waiting for (indirect) sensor thread termination", __FUNCTION__);
        while (!m_sensorThread->IsTerminated())
            usleep(SIG_WAITING_TICK);
        ALOGD("(%s): END   Waiting for (indirect) sensor thread termination", __FUNCTION__);
    }

    if (m_streamThreads[1]->m_numRegisteredStream == 0 && m_streamThreads[1]->m_activated) {
        ALOGV("(%s): deactivating stream thread 1 ", __FUNCTION__);
        targetStream = (StreamThread*)(m_streamThreads[1].get());
        targetStream->m_releasing = true;
        ALOGD("START stream thread release %d", __LINE__);
        do {
            targetStream->release();
            usleep(SIG_WAITING_TICK);
        } while (targetStream->m_releasing);
        m_camera_info.capture.status = false;
        ALOGD("END   stream thread release %d", __LINE__);
    }

    if (releasingScpMain || (m_streamThreads[0].get() != NULL && m_streamThreads[0]->m_numRegisteredStream == 0 && m_streamThreads[0]->m_activated)) {
        ALOGV("(%s): deactivating stream thread 0", __FUNCTION__);
        targetStream = (StreamThread*)(m_streamThreads[0].get());
        targetStream->m_releasing = true;
        ALOGD("(%s): START Waiting for (indirect) stream thread release - line(%d)", __FUNCTION__, __LINE__);
        do {
            targetStream->release();
            usleep(SIG_WAITING_TICK);
        } while (targetStream->m_releasing);
        ALOGD("(%s): END   Waiting for (indirect) stream thread release - line(%d)", __FUNCTION__, __LINE__);
        targetStream->SetSignal(SIGNAL_THREAD_TERMINATE);

        if (targetStream != NULL) {
            ALOGD("(%s): START Waiting for (indirect) stream thread termination", __FUNCTION__);
            while (!targetStream->IsTerminated())
                usleep(SIG_WAITING_TICK);
            ALOGD("(%s): END   Waiting for (indirect) stream thread termination", __FUNCTION__);
            m_streamThreads[0] = NULL;
        }
        if (m_camera_info.capture.status == true) {
            m_scpForceSuspended = true;
        }
        m_isIspStarted = false;
    }
    ALOGV("(%s): END", __FUNCTION__);
    return 0;
}

int ExynosCameraHWInterface2::allocateReprocessStream(
    uint32_t width, uint32_t height, uint32_t format,
    const camera2_stream_in_ops_t *reprocess_stream_ops,
    uint32_t *stream_id, uint32_t *consumer_usage, uint32_t *max_buffers)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return 0;
}

int ExynosCameraHWInterface2::allocateReprocessStreamFromStream(
            uint32_t output_stream_id,
            const camera2_stream_in_ops_t *reprocess_stream_ops,
            // outputs
            uint32_t *stream_id)
{
    ALOGD("(%s): output_stream_id(%d)", __FUNCTION__, output_stream_id);
    *stream_id = STREAM_ID_JPEG_REPROCESS;

    m_reprocessStreamId = *stream_id;
    m_reprocessOps = reprocess_stream_ops;
    m_reprocessOutputStreamId = output_stream_id;
    return 0;
}

int ExynosCameraHWInterface2::releaseReprocessStream(uint32_t stream_id)
{
    ALOGD("(%s): stream_id(%d)", __FUNCTION__, stream_id);
    if (stream_id == STREAM_ID_JPEG_REPROCESS) {
        m_reprocessStreamId = 0;
        m_reprocessOps = NULL;
        m_reprocessOutputStreamId = 0;
        return 0;
    }
    return 1;
}

int ExynosCameraHWInterface2::triggerAction(uint32_t trigger_id, int ext1, int ext2)
{
    Mutex::Autolock lock(m_TriggerLock);
    ALOGV("DEBUG(%s): id(%x), %d, %d", __FUNCTION__, trigger_id, ext1, ext2);

    switch (trigger_id) {
    case CAMERA2_TRIGGER_AUTOFOCUS:
        ALOGV("DEBUG(%s):TRIGGER_AUTOFOCUS id(%d)", __FUNCTION__, ext1);
        OnAfTrigger(ext1);
        break;

    case CAMERA2_TRIGGER_CANCEL_AUTOFOCUS:
        ALOGV("DEBUG(%s):CANCEL_AUTOFOCUS id(%d)", __FUNCTION__, ext1);
        OnAfCancel(ext1);
        break;
    case CAMERA2_TRIGGER_PRECAPTURE_METERING:
        ALOGV("DEBUG(%s):CAMERA2_TRIGGER_PRECAPTURE_METERING id(%d)", __FUNCTION__, ext1);
        OnPrecaptureMeteringTriggerStart(ext1);
        break;
    default:
        break;
    }
    return 0;
}

int ExynosCameraHWInterface2::setNotifyCallback(camera2_notify_callback notify_cb, void *user)
{
    ALOGV("DEBUG(%s): cb_addr(%x)", __FUNCTION__, (unsigned int)notify_cb);
    m_notifyCb = notify_cb;
    m_callbackCookie = user;
    return 0;
}

int ExynosCameraHWInterface2::getMetadataVendorTagOps(vendor_tag_query_ops_t **ops)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return 0;
}

int ExynosCameraHWInterface2::dump(int fd)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return 0;
}

void ExynosCameraHWInterface2::m_getAlignedYUVSize(int colorFormat, int w, int h, ExynosBuffer *buf)
{
    switch (colorFormat) {
    // 1p
    case V4L2_PIX_FMT_RGB565 :
    case V4L2_PIX_FMT_YUYV :
    case V4L2_PIX_FMT_UYVY :
    case V4L2_PIX_FMT_VYUY :
    case V4L2_PIX_FMT_YVYU :
        buf->size.extS[0] = FRAME_SIZE(V4L2_PIX_2_HAL_PIXEL_FORMAT(colorFormat), w, h);
        buf->size.extS[1] = 0;
        buf->size.extS[2] = 0;
        break;
    // 2p
    case V4L2_PIX_FMT_NV12 :
    case V4L2_PIX_FMT_NV12T :
    case V4L2_PIX_FMT_NV21 :
        buf->size.extS[0] = ALIGN(w,   16) * ALIGN(h,   16);
        buf->size.extS[1] = ALIGN(w/2, 16) * ALIGN(h/2, 16);
        buf->size.extS[2] = 0;
        break;
    case V4L2_PIX_FMT_NV12M :
    case V4L2_PIX_FMT_NV12MT_16X16 :
    case V4L2_PIX_FMT_NV21M:
        buf->size.extS[0] = ALIGN(w, 16) * ALIGN(h,     16);
        buf->size.extS[1] = ALIGN(buf->size.extS[0] / 2, 256);
        buf->size.extS[2] = 0;
        break;
    case V4L2_PIX_FMT_NV16 :
    case V4L2_PIX_FMT_NV61 :
        buf->size.extS[0] = ALIGN(w, 16) * ALIGN(h, 16);
        buf->size.extS[1] = ALIGN(w, 16) * ALIGN(h,  16);
        buf->size.extS[2] = 0;
        break;
     // 3p
    case V4L2_PIX_FMT_YUV420 :
    case V4L2_PIX_FMT_YVU420 :
        buf->size.extS[0] = (w * h);
        buf->size.extS[1] = (w * h) >> 2;
        buf->size.extS[2] = (w * h) >> 2;
        break;
    case V4L2_PIX_FMT_YUV420M:
    case V4L2_PIX_FMT_YVU420M :
        buf->size.extS[0] = ALIGN(w,  32) * ALIGN(h,  16);
        buf->size.extS[1] = ALIGN(w/2, 16) * ALIGN(h/2, 8);
        buf->size.extS[2] = ALIGN(w/2, 16) * ALIGN(h/2, 8);
        break;
    case V4L2_PIX_FMT_YUV422P :
        buf->size.extS[0] = ALIGN(w,  16) * ALIGN(h,  16);
        buf->size.extS[1] = ALIGN(w/2, 16) * ALIGN(h/2, 8);
        buf->size.extS[2] = ALIGN(w/2, 16) * ALIGN(h/2, 8);
        break;
    default:
        ALOGE("ERR(%s):unmatched colorFormat(%d)", __FUNCTION__, colorFormat);
        return;
        break;
    }
}

bool ExynosCameraHWInterface2::m_getRatioSize(int  src_w,  int   src_h,
                                             int  dst_w,  int   dst_h,
                                             int *crop_x, int *crop_y,
                                             int *crop_w, int *crop_h,
                                             int zoom)
{
    *crop_w = src_w;
    *crop_h = src_h;

    if (   src_w != dst_w
        || src_h != dst_h) {
        float src_ratio = 1.0f;
        float dst_ratio = 1.0f;

        // ex : 1024 / 768
        src_ratio = (float)src_w / (float)src_h;

        // ex : 352  / 288
        dst_ratio = (float)dst_w / (float)dst_h;

        if (dst_w * dst_h < src_w * src_h) {
            if (dst_ratio <= src_ratio) {
                // shrink w
                *crop_w = src_h * dst_ratio;
                *crop_h = src_h;
            } else {
                // shrink h
                *crop_w = src_w;
                *crop_h = src_w / dst_ratio;
            }
        } else {
            if (dst_ratio <= src_ratio) {
                // shrink w
                *crop_w = src_h * dst_ratio;
                *crop_h = src_h;
            } else {
                // shrink h
                *crop_w = src_w;
                *crop_h = src_w / dst_ratio;
            }
        }
    }

    if (zoom != 0) {
        float zoomLevel = ((float)zoom + 10.0) / 10.0;
        *crop_w = (int)((float)*crop_w / zoomLevel);
        *crop_h = (int)((float)*crop_h / zoomLevel);
    }

    #define CAMERA_CROP_WIDTH_RESTRAIN_NUM  (0x2)
    int w_align = (*crop_w & (CAMERA_CROP_WIDTH_RESTRAIN_NUM - 1));
    if (w_align != 0) {
        if (  (CAMERA_CROP_WIDTH_RESTRAIN_NUM >> 1) <= w_align
            && *crop_w + (CAMERA_CROP_WIDTH_RESTRAIN_NUM - w_align) <= dst_w) {
            *crop_w += (CAMERA_CROP_WIDTH_RESTRAIN_NUM - w_align);
        }
        else
            *crop_w -= w_align;
    }

    #define CAMERA_CROP_HEIGHT_RESTRAIN_NUM  (0x2)
    int h_align = (*crop_h & (CAMERA_CROP_HEIGHT_RESTRAIN_NUM - 1));
    if (h_align != 0) {
        if (  (CAMERA_CROP_HEIGHT_RESTRAIN_NUM >> 1) <= h_align
            && *crop_h + (CAMERA_CROP_HEIGHT_RESTRAIN_NUM - h_align) <= dst_h) {
            *crop_h += (CAMERA_CROP_HEIGHT_RESTRAIN_NUM - h_align);
        }
        else
            *crop_h -= h_align;
    }

    *crop_x = (src_w - *crop_w) >> 1;
    *crop_y = (src_h - *crop_h) >> 1;

    if (*crop_x & (CAMERA_CROP_WIDTH_RESTRAIN_NUM >> 1))
        *crop_x -= 1;

    if (*crop_y & (CAMERA_CROP_HEIGHT_RESTRAIN_NUM >> 1))
        *crop_y -= 1;

    return true;
}

void ExynosCameraHWInterface2::m_mainThreadFunc(SignalDrivenThread * self)
{
    camera_metadata_t *currentFrame = NULL;
    size_t numEntries = 0;
    size_t frameSize = 0;
    camera_metadata_t * preparedFrame = NULL;
    camera_metadata_t *deregisteredRequest = NULL;
    uint32_t currentSignal = self->GetProcessingSignal();
    MainThread *  selfThread      = ((MainThread*)self);
    int res = 0;

    int ret;

    ALOGV("DEBUG(%s): m_mainThreadFunc (%x)", __FUNCTION__, currentSignal);

    if (currentSignal & SIGNAL_THREAD_RELEASE) {
        ALOGV("DEBUG(%s): processing SIGNAL_THREAD_RELEASE", __FUNCTION__);

        ALOGV("DEBUG(%s): processing SIGNAL_THREAD_RELEASE DONE", __FUNCTION__);
        selfThread->SetSignal(SIGNAL_THREAD_TERMINATE);
        return;
    }

    if (currentSignal & SIGNAL_MAIN_STREAM_OUTPUT_DONE) {
        ALOGV("DEBUG(%s): MainThread processing SIGNAL_MAIN_STREAM_OUTPUT_DONE", __FUNCTION__);
        /*while (1)*/ {
            ret = m_requestManager->PrepareFrame(&numEntries, &frameSize, &preparedFrame, GetAfStateForService());
            if (ret == false)
                ALOGE("ERR(%s): PrepareFrame ret = %d", __FUNCTION__, ret);

            m_requestManager->DeregisterRequest(&deregisteredRequest);

            ret = m_requestQueueOps->free_request(m_requestQueueOps, deregisteredRequest);
            if (ret < 0)
                ALOGE("ERR(%s): free_request ret = %d", __FUNCTION__, ret);

            ret = m_frameQueueOps->dequeue_frame(m_frameQueueOps, numEntries, frameSize, &currentFrame);
            if (ret < 0)
                ALOGE("ERR(%s): dequeue_frame ret = %d", __FUNCTION__, ret);

            if (currentFrame==NULL) {
                ALOGV("DBG(%s): frame dequeue returned NULL",__FUNCTION__ );
            }
            else {
                ALOGV("DEBUG(%s): frame dequeue done. numEntries(%d) frameSize(%d)",__FUNCTION__ , numEntries, frameSize);
            }
            res = append_camera_metadata(currentFrame, preparedFrame);
            if (res==0) {
                ALOGV("DEBUG(%s): frame metadata append success",__FUNCTION__);
                m_frameQueueOps->enqueue_frame(m_frameQueueOps, currentFrame);
            }
            else {
                ALOGE("ERR(%s): frame metadata append fail (%d)",__FUNCTION__, res);
            }
        }
    }
    ALOGV("DEBUG(%s): MainThread Exit", __FUNCTION__);
    return;
}

void ExynosCameraHWInterface2::DumpInfoWithShot(struct camera2_shot_ext * shot_ext)
{
    ALOGD("####  common Section");
    ALOGD("####                 magic(%x) ",
        shot_ext->shot.magicNumber);
    ALOGD("####  ctl Section");
    ALOGD("####     meta(%d) aper(%f) exp(%lld) duration(%lld) ISO(%d) AWB(%d)",
        shot_ext->shot.ctl.request.metadataMode,
        shot_ext->shot.ctl.lens.aperture,
        shot_ext->shot.ctl.sensor.exposureTime,
        shot_ext->shot.ctl.sensor.frameDuration,
        shot_ext->shot.ctl.sensor.sensitivity,
        shot_ext->shot.ctl.aa.awbMode);

    ALOGD("####                 OutputStream Sensor(%d) SCP(%d) SCC(%d) streams(%x)",
        shot_ext->request_sensor, shot_ext->request_scp, shot_ext->request_scc,
        shot_ext->shot.ctl.request.outputStreams[0]);

    ALOGD("####  DM Section");
    ALOGD("####     meta(%d) aper(%f) exp(%lld) duration(%lld) ISO(%d) timestamp(%lld) AWB(%d) cnt(%d)",
        shot_ext->shot.dm.request.metadataMode,
        shot_ext->shot.dm.lens.aperture,
        shot_ext->shot.dm.sensor.exposureTime,
        shot_ext->shot.dm.sensor.frameDuration,
        shot_ext->shot.dm.sensor.sensitivity,
        shot_ext->shot.dm.sensor.timeStamp,
        shot_ext->shot.dm.aa.awbMode,
        shot_ext->shot.dm.request.frameCount );
}

void ExynosCameraHWInterface2::m_setShotAFRegion(struct camera2_shot_ext *shot_ext)
{
    shot_ext->shot.ctl.aa.afRegions[0] = m_currentAfRegion[0];
    shot_ext->shot.ctl.aa.afRegions[1] = m_currentAfRegion[1];
    shot_ext->shot.ctl.aa.afRegions[2] = m_currentAfRegion[2];
    shot_ext->shot.ctl.aa.afRegions[3] = m_currentAfRegion[3];

    if (shot_ext->shot.ctl.aa.afMode > 0) {
        AF_LOG("(%s), afMode (%d), region = (%d, %d, %d, %d)", __FUNCTION__, shot_ext->shot.ctl.aa.afMode,
            m_currentAfRegion[0], m_currentAfRegion[1], m_currentAfRegion[2], m_currentAfRegion[3]);
    }
}

void ExynosCameraHWInterface2::m_setShotZoom(struct camera2_shot_ext *shot_ext)
{
    float zoomLeft, zoomTop, zoomWidth, zoomHeight;
    int crop_x = 0, crop_y = 0, crop_w = 0, crop_h = 0;

    m_zoomRatio = (float)m_camera2->getSensorW() / (float)shot_ext->shot.ctl.scaler.cropRegion[2];

    m_getRatioSize(m_camera2->getSensorW(), m_camera2->getSensorH(),
                   m_streamThreads[0]->m_parameters.width, m_streamThreads[0]->m_parameters.height,
                   &crop_x, &crop_y,
                   &crop_w, &crop_h,
                   0);

    if (m_streamThreads[0]->m_parameters.width >= m_streamThreads[0]->m_parameters.height) {
        zoomWidth = m_camera2->getSensorW() / m_zoomRatio;
        zoomHeight = zoomWidth *
                m_streamThreads[0]->m_parameters.height / m_streamThreads[0]->m_parameters.width;
    } else {
        zoomHeight = m_camera2->getSensorH() / m_zoomRatio;
        zoomWidth = zoomHeight *
                m_streamThreads[0]->m_parameters.width / m_streamThreads[0]->m_parameters.height;
    }
    zoomLeft = (crop_w - zoomWidth) / 2;
    zoomTop = (crop_h - zoomHeight) / 2;

    int32_t new_cropRegion[3] = { zoomLeft, zoomTop, zoomWidth };

    int cropCompensation = (new_cropRegion[0] * 2 + new_cropRegion[2]) - ALIGN(crop_w, 4);
    if (cropCompensation)
        new_cropRegion[2] -= cropCompensation;

    shot_ext->shot.ctl.scaler.cropRegion[0] = new_cropRegion[0];
    shot_ext->shot.ctl.scaler.cropRegion[1] = new_cropRegion[1];
    shot_ext->shot.ctl.scaler.cropRegion[2] = new_cropRegion[2];
}

void ExynosCameraHWInterface2::m_setShotAFMode(struct camera2_shot_ext *shot_ext)
{
    Mutex::Autolock lock(m_TriggerLock);

    /* Default off */
    shot_ext->shot.ctl.aa.afTrigger = 0;
    shot_ext->shot.ctl.aa.afMode = m_afMode;

    if (AF_MODE_CAF(m_afMode)) {
        if ((m_ctlInfo.scene.prevSceneMode == AA_SCENE_MODE_UNSUPPORTED) ||
           (m_ctlInfo.scene.prevSceneMode == AA_SCENE_MODE_FACE_PRIORITY))
            if(m_afMode == AA_AFMODE_CONTINUOUS_PICTURE)
                /* Only at CAF picture mode, face caf can be applied */
                shot_ext->shot.ctl.aa.afMode = AA_AFMODE_CONTINUOUS_PICTURE_FACE;

        /* reset flash result */
        if (m_ctlInfo.flash.m_afFlashFired) {
            m_ctlInfo.flash.m_flashEnableFlg = false;
            m_ctlInfo.flash.m_afFlashFired = false;
            m_ctlInfo.flash.m_flashDecisionResult = false;
            m_ctlInfo.flash.m_flashState = FLASH_STATE_NONE;
        }

        m_afState = HAL_AFSTATE_STARTED;
        shot_ext->shot.ctl.aa.afTrigger = 1;
    } else if ((m_afMode == AA_AFMODE_MANUAL) &&
                ( shot_ext->shot.ctl.lens.focusDistance == 0)) {
        shot_ext->shot.ctl.aa.afMode = AA_AFMODE_INFINITY;
        shot_ext->shot.ctl.aa.afTrigger = 1;
        m_afState = HAL_AFSTATE_STARTED;
    }

    AF_LOG("(%s): Set AF Mode[%d], State [%s]", __FUNCTION__,
                shot_ext->shot.ctl.aa.afMode, HAL_AFState_Name[m_afState]);

    m_IsAfModeUpdateRequired = false;

    if (m_afMode2 != NO_CHANGE) {
        enum aa_afmode tempAfMode = m_afMode2;
        m_afMode2 = NO_CHANGE;
        SetAfMode(tempAfMode);
    }
}

void ExynosCameraHWInterface2::m_setShotAFTrigger(struct camera2_shot_ext *shot_ext)
{
    Mutex::Autolock lock(m_TriggerLock);

    /* Default off */
    shot_ext->shot.ctl.aa.afTrigger = 0;
    shot_ext->shot.ctl.aa.afMode = NO_CHANGE;

    if(m_IsAfLockRequired) {
        shot_ext->shot.ctl.aa.afMode = AA_AFMODE_OFF;
        m_IsAfLockRequired = false;
        return;
    }

    /* Trigger Auto/Macro/Manual AF */
    if(AF_MODE_NORMAL_MANUAL(m_afMode)) {
        if (m_afState == HAL_AFSTATE_TRIGGERED) {
            if (m_ctlInfo.flash.m_flashEnableFlg && m_ctlInfo.flash.m_afFlashFired) {
                /* flash case */
                if (m_ctlInfo.flash.m_flashState == FLASH_STATE_ON_DONE) {
                    m_afState = HAL_AFSTATE_STARTED;
                    shot_ext->shot.ctl.aa.afMode = m_afMode;
                    shot_ext->shot.ctl.aa.afTrigger = 1;
                }
            } else {
                /* non-flash case */
                m_afState = HAL_AFSTATE_STARTED;
                shot_ext->shot.ctl.aa.afMode = m_afMode;
                shot_ext->shot.ctl.aa.afTrigger = 1;
            }

            /* Set a different touch af scheme at video mode */
            if (m_wideAspect && (m_afMode == AA_AFMODE_AUTO)) {
                shot_ext->shot.ctl.aa.afMode = AA_AFMODE_AUTO_VIDEO;
                AF_LOG("(%s): AF Mode change (%d) -> (%d) at video mode", __FUNCTION__, m_afMode, shot_ext->shot.ctl.aa.afMode);
            }
            AF_LOG("(%s): AF State [%s]", __FUNCTION__, HAL_AFState_Name[m_afState]);
        }
    }
}

void ExynosCameraHWInterface2::m_setShotNightshot(struct camera2_shot_ext *shot_ext,
                                                  int &matchedFrameCnt)
{
    if (shot_ext->shot.ctl.aa.sceneMode == AA_SCENE_MODE_NIGHT
                        && shot_ext->shot.ctl.aa.aeMode == AA_AEMODE_LOCKED)
        /* Unlock ae lock. In this case metering can't be applied */
        /* TODO : consider metering case */
        shot_ext->shot.ctl.aa.aeMode = AA_AEMODE_ON;

    if (m_nightCaptureCnt == 0) {
        if (shot_ext->shot.ctl.aa.captureIntent == AA_CAPTURE_INTENT_STILL_CAPTURE
                && shot_ext->shot.ctl.aa.sceneMode == AA_SCENE_MODE_NIGHT) {
            shot_ext->shot.ctl.aa.sceneMode = AA_SCENE_MODE_NIGHT_CAPTURE;
            shot_ext->shot.ctl.aa.aeTargetFpsRange[0] = 2;
            shot_ext->shot.ctl.aa.aeTargetFpsRange[1] = 30;
            m_nightCaptureCnt = 4;
            m_nightCaptureFrameCnt = matchedFrameCnt;
            shot_ext->request_scc = 0;
        }
    } else if (m_nightCaptureCnt == 1) {
        shot_ext->shot.ctl.aa.sceneMode = AA_SCENE_MODE_NIGHT_CAPTURE;
        shot_ext->shot.ctl.aa.aeTargetFpsRange[0] = 30;
        shot_ext->shot.ctl.aa.aeTargetFpsRange[1] = 30;
        m_nightCaptureCnt--;
        m_nightCaptureFrameCnt = 0;
        shot_ext->request_scc = 1;
    } else if (m_nightCaptureCnt == 2) {
        shot_ext->shot.ctl.aa.sceneMode = AA_SCENE_MODE_NIGHT_CAPTURE;
        shot_ext->shot.ctl.aa.aeTargetFpsRange[0] = 2;
        shot_ext->shot.ctl.aa.aeTargetFpsRange[1] = 30;
        m_nightCaptureCnt--;
        shot_ext->request_scc = 0;
    } else if (m_nightCaptureCnt == 3) {
        shot_ext->shot.ctl.aa.sceneMode = AA_SCENE_MODE_NIGHT_CAPTURE;
        shot_ext->shot.ctl.aa.aeTargetFpsRange[0] = 2;
        shot_ext->shot.ctl.aa.aeTargetFpsRange[1] = 30;
        m_nightCaptureCnt--;
        shot_ext->request_scc = 0;
    } else if (m_nightCaptureCnt == 4) {
        shot_ext->shot.ctl.aa.sceneMode = AA_SCENE_MODE_NIGHT_CAPTURE;
        shot_ext->shot.ctl.aa.aeTargetFpsRange[0] = 2;
        shot_ext->shot.ctl.aa.aeTargetFpsRange[1] = 30;
        m_nightCaptureCnt--;
        shot_ext->request_scc = 0;
    }
}

void ExynosCameraHWInterface2::m_setShotFrameDuration(struct camera2_shot_ext *shot_ext)
{
    uint64_t frameDuration = (33333 * 1000);

    if (shot_ext->shot.ctl.aa.aeTargetFpsRange[1])
        frameDuration = 1000 * 1000 / shot_ext->shot.ctl.aa.aeTargetFpsRange[1] * 1000;
    shot_ext->shot.ctl.sensor.frameDuration = frameDuration;
    /* Set max fps to 30, ISP controls fps by fps range[0] and frameduration */
    shot_ext->shot.ctl.aa.aeTargetFpsRange[1] = 30;
    if (m_requestManager->GetSkipCnt() > 0)
        shot_ext->shot.ctl.aa.aeTargetFpsRange[0] = 30;
}

void ExynosCameraHWInterface2::m_controlFlashTorch(struct camera2_shot_ext *shot_ext)
{
    if (shot_ext->shot.ctl.flash.flashMode == CAM2_FLASH_MODE_TORCH) {
        if (m_ctlInfo.flash.m_flashTorchMode == false)
            m_ctlInfo.flash.m_flashTorchMode = true;
    } else {
        if (m_ctlInfo.flash.m_flashTorchMode == true) {
            shot_ext->shot.ctl.flash.flashMode = CAM2_FLASH_MODE_OFF;
            shot_ext->shot.ctl.flash.firingPower = 0;
            m_ctlInfo.flash.m_flashTorchMode = false;
        } else {
            shot_ext->shot.ctl.flash.flashMode = CAM2_FLASH_MODE_NOP;
        }
    }
}

void ExynosCameraHWInterface2::m_controlFlashCapture(struct camera2_shot_ext *shot_ext,
                                                     int &matchedFrameCnt)
{
    if(shot_ext->shot.ctl.aa.captureIntent == AA_CAPTURE_INTENT_STILL_CAPTURE)
    {
        Mutex::Autolock lock(m_TriggerLock);

        if ((m_ctlInfo.flash.i_flashMode >= AA_AEMODE_ON_AUTO_FLASH) &&
            (m_cameraId == 0)) {
            /* Flash Auto, Always mode */
            if (!m_ctlInfo.flash.m_flashDecisionResult) {
                m_ctlInfo.flash.m_flashEnableFlg = false;
                m_ctlInfo.flash.m_afFlashFired = false;
                m_ctlInfo.flash.m_flashState = FLASH_STATE_NONE;
            } else if ((m_ctlInfo.flash.m_flashState == FLASH_STATE_AUTO_DONE) ||
                       (m_ctlInfo.flash.m_flashState == FLASH_STATE_AUTO_OFF)) {
                m_ctlInfo.flash.m_flashFrameCount = matchedFrameCnt;
                m_ctlInfo.flash.m_flashEnableFlg = true;
                m_ctlInfo.flash.m_afFlashFired = false;
                m_ctlInfo.flash.m_flashState = FLASH_STATE_CAPTURE;
            } else if (m_ctlInfo.flash.m_flashState < FLASH_STATE_AUTO_DONE) {
                m_ctlInfo.flash.m_flashEnableFlg = false;
                m_ctlInfo.flash.m_afFlashFired = false;
                m_ctlInfo.flash.m_flashState = FLASH_STATE_NONE;
            }
        } else {
            m_ctlInfo.flash.m_flashDecisionResult = false;
        }

        FLASH_LOG("(%s): State[%s], Mode[%d]", __FUNCTION__,
                            HAL_FlashState_Name[m_ctlInfo.flash.m_flashState],
                            m_ctlInfo.flash.i_flashMode);
    }
}

void ExynosCameraHWInterface2::m_setShotFlash(struct camera2_shot_ext *shot_ext)
{
    int nextState = FLASH_STATE_NO_TRANSITION;

    Mutex::Autolock lock(m_TriggerLock);

    /* Process Flash State Machine */
    switch (m_ctlInfo.flash.m_flashState) {
    case FLASH_STATE_ON:
        ALOGV("(%s): [Flash] Flash ON for Capture (%d)", __FUNCTION__, shot_ext->shot.ctl.request.frameCount);
        /* check AF locked */
        if (m_ctlInfo.flash.m_precaptureTriggerId > 0) {
            if (m_ctlInfo.flash.m_flashTimeOut == 0) {
                if (m_ctlInfo.flash.i_flashMode == AA_AEMODE_ON_ALWAYS_FLASH) {
                    shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_ON_ALWAYS;
                    m_ctlInfo.flash.m_flashTimeOut = 5;
                } else {
                    shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_ON;
                }
                nextState = FLASH_STATE_ON_WAIT;
            } else {
                m_ctlInfo.flash.m_flashTimeOut--;
            }
        } else {
            if (m_ctlInfo.flash.i_flashMode == AA_AEMODE_ON_ALWAYS_FLASH) {
                shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_ON_ALWAYS;
                m_ctlInfo.flash.m_flashTimeOut = 5;
            } else {
                shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_ON;
            }
            nextState = FLASH_STATE_ON_WAIT;
        }
        break;
    case FLASH_STATE_ON_WAIT:
        break;
    case FLASH_STATE_ON_DONE:
        if (!m_ctlInfo.flash.m_afFlashFired) {
            /* auto transition at pre-capture trigger */
            nextState = FLASH_STATE_AUTO_AE_AWB_LOCK;
        }
        break;
    case FLASH_STATE_AUTO_AE_AWB_LOCK:
        ALOGV("(%s): [Flash] IS_FLASH_AF_AUTO_AE_AWB_LOCK (%d)", __FUNCTION__, shot_ext->shot.ctl.request.frameCount);
        shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_AUTO;
        shot_ext->shot.ctl.aa.awbMode = AA_AWBMODE_LOCKED;
        nextState = FLASH_STATE_AE_AWB_LOCK_WAIT;
        break;
    case FLASH_STATE_AE_AWB_LOCK_WAIT:
        shot_ext->shot.ctl.aa.aeMode =(enum aa_aemode)0;
        shot_ext->shot.ctl.aa.awbMode = (enum aa_awbmode)0;
        break;
    case FLASH_STATE_AUTO_WAIT:
        if (m_ctlInfo.flash.m_flashDecisionResult) {
            if (shot_ext->shot.dm.flash.flashMode == CAM2_FLASH_MODE_OFF) {
                shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_OFF;
                nextState = FLASH_STATE_AUTO_DONE;
                break;
            }
        } else {
            shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_OFF;
            nextState = FLASH_STATE_AUTO_DONE;
            break;
        }
        shot_ext->shot.ctl.aa.aeMode =(enum aa_aemode)0;
        shot_ext->shot.ctl.aa.awbMode = (enum aa_awbmode)0;
        break;
    case FLASH_STATE_AUTO_DONE:
        ALOGV("(%s): [Flash] IS_FLASH_AF_AUTO DONE (%d)", __FUNCTION__, shot_ext->shot.ctl.request.frameCount);
        shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_OFF;
        break;
    case FLASH_STATE_AUTO_OFF:
        ALOGV("(%s): [Flash] IS_FLASH_AF_AUTO Clear (%d)", __FUNCTION__, shot_ext->shot.ctl.request.frameCount);
        shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_OFF;
        m_ctlInfo.flash.m_flashEnableFlg = false;
        break;
    case FLASH_STATE_CAPTURE:
        ALOGV("(%s): [Flash] IS_FLASH_CAPTURE (%d)", __FUNCTION__, shot_ext->shot.ctl.request.frameCount);
        m_ctlInfo.flash.m_flashTimeOut = FLASH_STABLE_WAIT_TIMEOUT;
        shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_CAPTURE;
        shot_ext->request_scc = 0;
        shot_ext->request_scp = 0;
        nextState = FLASH_STATE_CAPTURE_WAIT; /* auto transition */
        break;
    case FLASH_STATE_CAPTURE_WAIT:
        shot_ext->request_scc = 0;
        shot_ext->request_scp = 0;
        break;
    case FLASH_STATE_CAPTURE_JPEG:
        ALOGV("(%s): [Flash] Flash Capture  (%d)!!!!!", __FUNCTION__, (FLASH_STABLE_WAIT_TIMEOUT -m_ctlInfo.flash.m_flashTimeOut));
        shot_ext->request_scc = 1;
        shot_ext->request_scp = 1;
        nextState = FLASH_STATE_CAPTURE_END;  /* auto transition */
        break;
    case FLASH_STATE_CAPTURE_END:
        ALOGV("(%s): [Flash] Flash Capture END (%d)", __FUNCTION__, shot_ext->shot.ctl.request.frameCount);
        shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_OFF;
        shot_ext->request_scc = 0;
        shot_ext->request_scp = 0;
        m_ctlInfo.flash.m_flashEnableFlg = false;
        nextState = FLASH_STATE_NONE;
        m_ctlInfo.flash.m_afFlashFired= false;
        break;
    case FLASH_STATE_NONE:
        break;
    default:
        ALOGE("(%s): [Flash] flash state error!! (%d)", __FUNCTION__, m_ctlInfo.flash.m_flashState);
        break;
    }

    FLASH_LOG("(%s): [%s] => [%s]", __FUNCTION__,
                                HAL_FlashState_Name[m_ctlInfo.flash.m_flashState],
                                HAL_FlashState_Name[nextState]);

    if (nextState != FLASH_STATE_NO_TRANSITION)
        m_ctlInfo.flash.m_flashState = nextState;
}

void ExynosCameraHWInterface2::m_setShotSensorMode(struct camera2_shot_ext *shot_ext)
{
    if (m_wideAspect)
        shot_ext->setfile = ISS_SUB_SCENARIO_VIDEO;
    else
        shot_ext->setfile = ISS_SUB_SCENARIO_STILL;
}

void ExynosCameraHWInterface2::m_setShotReprocessing(struct camera2_shot_ext *shot_ext)
{
    if (shot_ext->isReprocessing) {
        ALOGV("(%s): ISP Qbuf - Reprocessing request", __FUNCTION__);
        m_currentReprocessOutStreams = shot_ext->shot.ctl.request.outputStreams[0];
        shot_ext->request_scp = 0;
        shot_ext->request_scc = 0;
        m_ctlInfo.flash.m_flashDecisionResult = false;
        m_ctlInfo.flash.m_flashEnableFlg = false;
        shot_ext->isReprocessing = 0;
    }
}

void ExynosCameraHWInterface2::m_setDmFaceRectangles(struct camera2_shot_ext *shot_ext)
{
    /* FD orientation axis transformation */
    for (int i = 0; i < CAMERA2_MAX_FACES; i++) {
        if (shot_ext->shot.dm.stats.faceRectangles[i][0] > 0)
            shot_ext->shot.dm.stats.faceRectangles[i][0] = (m_camera2->m_curCameraInfo->sensorW
                                                            * shot_ext->shot.dm.stats.faceRectangles[i][0])
                                                            / m_streamThreads[0].get()->m_parameters.width;
        if (shot_ext->shot.dm.stats.faceRectangles[i][1] > 0)
            shot_ext->shot.dm.stats.faceRectangles[i][1] = (m_camera2->m_curCameraInfo->sensorH
                                                            * shot_ext->shot.dm.stats.faceRectangles[i][1])
                                                            / m_streamThreads[0].get()->m_parameters.height;
        if (shot_ext->shot.dm.stats.faceRectangles[i][2] > 0)
            shot_ext->shot.dm.stats.faceRectangles[i][2] = (m_camera2->m_curCameraInfo->sensorW
                                                            * shot_ext->shot.dm.stats.faceRectangles[i][2])
                                                            / m_streamThreads[0].get()->m_parameters.width;
        if (shot_ext->shot.dm.stats.faceRectangles[i][3] > 0)
            shot_ext->shot.dm.stats.faceRectangles[i][3] = (m_camera2->m_curCameraInfo->sensorH
                                                            * shot_ext->shot.dm.stats.faceRectangles[i][3])
                                                            / m_streamThreads[0].get()->m_parameters.height;
    }
}

void ExynosCameraHWInterface2::m_setDmAeState(struct camera2_shot_ext *shot_ext)
{
    switch (m_ctlInfo.flash.i_flashMode) {
    case AA_AEMODE_ON: /* FLASH OFF MODE */
        /* At flash off mode, capture operation is done as zsl capture only */
        shot_ext->shot.dm.aa.aeState = AE_STATE_CONVERGED;
        break;
    case AA_AEMODE_ON_AUTO_FLASH: /* FLASH AUTO MODE */
        /* At flash auto mode, main flash have to be done if pre-flash was done. */
        if (m_ctlInfo.flash.m_flashDecisionResult && m_ctlInfo.flash.m_afFlashFired)
            shot_ext->shot.dm.aa.aeState = AE_STATE_FLASH_REQUIRED;
        break;
    default:
        break;
    }
}

void ExynosCameraHWInterface2::m_sensorThreadFunc(SignalDrivenThread * self)
{
    uint32_t        currentSignal = self->GetProcessingSignal();
    SensorThread *  selfThread      = ((SensorThread*)self);
    int index;
    int index_isp, index_sensor_qbuf;
    nsecs_t frameTime;

    ALOGV("DEBUG(%s): m_sensorThreadFunc (%x)", __FUNCTION__, currentSignal);

    if (currentSignal & SIGNAL_THREAD_RELEASE) {
        ALOGV("(%s): ENTER processing SIGNAL_THREAD_RELEASE", __FUNCTION__);

        ALOGV("(%s): calling sensor streamoff", __FUNCTION__);
        cam_int_streamoff(&(m_camera_info.sensor));
        ALOGV("(%s): calling sensor streamoff done", __FUNCTION__);

        m_camera_info.sensor.buffers = 0;
        ALOGV("DEBUG(%s): sensor calling reqbuf 0 ", __FUNCTION__);
        cam_int_reqbufs(&(m_camera_info.sensor));
        ALOGV("DEBUG(%s): sensor calling reqbuf 0 done", __FUNCTION__);
        m_camera_info.sensor.status = false;

        ALOGV("(%s): calling ISP streamoff", __FUNCTION__);
        isp_int_streamoff(&(m_camera_info.isp));
        ALOGV("(%s): calling ISP streamoff done", __FUNCTION__);

        m_camera_info.isp.buffers = 0;
        ALOGV("DEBUG(%s): isp calling reqbuf 0 ", __FUNCTION__);
        cam_int_reqbufs(&(m_camera_info.isp));
        ALOGV("DEBUG(%s): isp calling reqbuf 0 done", __FUNCTION__);

        exynos_v4l2_s_ctrl(m_camera_info.sensor.fd, V4L2_CID_IS_S_STREAM, IS_DISABLE_STREAM);

        m_requestManager->releaseSensorQ();
        m_requestManager->ResetEntry();
        ALOGV("(%s): EXIT processing SIGNAL_THREAD_RELEASE", __FUNCTION__);
        selfThread->SetSignal(SIGNAL_THREAD_TERMINATE);
        return;
    }

    if (currentSignal & SIGNAL_SENSOR_START_REQ_PROCESSING)
    {
        ALOGV("DEBUG(%s): SensorThread processing SIGNAL_SENSOR_START_REQ_PROCESSING", __FUNCTION__);
        int targetStreamIndex = 0, i=0;
        int matchedFrameCnt = -1, processingReqIndex;
        struct camera2_shot_ext *shot_ext;
        struct camera2_shot_ext *shot_ext_sensor_qbuf;
        struct camera2_shot_ext *shot_ext_capture;
        camera_metadata_t *currentRequest = NULL;
        bool triggered = false;
        bool rawDumpMode = false;
        int afMode;

        /* dqbuf from sensor */
        ALOGV("Sensor DQbuf start");
        index = cam_int_dqbuf(&(m_camera_info.sensor));
        m_requestManager->pushSensorQ(index);
        shot_ext = (struct camera2_shot_ext *)(m_camera_info.sensor.buffer[index].virt.extP[1]);
        ALOGV("Sensor DQbuf done index(%d)", index);

        index_sensor_qbuf = m_requestManager->popSensorQ();
        if (index_sensor_qbuf < 0){
            ALOGE("sensorQ is empty");
            return;
        }

        if (m_isAlreadyRegistered) {
            ALOGD("(%s): previously registered request exists", __FUNCTION__);
        } else {
            if (!m_isRequestQueueNull && !(m_requestManager->IsRequestQueueFull())) {
                Mutex::Autolock lock(m_TriggerLock);
                m_requestQueueOps->dequeue_request(m_requestQueueOps, &currentRequest);
                if (NULL == currentRequest) {
                    ALOGD("(%s): No more service requests left in the queue", __FUNCTION__);
                    m_isRequestQueueNull = true;
                    if (m_requestManager->IsVdisEnable()) {
                        ALOGD("(%s): enabling vdis bubble", __FUNCTION__);
                        m_vdisBubbleCnt = 1;
                    }
                } else {
                    m_requestManager->RegisterRequest(currentRequest, &afMode, m_currentAfRegion);

                    if (afMode != -1)
                        SetAfMode((enum aa_afmode)afMode);

                    ALOGV("(%s): remaining req cnt (%d)", __FUNCTION__, m_requestQueueOps->request_count(m_requestQueueOps));
                }
            }
        }

        if (m_vdisBubbleCnt == 0)
            processingReqIndex = m_requestManager->MarkProcessingRequest(&(m_camera_info.sensor.buffer[index_sensor_qbuf]));
        else
            processingReqIndex = -1;
        shot_ext_sensor_qbuf = (struct camera2_shot_ext *)(m_camera_info.sensor.buffer[index_sensor_qbuf].virt.extP[1]);

        m_isAlreadyRegistered = false;
        if (processingReqIndex >= 0 && shot_ext_sensor_qbuf->isReprocessing) {
            struct camera2_shot_ext *next_shot_ext;
            ALOGV("(%s): Sending signal for Reprocess request", __FUNCTION__);
            m_currentReprocessOutStreams = shot_ext_sensor_qbuf->shot.ctl.request.outputStreams[0];
            shot_ext_sensor_qbuf->request_scp = 0;
            shot_ext_sensor_qbuf->request_scc = 0;
            m_reprocessingFrameCnt = shot_ext_sensor_qbuf->shot.ctl.request.frameCount;
            memcpy(&m_jpegMetadata, (void*)(m_requestManager->GetInternalShotExtByFrameCnt(m_reprocessingFrameCnt)),
                sizeof(struct camera2_shot_ext));
            m_miscThread->SetSignal(SIGNAL_STREAM_REPROCESSING_START);

            if (!m_isRequestQueueNull && !(m_requestManager->IsRequestQueueFull())) {
                Mutex::Autolock lock(m_TriggerLock);
                m_requestQueueOps->dequeue_request(m_requestQueueOps, &currentRequest);
                if (NULL == currentRequest) {
                    ALOGD("(%s): ## No more service requests left in the queue", __FUNCTION__);
                    m_isRequestQueueNull = true;
                    if (m_requestManager->IsVdisEnable()) {
                        ALOGD("(%s): ## enabling vdis bubble", __FUNCTION__);
                        m_vdisBubbleCnt = 1;
                    }
                } else {
                    m_requestManager->RegisterRequest(currentRequest, &afMode, m_currentAfRegion);
                    if (afMode != -1)
                        SetAfMode((enum aa_afmode)afMode);

                    next_shot_ext = m_requestManager->GetInternalShotExtByFrameCnt(m_reprocessingFrameCnt + 1);
                    if (next_shot_ext && !(next_shot_ext->isReprocessing)) {
                        ALOGV("(%s): Processing next request", __FUNCTION__);
                        processingReqIndex = m_requestManager->MarkProcessingRequest(&(m_camera_info.sensor.buffer[index_sensor_qbuf]));
                        m_isAlreadyRegistered = false;
                    } else {
                        m_isAlreadyRegistered = true;
                    }
                }
            }
        }
        cam_int_qbuf(&(m_camera_info.sensor), index_sensor_qbuf);
        ALOGV("Sensor Qbuf done(%d)", index_sensor_qbuf);

        if (m_nightCaptureCnt != 0) {
            matchedFrameCnt = m_nightCaptureFrameCnt;
        } else if (m_ctlInfo.flash.m_flashState >= FLASH_STATE_CAPTURE) {
            matchedFrameCnt = m_ctlInfo.flash.m_flashFrameCount;
            ALOGV("Skip frame, request is fixed at %d", matchedFrameCnt);
        } else {
            matchedFrameCnt = m_requestManager->FindFrameCnt(shot_ext);
        }

        if (matchedFrameCnt == -1 && m_vdisBubbleCnt > 0) {
            matchedFrameCnt = m_vdisDupFrame;
        }

        if (matchedFrameCnt != -1) {
            if (m_vdisBubbleCnt == 0 || m_vdisDupFrame != matchedFrameCnt) {
                frameTime = systemTime();
                m_requestManager->RegisterTimestamp(matchedFrameCnt, &frameTime);
                m_requestManager->UpdateIspParameters(shot_ext, matchedFrameCnt, &m_ctlInfo);

                char value[PROPERTY_VALUE_MAX];
                property_get("camera.disable_zsl_mode", value, "0");
                if (!strcmp(value,"1"))
                    rawDumpMode = true;
                else
                    rawDumpMode = false;
            } else {
                ALOGV("bubble for vids: m_vdisBubbleCnt %d, matchedFrameCnt %d", m_vdisDupFrame, matchedFrameCnt);
            }

            /* face af mode setting in case of face priority scene mode */
            if (m_ctlInfo.scene.prevSceneMode != shot_ext->shot.ctl.aa.sceneMode) {
                ALOGV("(%s): Scene mode changed (%d)", __FUNCTION__, shot_ext->shot.ctl.aa.sceneMode);
                m_ctlInfo.scene.prevSceneMode = shot_ext->shot.ctl.aa.sceneMode;
            }

            m_lastSceneMode = shot_ext->shot.ctl.aa.sceneMode;

            /* Set Zoom */
            m_setShotZoom(shot_ext);

            /* Set AF */
            if (m_IsAfModeUpdateRequired)
                /* Set AF Mode */
                m_setShotAFMode(shot_ext);
            else
                /* Set AF Trigger */
                m_setShotAFTrigger(shot_ext);

            /* Set AF Region */
            m_setShotAFRegion(shot_ext);

            /* Set NightShot */
            m_setShotNightshot(shot_ext, matchedFrameCnt);

            /* Set Frame Duration From FPS Range */
            m_setShotFrameDuration(shot_ext);

            /* Set Sensor Mode */
            m_setShotSensorMode(shot_ext);

            /* Control Flash Torch */
            m_controlFlashTorch(shot_ext);

            /* Control Flash Capture */
            m_controlFlashCapture(shot_ext, matchedFrameCnt);

            /* Set Reprocessing */
            m_setShotReprocessing(shot_ext);

            if(m_ctlInfo.flash.m_flashEnableFlg) {
                /* Set Flash when flash is enabled */
                m_setShotFlash(shot_ext);
            }

            if (m_vdisBubbleCnt > 0 && m_vdisDupFrame == matchedFrameCnt) {
                shot_ext->dis_bypass = 1;
                shot_ext->dnr_bypass = 1;
                shot_ext->request_scp = 0;
                shot_ext->request_scc = 0;
                m_vdisBubbleCnt--;
                matchedFrameCnt = -1;
            } else {
                m_vdisDupFrame = matchedFrameCnt;
            }
            if (m_scpForceSuspended)
                shot_ext->request_scc = 0;

            uint32_t current_scp = shot_ext->request_scp;
            uint32_t current_scc = shot_ext->request_scc;

            if (shot_ext->shot.dm.request.frameCount == 0) {
                ALOGE("ERR(%s): dm.request.frameCount = %d", __FUNCTION__, shot_ext->shot.dm.request.frameCount);
            }

            /* raw file dump */
            if ((shot_ext->request_scc == 1) &&
                (shot_ext->shot.ctl.request.outputStreams[0] & STREAM_MASK_JPEG) &&
                rawDumpMode) {
                ALOGD("bayer dump - %d", matchedFrameCnt);
                dumpImage(&m_camera_info.sensor.buffer[index], "bayer", matchedFrameCnt, "raw");
            }

            cam_int_qbuf(&(m_camera_info.isp), index);

            ALOGV("### isp DQBUF start");
            index_isp = cam_int_dqbuf(&(m_camera_info.isp));

            shot_ext = (struct camera2_shot_ext *)(m_camera_info.isp.buffer[index_isp].virt.extP[1]);

            if (m_ctlInfo.flash.m_flashEnableFlg)
                OnFlashListenerISP(shot_ext);

            /* Set Dynamic Metadata for Face Rectangles */
            if (!shot_ext->fd_bypass) {
                m_setDmFaceRectangles(shot_ext);
            }

            /* Set Dynamic Metadata for AE State */
            if (shot_ext->shot.ctl.aa.sceneMode != AA_SCENE_MODE_NIGHT)
                m_setDmAeState(shot_ext);

            /* At scene mode face priority */
            if (shot_ext->shot.dm.aa.afMode == AA_AFMODE_CONTINUOUS_PICTURE_FACE)
                shot_ext->shot.dm.aa.afMode = AA_AFMODE_CONTINUOUS_PICTURE;

            if (matchedFrameCnt != -1 && m_nightCaptureCnt == 0 && (m_ctlInfo.flash.m_flashState < FLASH_STATE_CAPTURE)) {
                m_requestManager->ApplyDynamicMetadata(shot_ext);
            }

            if (current_scc != shot_ext->request_scc) {
                ALOGD("(%s): scc frame drop1 request_scc(%d to %d)",
                                __FUNCTION__, current_scc, shot_ext->request_scc);
                m_requestManager->NotifyStreamOutput(shot_ext->shot.ctl.request.frameCount);
            }
            if (shot_ext->request_scc) {
                ALOGV("send SIGNAL_STREAM_DATA_COMING (SCC)");
                if (shot_ext->shot.ctl.request.outputStreams[0] & STREAM_MASK_JPEG) {
                    if (m_ctlInfo.flash.m_flashState < FLASH_STATE_CAPTURE)
                        memcpy(&m_jpegMetadata, (void*)(m_requestManager->GetInternalShotExtByFrameCnt(shot_ext->shot.ctl.request.frameCount)),
                            sizeof(struct camera2_shot_ext));
                    else
                        memcpy(&m_jpegMetadata, (void*)shot_ext, sizeof(struct camera2_shot_ext));
                }
                m_streamThreads[1]->SetSignal(SIGNAL_STREAM_DATA_COMING);
            }

            if (current_scp != shot_ext->request_scp) {
                ALOGD("(%s): scp frame drop1 request_scp(%d to %d)",
                                __FUNCTION__, current_scp, shot_ext->request_scp);
                m_requestManager->NotifyStreamOutput(shot_ext->shot.ctl.request.frameCount);
            }
            if (shot_ext->request_scp) {
                ALOGV("send SIGNAL_STREAM_DATA_COMING (SCP)");
                m_streamThreads[0]->SetSignal(SIGNAL_STREAM_DATA_COMING);
            }

            OnAfNotification(shot_ext->shot.dm.aa.afState);
            OnPrecaptureMeteringNotificationISP();
        }   else {
            memcpy(&shot_ext->shot.ctl, &m_camera_info.dummy_shot.shot.ctl, sizeof(struct camera2_ctl));
            shot_ext->shot.ctl.request.frameCount = 0xfffffffe;
            shot_ext->request_sensor = 1;
            shot_ext->dis_bypass = 1;
            shot_ext->dnr_bypass = 1;
            shot_ext->fd_bypass = 1;
            shot_ext->drc_bypass = 1;
            shot_ext->request_scc = 0;
            shot_ext->request_scp = 0;
            if (m_wideAspect) {
                shot_ext->setfile = ISS_SUB_SCENARIO_VIDEO;
            } else {
                shot_ext->setfile = ISS_SUB_SCENARIO_STILL;
            }
            shot_ext->shot.ctl.aa.sceneMode = (enum aa_scene_mode)m_lastSceneMode;
            if (shot_ext->shot.ctl.aa.sceneMode == AA_SCENE_MODE_NIGHT_CAPTURE || shot_ext->shot.ctl.aa.sceneMode == AA_SCENE_MODE_NIGHT) {
                shot_ext->shot.ctl.aa.aeTargetFpsRange[0] = 8;
                shot_ext->shot.ctl.aa.aeTargetFpsRange[1] = 30;
            }
            shot_ext->shot.ctl.aa.aeflashMode = AA_FLASHMODE_OFF;
            shot_ext->shot.ctl.flash.flashMode = CAM2_FLASH_MODE_OFF;
            ALOGV("### isp QBUF start (bubble)");
            ALOGV("bubble: queued  aa(%d) aemode(%d) awb(%d) afmode(%d) trigger(%d)",
                (int)(shot_ext->shot.ctl.aa.mode), (int)(shot_ext->shot.ctl.aa.aeMode),
                (int)(shot_ext->shot.ctl.aa.awbMode), (int)(shot_ext->shot.ctl.aa.afMode),
                (int)(shot_ext->shot.ctl.aa.afTrigger));

            cam_int_qbuf(&(m_camera_info.isp), index);
            ALOGV("### isp DQBUF start (bubble)");
            index_isp = cam_int_dqbuf(&(m_camera_info.isp));
            shot_ext = (struct camera2_shot_ext *)(m_camera_info.isp.buffer[index_isp].virt.extP[1]);
            ALOGV("bubble: DM aa(%d) aemode(%d) awb(%d) afmode(%d)",
                (int)(shot_ext->shot.dm.aa.mode), (int)(shot_ext->shot.dm.aa.aeMode),
                (int)(shot_ext->shot.dm.aa.awbMode),
                (int)(shot_ext->shot.dm.aa.afMode));

            OnAfNotification(shot_ext->shot.dm.aa.afState);
        }

        selfThread->SetSignal(SIGNAL_SENSOR_START_REQ_PROCESSING);
    }
    return;
}

void ExynosCameraHWInterface2::m_streamBufferInit(SignalDrivenThread *self)
{
    Mutex::Autolock lock(m_streamInitLock);
    uint32_t                currentSignal   = self->GetProcessingSignal();
    StreamThread *          selfThread      = ((StreamThread*)self);
    stream_parameters_t     *selfStreamParms =  &(selfThread->m_parameters);
    node_info_t             *currentNode    = selfStreamParms->node;
    substream_parameters_t  *subParms;
    buffer_handle_t * buf = NULL;
    status_t res;
    void *virtAddr[3];
    int i, j;
    int index;
    nsecs_t timestamp;

    if (!(selfThread->m_isBufferInit))
    {
        for ( i=0 ; i < selfStreamParms->numSvcBuffers; i++) {
            res = selfStreamParms->streamOps->dequeue_buffer(selfStreamParms->streamOps, &buf);
            if (res != NO_ERROR || buf == NULL) {
                ALOGE("ERR(%s): Init: unable to dequeue buffer : %d",__FUNCTION__ , res);
                return;
            }
            ALOGV("DEBUG(%s): got buf(%x) version(%d), numFds(%d), numInts(%d)", __FUNCTION__, (uint32_t)(*buf),
               ((native_handle_t*)(*buf))->version, ((native_handle_t*)(*buf))->numFds, ((native_handle_t*)(*buf))->numInts);

            index = selfThread->findBufferIndex(buf);
            if (index == -1) {
                ALOGE("ERR(%s): could not find buffer index", __FUNCTION__);
            }
            else {
                ALOGV("DEBUG(%s): found buffer index[%d] - status(%d)",
                    __FUNCTION__, index, selfStreamParms->svcBufStatus[index]);
                if (selfStreamParms->svcBufStatus[index]== REQUIRES_DQ_FROM_SVC)
                    selfStreamParms->svcBufStatus[index] = ON_DRIVER;
                else if (selfStreamParms->svcBufStatus[index]== ON_SERVICE)
                    selfStreamParms->svcBufStatus[index] = ON_HAL;
                else {
                    ALOGV("DBG(%s): buffer status abnormal (%d) "
                        , __FUNCTION__, selfStreamParms->svcBufStatus[index]);
                }
                selfStreamParms->numSvcBufsInHal++;
            }
            selfStreamParms->bufIndex = 0;
        }
        selfThread->m_isBufferInit = true;
    }
    for (int i = 0 ; i < NUM_MAX_SUBSTREAM ; i++) {
        if (selfThread->m_attachedSubStreams[i].streamId == -1)
            continue;

        subParms = &m_subStreams[selfThread->m_attachedSubStreams[i].streamId];
        if (subParms->type && subParms->needBufferInit) {
            ALOGV("(%s): [subStream] (id:%d) Buffer Initialization numsvcbuf(%d)",
                __FUNCTION__, selfThread->m_attachedSubStreams[i].streamId, subParms->numSvcBuffers);
            int checkingIndex = 0;
            bool found = false;
            for ( i = 0 ; i < subParms->numSvcBuffers; i++) {
                res = subParms->streamOps->dequeue_buffer(subParms->streamOps, &buf);
                if (res != NO_ERROR || buf == NULL) {
                    ALOGE("ERR(%s): Init: unable to dequeue buffer : %d",__FUNCTION__ , res);
                    return;
                }
                subParms->numSvcBufsInHal++;
                ALOGV("DEBUG(%s): [subStream] got buf(%x) bufInHal(%d) version(%d), numFds(%d), numInts(%d)", __FUNCTION__, (uint32_t)(*buf),
                   subParms->numSvcBufsInHal, ((native_handle_t*)(*buf))->version, ((native_handle_t*)(*buf))->numFds, ((native_handle_t*)(*buf))->numInts);

                if (m_grallocHal->lock(m_grallocHal, *buf,
                       subParms->usage, 0, 0,
                       subParms->width, subParms->height, virtAddr) != 0) {
                    ALOGE("ERR(%s): could not obtain gralloc buffer", __FUNCTION__);
                }
                else {
                      ALOGV("DEBUG(%s): [subStream] locked img buf plane0(%x) plane1(%x) plane2(%x)",
                        __FUNCTION__, (unsigned int)virtAddr[0], (unsigned int)virtAddr[1], (unsigned int)virtAddr[2]);
                }
                found = false;
                for (checkingIndex = 0; checkingIndex < subParms->numSvcBuffers ; checkingIndex++) {
                    if (subParms->svcBufHandle[checkingIndex] == *buf ) {
                        found = true;
                        break;
                    }
                }
                ALOGV("DEBUG(%s): [subStream] found(%d) - index[%d]", __FUNCTION__, found, checkingIndex);
                if (!found) break;

                index = checkingIndex;

                if (index == -1) {
                    ALOGV("ERR(%s): could not find buffer index", __FUNCTION__);
                }
                else {
                    ALOGV("DEBUG(%s): found buffer index[%d] - status(%d)",
                        __FUNCTION__, index, subParms->svcBufStatus[index]);
                    if (subParms->svcBufStatus[index]== ON_SERVICE)
                        subParms->svcBufStatus[index] = ON_HAL;
                    else {
                        ALOGV("DBG(%s): buffer status abnormal (%d) "
                            , __FUNCTION__, subParms->svcBufStatus[index]);
                    }
                    if (*buf != subParms->svcBufHandle[index])
                        ALOGV("DBG(%s): different buf_handle index ", __FUNCTION__);
                    else
                        ALOGV("DEBUG(%s): same buf_handle index", __FUNCTION__);
                }
                subParms->svcBufIndex = 0;
            }
            if (subParms->type == SUBSTREAM_TYPE_JPEG) {
                m_resizeBuf.size.extS[0] = ALIGN(subParms->width, 16) * ALIGN(subParms->height, 16) * 2;
                m_resizeBuf.size.extS[1] = 0;
                m_resizeBuf.size.extS[2] = 0;

                if (allocCameraMemory(m_ionCameraClient, &m_resizeBuf, 1, 1 << 0) == -1) {
                    ALOGE("ERR(%s): Failed to allocate resize buf", __FUNCTION__);
                }
            }
            if (subParms->type == SUBSTREAM_TYPE_PRVCB) {
                m_getAlignedYUVSize(HAL_PIXEL_FORMAT_2_V4L2_PIX(subParms->internalFormat), subParms->width,
                subParms->height, &m_previewCbBuf);

                if (allocCameraMemory(m_ionCameraClient, &m_previewCbBuf, subParms->internalPlanes) == -1) {
                    ALOGE("ERR(%s): Failed to allocate prvcb buf", __FUNCTION__);
                }
            }
            subParms->needBufferInit= false;
        }
    }
}

void ExynosCameraHWInterface2::m_streamThreadInitialize(SignalDrivenThread * self)
{
    StreamThread *          selfThread      = ((StreamThread*)self);
    ALOGV("DEBUG(%s): ", __FUNCTION__ );
    memset(&(selfThread->m_parameters), 0, sizeof(stream_parameters_t));
    selfThread->m_isBufferInit = false;
    for (int i = 0 ; i < NUM_MAX_SUBSTREAM ; i++) {
        selfThread->m_attachedSubStreams[i].streamId    = -1;
        selfThread->m_attachedSubStreams[i].priority    = 0;
    }
    return;
}

int ExynosCameraHWInterface2::m_runSubStreamFunc(StreamThread *selfThread, ExynosBuffer *srcImageBuf,
    int stream_id, nsecs_t frameTimeStamp)
{
    substream_parameters_t  *subParms = &m_subStreams[stream_id];

    switch (stream_id) {

    case STREAM_ID_JPEG:
        return m_jpegCreator(selfThread, srcImageBuf, frameTimeStamp);

    case STREAM_ID_RECORD:
        return m_recordCreator(selfThread, srcImageBuf, frameTimeStamp);

    case STREAM_ID_PRVCB:
        return m_prvcbCreator(selfThread, srcImageBuf, frameTimeStamp);

    default:
        return 0;
    }
}
void ExynosCameraHWInterface2::m_streamFunc_direct(SignalDrivenThread *self)
{
    uint32_t                currentSignal   = self->GetProcessingSignal();
    StreamThread *          selfThread      = ((StreamThread*)self);
    stream_parameters_t     *selfStreamParms =  &(selfThread->m_parameters);
    node_info_t             *currentNode    = selfStreamParms->node;
    int i = 0;
    nsecs_t frameTimeStamp;

    if (currentSignal & SIGNAL_THREAD_RELEASE) {
        ALOGV("(%s): [%d] START SIGNAL_THREAD_RELEASE", __FUNCTION__, selfThread->m_index);

        if (selfThread->m_isBufferInit) {
            if (!(currentNode->fd == m_camera_info.capture.fd && m_camera_info.capture.status == false)) {
                ALOGV("(%s): [%d] calling streamoff (fd:%d)", __FUNCTION__,
                    selfThread->m_index, currentNode->fd);
                if (cam_int_streamoff(currentNode) < 0 ) {
                    ALOGE("ERR(%s): stream off fail", __FUNCTION__);
                }
                ALOGV("(%s): [%d] streamoff done and calling reqbuf 0 (fd:%d)", __FUNCTION__,
                        selfThread->m_index, currentNode->fd);
                currentNode->buffers = 0;
                cam_int_reqbufs(currentNode);
                ALOGV("(%s): [%d] reqbuf 0 DONE (fd:%d)", __FUNCTION__,
                        selfThread->m_index, currentNode->fd);
            }
        }
        // free metabuffers
        for (i = 0; i < NUM_MAX_CAMERA_BUFFERS; i++)
            if (selfStreamParms->metaBuffers[i].fd.extFd[0] != 0) {
                freeCameraMemory(&(selfStreamParms->metaBuffers[i]), 1);
                selfStreamParms->metaBuffers[i].fd.extFd[0] = 0;
                selfStreamParms->metaBuffers[i].size.extS[0] = 0;
            }

        selfThread->m_isBufferInit = false;
        selfThread->m_releasing = false;
        selfThread->m_activated = false;
        ALOGV("(%s): [%d] END  SIGNAL_THREAD_RELEASE", __FUNCTION__, selfThread->m_index);
        return;
    }
    if (currentSignal & SIGNAL_STREAM_DATA_COMING) {
        buffer_handle_t * buf = NULL;
        status_t res = 0;
        int i, j;
        int index;
        nsecs_t timestamp;
        camera2_stream *frame;
        uint8_t currentOutputStreams;
        bool directOutputEnabled = false;
        int numOfUndqbuf = 0;

        ALOGV("(%s): streamthread[%d] START SIGNAL_STREAM_DATA_COMING", __FUNCTION__,selfThread->m_index);

        m_streamBufferInit(self);

        do {
            ALOGV("DEBUG(%s): streamthread[%d] type(%d) DQBUF START ",__FUNCTION__,
                selfThread->m_index, selfThread->streamType);

            selfStreamParms->bufIndex = cam_int_dqbuf(currentNode, selfStreamParms->planes + selfStreamParms->metaPlanes);
            frame = (struct camera2_stream *)(selfStreamParms->metaBuffers[selfStreamParms->bufIndex].virt.extP[0]);
            frameTimeStamp = m_requestManager->GetTimestampByFrameCnt(frame->rcount);
            currentOutputStreams = m_requestManager->GetOutputStreamByFrameCnt(frame->rcount);
            ALOGV("frame count streamthread[%d] : %d, outputStream(%x)", selfThread->m_index, frame->rcount, currentOutputStreams);
            if (((currentOutputStreams & STREAM_MASK_PREVIEW) && selfThread->m_index == 0)||
                 ((currentOutputStreams & STREAM_MASK_ZSL) && selfThread->m_index == 1)) {
                directOutputEnabled = true;
            }
            if (!directOutputEnabled) {
                if (!m_nightCaptureFrameCnt)
                    m_requestManager->NotifyStreamOutput(frame->rcount);
            }

            ALOGV("(%s): streamthread[%d] DQBUF done index(%d)", __FUNCTION__,
                selfThread->m_index, selfStreamParms->bufIndex);

            if (selfStreamParms->svcBufStatus[selfStreamParms->bufIndex] !=  ON_DRIVER)
                ALOGV("DBG(%s): DQed buffer status abnormal (%d) ",
                       __FUNCTION__, selfStreamParms->svcBufStatus[selfStreamParms->bufIndex]);
            selfStreamParms->svcBufStatus[selfStreamParms->bufIndex] = ON_HAL;

            for (int i = 0 ; i < NUM_MAX_SUBSTREAM ; i++) {
                if (selfThread->m_attachedSubStreams[i].streamId == -1)
                    continue;
                if (currentOutputStreams & (1<<selfThread->m_attachedSubStreams[i].streamId)) {
                    m_runSubStreamFunc(selfThread, &(selfStreamParms->svcBuffers[selfStreamParms->bufIndex]),
                        selfThread->m_attachedSubStreams[i].streamId, frameTimeStamp);
                }
            }

            if (m_requestManager->GetSkipCnt() <= 0) {
                if ((currentOutputStreams & STREAM_MASK_PREVIEW) && selfThread->m_index == 0) {
                    ALOGV("** Display Preview(frameCnt:%d)", frame->rcount);
                    res = selfStreamParms->streamOps->enqueue_buffer(selfStreamParms->streamOps,
                            frameTimeStamp,
                            &(selfStreamParms->svcBufHandle[selfStreamParms->bufIndex]));
                }
                else if ((currentOutputStreams & STREAM_MASK_ZSL) && selfThread->m_index == 1) {
                    ALOGV("** SCC output (frameCnt:%d)", frame->rcount);
                    res = selfStreamParms->streamOps->enqueue_buffer(selfStreamParms->streamOps,
                                frameTimeStamp,
                                &(selfStreamParms->svcBufHandle[selfStreamParms->bufIndex]));
                }
                else {
                    res = selfStreamParms->streamOps->cancel_buffer(selfStreamParms->streamOps,
                            &(selfStreamParms->svcBufHandle[selfStreamParms->bufIndex]));
                    ALOGV("DEBUG(%s): streamthread[%d] cancel_buffer to svc done res(%d)", __FUNCTION__, selfThread->m_index, res);
                }
                ALOGV("DEBUG(%s): streamthread[%d] enqueue_buffer to svc done res(%d)", __FUNCTION__, selfThread->m_index, res);
            }
            else {
                res = selfStreamParms->streamOps->cancel_buffer(selfStreamParms->streamOps,
                        &(selfStreamParms->svcBufHandle[selfStreamParms->bufIndex]));
                ALOGV("DEBUG(%s): streamthread[%d] cancel_buffer to svc done res(%d)", __FUNCTION__, selfThread->m_index, res);
                if ((currentOutputStreams & STREAM_MASK_PREVIEW) && selfThread->m_index == 0)
                    m_requestManager->DecreaseSkipCnt();
            }
            if (directOutputEnabled) {
                if (!m_nightCaptureFrameCnt)
                     m_requestManager->NotifyStreamOutput(frame->rcount);
            }
            if (res == 0) {
                selfStreamParms->svcBufStatus[selfStreamParms->bufIndex] = ON_SERVICE;
                selfStreamParms->numSvcBufsInHal--;
            }
            else {
                selfStreamParms->svcBufStatus[selfStreamParms->bufIndex] = ON_HAL;
            }

        }
        while(0);

        while ((selfStreamParms->numSvcBufsInHal - (selfStreamParms->numSvcBuffers - NUM_SCP_BUFFERS)) 
                    < selfStreamParms->minUndequedBuffer) {
            res = selfStreamParms->streamOps->dequeue_buffer(selfStreamParms->streamOps, &buf);
            if (res != NO_ERROR || buf == NULL) {
                ALOGV("DEBUG(%s): streamthread[%d] dequeue_buffer fail res(%d) numInHal(%d)",__FUNCTION__ , selfThread->m_index,  res, selfStreamParms->numSvcBufsInHal);
                break;
            }
            selfStreamParms->numSvcBufsInHal++;
            ALOGV("DEBUG(%s): streamthread[%d] got buf(%x) numInHal(%d) version(%d), numFds(%d), numInts(%d)", __FUNCTION__,
                selfThread->m_index, (uint32_t)(*buf), selfStreamParms->numSvcBufsInHal,
               ((native_handle_t*)(*buf))->version, ((native_handle_t*)(*buf))->numFds, ((native_handle_t*)(*buf))->numInts);
            const private_handle_t *priv_handle = reinterpret_cast<const private_handle_t *>(*buf);

            bool found = false;
            int checkingIndex = 0;
            for (checkingIndex = 0; checkingIndex < selfStreamParms->numSvcBuffers ; checkingIndex++) {
                if (priv_handle->fd == selfStreamParms->svcBuffers[checkingIndex].fd.extFd[0] ) {
                    found = true;
                    break;
                }
            }
            if (!found) break;
            selfStreamParms->bufIndex = checkingIndex;
            if (selfStreamParms->bufIndex < selfStreamParms->numHwBuffers) {
                uint32_t    plane_index = 0;
                ExynosBuffer*  currentBuf = &(selfStreamParms->svcBuffers[selfStreamParms->bufIndex]);
                struct v4l2_buffer v4l2_buf;
                struct v4l2_plane  planes[VIDEO_MAX_PLANES];

                v4l2_buf.m.planes   = planes;
                v4l2_buf.type       = currentNode->type;
                v4l2_buf.memory     = currentNode->memory;
                v4l2_buf.index      = selfStreamParms->bufIndex;
                v4l2_buf.length     = currentNode->planes;

                v4l2_buf.m.planes[0].m.fd = priv_handle->fd;
                v4l2_buf.m.planes[2].m.fd = priv_handle->fd1;
                v4l2_buf.m.planes[1].m.fd = priv_handle->fd2;
                for (plane_index=0 ; plane_index < v4l2_buf.length ; plane_index++) {
                    v4l2_buf.m.planes[plane_index].length  = currentBuf->size.extS[plane_index];
                }
                /* add plane for metadata*/
                v4l2_buf.length += selfStreamParms->metaPlanes;
                v4l2_buf.m.planes[v4l2_buf.length-1].m.fd = selfStreamParms->metaBuffers[selfStreamParms->bufIndex].fd.extFd[0];
                v4l2_buf.m.planes[v4l2_buf.length-1].length = selfStreamParms->metaBuffers[selfStreamParms->bufIndex].size.extS[0];
                if (exynos_v4l2_qbuf(currentNode->fd, &v4l2_buf) < 0) {
                    ALOGE("ERR(%s): streamthread[%d] exynos_v4l2_qbuf() fail",
                        __FUNCTION__, selfThread->m_index);
                    return;
                }
                selfStreamParms->svcBufStatus[selfStreamParms->bufIndex] = ON_DRIVER;
                ALOGV("DEBUG(%s): streamthread[%d] QBUF done index(%d)",
                    __FUNCTION__, selfThread->m_index, selfStreamParms->bufIndex);
            }
        }

        ALOGV("(%s): streamthread[%d] END SIGNAL_STREAM_DATA_COMING", __FUNCTION__,selfThread->m_index);
    }
    return;
}

void ExynosCameraHWInterface2::m_streamFunc_indirect(SignalDrivenThread *self)
{
    uint32_t                currentSignal   = self->GetProcessingSignal();
    StreamThread *          selfThread      = ((StreamThread*)self);
    stream_parameters_t     *selfStreamParms =  &(selfThread->m_parameters);
    node_info_t             *currentNode    = selfStreamParms->node;


    if (currentSignal & SIGNAL_THREAD_RELEASE) {
        ALOGV("(%s): [%d] START SIGNAL_THREAD_RELEASE", __FUNCTION__, selfThread->m_index);

        if (selfThread->m_isBufferInit) {
            if (currentNode->fd == m_camera_info.capture.fd) {
                if (m_camera_info.capture.status == true) {
                    ALOGV("DEBUG(%s): calling streamthread[%d] streamoff (fd:%d)", __FUNCTION__,
                    selfThread->m_index, currentNode->fd);
                    if (cam_int_streamoff(currentNode) < 0 ){
                        ALOGE("ERR(%s): stream off fail", __FUNCTION__);
                    } else {
                        m_camera_info.capture.status = false;
                    }
                }
            } else {
                ALOGV("DEBUG(%s): calling streamthread[%d] streamoff (fd:%d)", __FUNCTION__,
                selfThread->m_index, currentNode->fd);
                if (cam_int_streamoff(currentNode) < 0 ){
                    ALOGE("ERR(%s): stream off fail", __FUNCTION__);
                }
            }
            ALOGV("DEBUG(%s): calling streamthread[%d] streamoff done", __FUNCTION__, selfThread->m_index);
            ALOGV("DEBUG(%s): calling streamthread[%d] reqbuf 0 (fd:%d)", __FUNCTION__,
                    selfThread->m_index, currentNode->fd);
            currentNode->buffers = 0;
            cam_int_reqbufs(currentNode);
            ALOGV("DEBUG(%s): calling streamthread[%d] reqbuf 0 DONE(fd:%d)", __FUNCTION__,
                    selfThread->m_index, currentNode->fd);
        }

        selfThread->m_isBufferInit = false;
        selfThread->m_releasing = false;
        selfThread->m_activated = false;
        ALOGV("(%s): [%d] END SIGNAL_THREAD_RELEASE", __FUNCTION__, selfThread->m_index);
        return;
    }

    if (currentSignal & SIGNAL_STREAM_DATA_COMING) {
        camera2_stream *frame;
        uint8_t currentOutputStreams;
        nsecs_t frameTimeStamp;

        ALOGV("DEBUG(%s): streamthread[%d] processing SIGNAL_STREAM_DATA_COMING",
            __FUNCTION__,selfThread->m_index);

        m_streamBufferInit(self);

        ALOGV("DEBUG(%s): streamthread[%d] DQBUF START", __FUNCTION__, selfThread->m_index);
        selfStreamParms->bufIndex = cam_int_dqbuf(currentNode);
        ALOGV("DEBUG(%s): streamthread[%d] DQBUF done index(%d)",__FUNCTION__,
            selfThread->m_index, selfStreamParms->bufIndex);

        frame = (struct camera2_stream *)(currentNode->buffer[selfStreamParms->bufIndex].virt.extP[selfStreamParms->planes -1]);
        frameTimeStamp = m_requestManager->GetTimestampByFrameCnt(frame->rcount);
        currentOutputStreams = m_requestManager->GetOutputStreamByFrameCnt(frame->rcount);
        ALOGV("frame count(SCC) : %d outputStream(%x)",  frame->rcount, currentOutputStreams);

        for (int i = 0 ; i < NUM_MAX_SUBSTREAM ; i++) {
            if (selfThread->m_attachedSubStreams[i].streamId == -1)
                continue;
            if (currentOutputStreams & (1<<selfThread->m_attachedSubStreams[i].streamId)) {
                m_requestManager->NotifyStreamOutput(frame->rcount);
                m_runSubStreamFunc(selfThread, &(currentNode->buffer[selfStreamParms->bufIndex]),
                    selfThread->m_attachedSubStreams[i].streamId, frameTimeStamp);
            }
        }
        cam_int_qbuf(currentNode, selfStreamParms->bufIndex);
        ALOGV("DEBUG(%s): streamthread[%d] QBUF DONE", __FUNCTION__, selfThread->m_index);



        ALOGV("DEBUG(%s): streamthread[%d] processing SIGNAL_STREAM_DATA_COMING DONE",
            __FUNCTION__, selfThread->m_index);
    }


    return;
}

void ExynosCameraHWInterface2::m_streamThreadFunc(SignalDrivenThread * self)
{
    uint32_t                currentSignal   = self->GetProcessingSignal();
    StreamThread *          selfThread      = ((StreamThread*)self);

    ALOGV("DEBUG(%s): m_streamThreadFunc[%d] (%x)", __FUNCTION__, selfThread->m_index, currentSignal);

    // Do something in Child thread handler
    // Should change function to class that inherited StreamThread class to support dynamic stream allocation
    if (selfThread->streamType == STREAM_TYPE_DIRECT) {
        m_streamFunc_direct(self);
    } else if (selfThread->streamType == STREAM_TYPE_INDIRECT) {
        m_streamFunc_indirect(self);
    }

    return;
}

void ExynosCameraHWInterface2::m_jpegEncThreadFunc(SignalDrivenThread * self)
{
    uint32_t            currentSignal   = self->GetProcessingSignal();
    JpegEncThread       *selfThread     = (JpegEncThread*)self;
    substream_parameters_t  *subParms   = &m_subStreams[STREAM_ID_JPEG];

    ALOGV("DEBUG(%s): signal (%x)", __FUNCTION__, currentSignal);

    if (currentSignal & SIGNAL_THREAD_RELEASE) {
        ALOGV("(%s): START SIGNAL_THREAD_RELEASE", __FUNCTION__);

        ALOGV("(%s): END SIGNAL_THREAD_RELEASE", __FUNCTION__);
        selfThread->SetSignal(SIGNAL_THREAD_TERMINATE);
        return;
    }

    if (currentSignal & SIGNAL_JPEG_START_ENCODING) {
        ALOGV("(%s): START SIGNAL_JPEG_START_ENCODING", __FUNCTION__);
        bool jpegRes = false;
        camera2_jpeg_blob *jpegBlob = NULL;
        int jpegSize = 0;
        int jpegBufSize = 0;
        char *jpegBuffer = NULL;
        status_t    res;

        jpegBufSize = m_jpegEncParameters.m_jpegBuf.size.extS[0];
        jpegRes = yuv2Jpeg(&m_jpegEncParameters.m_yuvBuf, &m_jpegEncParameters.m_jpegBuf,
                        &m_jpegEncParameters.m_rect, &m_jpegEncParameters.m_exifInfo,
                        m_jpegEncParameters.m_jpegQuality, m_jpegEncParameters.m_thumbQuality);

        if (jpegRes) {
            jpegSize = m_jpegEncParameters.m_jpegBuf.size.s;
            ALOGD("(%s): (%d x %d) encoded size(%d)", __FUNCTION__,
                    m_jpegEncParameters.m_rect.w, m_jpegEncParameters.m_rect.h, jpegSize);

            jpegBuffer = (char*)(m_jpegEncParameters.m_jpegBuf.virt.extP[0]);
            jpegBlob = (camera2_jpeg_blob*)(&jpegBuffer[jpegBufSize - sizeof(camera2_jpeg_blob)]);

            if (jpegBuffer[jpegSize-1] == 0)
                jpegSize--;
            jpegBlob->jpeg_size = jpegSize;
            jpegBlob->jpeg_blob_id = CAMERA2_JPEG_BLOB_ID;
            res = subParms->streamOps->enqueue_buffer(subParms->streamOps, m_jpegEncParameters.m_frameTimeStamp,
                    &subParms->svcBufHandle[m_jpegEncParameters.m_svcBufIndex]);

            ALOGV("DEBUG(%s): enqueue_buffer index(%d) to svc done res(%d)",
                    __FUNCTION__, m_jpegEncParameters.m_svcBufIndex, res);
            if (res == 0) {
                subParms->svcBufStatus[m_jpegEncParameters.m_svcBufIndex] = ON_SERVICE;
                subParms->numSvcBufsInHal--;
            } else {
                subParms->svcBufStatus[m_jpegEncParameters.m_svcBufIndex] = ON_HAL;
            }
        }
        m_jpegEncThread->m_inProgressCount--;
        ALOGV("(%s): END SIGNAL_JPEG_START_ENCODING", __FUNCTION__);
    }
    return;
}

void ExynosCameraHWInterface2::m_miscThreadFunc(SignalDrivenThread *self)
{
    uint32_t currentSignal = self->GetProcessingSignal();
    MiscThread *selfThread = (MiscThread *)self;

    ALOGV("(%s): signal (%x)", __FUNCTION__, currentSignal);

    if (currentSignal & SIGNAL_THREAD_RELEASE) {
        ALOGV("(%s): START SIGNAL_THREAD_RELEASE", __FUNCTION__);

        ALOGV("(%s): END SIGNAL_THREAD_RELEASE", __FUNCTION__);
        selfThread->SetSignal(SIGNAL_THREAD_TERMINATE);
        return;
    }

    if (currentSignal & SIGNAL_STREAM_REPROCESSING_START) {
        StreamThread *sccThread = (StreamThread *)(m_streamThreads[1].get());
        stream_parameters_t *sccStreamParms = &(sccThread->m_parameters);
        status_t res;
        buffer_handle_t *buf = NULL;
        bool found = false;
        nsecs_t frameTimeStamp;

        ALOGV("(%s):  START SIGNAL_STREAM_REPROCESSING_START", __FUNCTION__);

        m_streamBufferInit(sccThread);

        res = m_reprocessOps->acquire_buffer(m_reprocessOps, &buf);
        if (res != NO_ERROR || buf == NULL) {
            ALOGE("(%s): [reprocess] unable to acquire_buffer : %d", __FUNCTION__, res);
            return;
        }

        const private_handle_t *priv_handle = reinterpret_cast<const private_handle_t *>(*buf);
        int checkingIndex = 0;
        for (checkingIndex = 0; checkingIndex < sccStreamParms->numSvcBuffers; checkingIndex++) {
            if (priv_handle->fd == sccStreamParms->svcBuffers[checkingIndex].fd.extFd[0]) {
                found = true;
                break;
            }
        }
        ALOGV("(%s): dequeued buf %x => found(%d) index(%d)",
            __FUNCTION__, (unsigned int)buf, found, checkingIndex);

        if (!found)
            return;

        for (int i = 0; i < NUM_MAX_SUBSTREAM; i++) {
            if (sccThread->m_attachedSubStreams[i].streamId == -1)
                continue;

            frameTimeStamp = m_requestManager->GetTimestampByFrameCnt(m_reprocessingFrameCnt);
            m_requestManager->NotifyStreamOutput(m_reprocessingFrameCnt);
            if (m_currentReprocessOutStreams & (1 << sccThread->m_attachedSubStreams[i].streamId))
                m_runSubStreamFunc(sccThread, &(sccStreamParms->svcBuffers[checkingIndex]),
                    sccThread->m_attachedSubStreams[i].streamId, frameTimeStamp);
        }

        res = m_reprocessOps->release_buffer(m_reprocessOps, buf);
        if (res != NO_ERROR) {
            ALOGE("(%s): [reprocess] unable to release_buffer : %d", __FUNCTION__, res);
            return;
        }
        ALOGV("(%s): END   SIGNAL_STREAM_REPROCESSING_START", __FUNCTION__);
    }
    return;
}

int ExynosCameraHWInterface2::m_jpegCreator(StreamThread *selfThread, ExynosBuffer *srcImageBuf, nsecs_t frameTimeStamp)
{
    Mutex::Autolock lock(m_jpegEncoderLock);
    stream_parameters_t     *selfStreamParms = &(selfThread->m_parameters);
    substream_parameters_t  *subParms        = &m_subStreams[STREAM_ID_JPEG];
    status_t    res;
    bool found = false;
    int srcW, srcH, srcCropX, srcCropY;
    int pictureW, pictureH, pictureFramesize = 0;
    int pictureFormat;
    int cropX, cropY, cropW, cropH = 0;
    ExynosRect   m_jpegPictureRect;
    buffer_handle_t * buf = NULL;

    jpegEnc_parameters_t           jpegEncParms;

    if (subParms->numSvcBufsInHal == 0) {
        if (m_dequeueSubstreamBuffer(subParms, true) == 0) {
            ALOGE("(%s): No free svc buffer", __FUNCTION__);
            return 1;
        }
    }

    ALOGV("DEBUG(%s): current svcbuf index(%d), numBufsInHal(%d)", __FUNCTION__,
            subParms->svcBufIndex, subParms->numSvcBufsInHal);
    for (int i = 0 ; subParms->numSvcBuffers ; i++) {
        if (subParms->svcBufStatus[subParms->svcBufIndex] == ON_HAL) {
            found = true;
            break;
        }
        subParms->svcBufIndex++;
        if (subParms->svcBufIndex >= subParms->numSvcBuffers)
            subParms->svcBufIndex = 0;
    }
    if (!found) {
        ALOGE("(%s): cannot find free svc buffer", __FUNCTION__);
        subParms->svcBufIndex++;
        return 1;
    }

    ALOGV("DEBUG(%s): chosen svcbuf index(%d)", __FUNCTION__, subParms->svcBufIndex);

    m_getRatioSize(selfStreamParms->width, selfStreamParms->height,
                    m_streamThreads[0]->m_parameters.width, m_streamThreads[0]->m_parameters.height,
                    &srcCropX, &srcCropY,
                    &srcW, &srcH,
                    0);

    m_jpegPictureRect.w = subParms->width;
    m_jpegPictureRect.h = subParms->height;

    ALOGV("DEBUG(%s):w = %d, h = %d, w = %d, h = %d",
              __FUNCTION__, selfStreamParms->width, selfStreamParms->height,
                   m_jpegPictureRect.w, m_jpegPictureRect.h);

    m_getRatioSize(srcW, srcH,
                   m_jpegPictureRect.w, m_jpegPictureRect.h,
                   &cropX, &cropY,
                   &pictureW, &pictureH,
                   0);
    pictureFormat = V4L2_PIX_FMT_YUYV;
    pictureFramesize = FRAME_SIZE(V4L2_PIX_2_HAL_PIXEL_FORMAT(pictureFormat), pictureW, pictureH);

    m_getAlignedYUVSize(V4L2_PIX_FMT_YUYV, m_jpegPictureRect.w, m_jpegPictureRect.h, &m_resizeBuf);

    if (m_exynosPictureCSC) {
        float zoom_w = 0, zoom_h = 0;
        if (m_zoomRatio == 0)
            m_zoomRatio = 1;

        if (m_jpegPictureRect.w >= m_jpegPictureRect.h) {
            zoom_w =  pictureW / m_zoomRatio;
            zoom_h = zoom_w * m_jpegPictureRect.h / m_jpegPictureRect.w;
        } else {
            zoom_h = pictureH / m_zoomRatio;
            zoom_w = zoom_h * m_jpegPictureRect.w / m_jpegPictureRect.h;
        }
        cropX = (srcW - zoom_w) / 2;
        cropY = (srcH - zoom_h) / 2;
        cropW = zoom_w;
        cropH = zoom_h;

        ALOGV("DEBUG(%s):cropX = %d, cropY = %d, cropW = %d, cropH = %d",
              __FUNCTION__, cropX, cropY, cropW, cropH);

        csc_set_src_format(m_exynosPictureCSC,
                           ALIGN(srcW, 16), ALIGN(srcH, 16),
                           cropX, cropY, cropW, cropH,
                           V4L2_PIX_2_HAL_PIXEL_FORMAT(pictureFormat),
                           0);

        csc_set_dst_format(m_exynosPictureCSC,
                           m_jpegPictureRect.w, m_jpegPictureRect.h,
                           0, 0, m_jpegPictureRect.w, m_jpegPictureRect.h,
                           V4L2_PIX_2_HAL_PIXEL_FORMAT(V4L2_PIX_FMT_YUYV),
                           0);
        for (int i = 0 ; i < 3 ; i++)
            ALOGV("DEBUG(%s): m_pictureBuf.fd.extFd[%d]=%d ",
                __FUNCTION__, i, srcImageBuf->fd.extFd[i]);
        csc_set_src_buffer(m_exynosPictureCSC,
                           (void **)&srcImageBuf->fd.fd, CSC_MEMORY_DMABUF);

        csc_set_dst_buffer(m_exynosPictureCSC,
                           (void **)&m_resizeBuf.fd.fd, CSC_MEMORY_DMABUF);
        for (int i = 0 ; i < 3 ; i++)
            ALOGV("DEBUG(%s): m_resizeBuf.virt.extP[%d]=%d m_resizeBuf.size.extS[%d]=%d",
                __FUNCTION__, i, m_resizeBuf.fd.extFd[i], i, m_resizeBuf.size.extS[i]);

        if (csc_convert(m_exynosPictureCSC) != 0)
            ALOGE("ERR(%s): csc_convert() fail", __FUNCTION__);

        m_jpegPictureRect.colorFormat = V4L2_PIX_FMT_YUYV;

        for (int j = 0; j < 3; j++)
            ALOGV("DEBUG(%s): dest buf node  fd.extFd[%d]=%d size=%d virt=%x ",
                __FUNCTION__, j, subParms->svcBuffers[subParms->svcBufIndex].fd.extFd[j],
                (unsigned int)subParms->svcBuffers[subParms->svcBufIndex].size.extS[j],
                (unsigned int)subParms->svcBuffers[subParms->svcBufIndex].virt.extP[j]);

        m_setExifChangedAttribute(&mExifInfo, &m_jpegPictureRect, &m_jpegMetadata);

        jpegEncParms.m_yuvBuf = m_resizeBuf;
        jpegEncParms.m_jpegBuf = subParms->svcBuffers[subParms->svcBufIndex];
        jpegEncParms.m_rect = m_jpegPictureRect;
        memcpy(&jpegEncParms.m_exifInfo, &mExifInfo, sizeof(exif_attribute_t));
        jpegEncParms.m_jpegQuality = m_jpegMetadata.shot.ctl.jpeg.quality;
        jpegEncParms.m_thumbQuality = m_jpegMetadata.shot.ctl.jpeg.thumbnailQuality;
        jpegEncParms.m_frameTimeStamp = frameTimeStamp;

        jpegEncParms.m_svcBufIndex = subParms->svcBufIndex;

        m_jpegEncThread->enqueueEncoding(&jpegEncParms);
    } else {
        ALOGE("ERR(%s): m_exynosPictureCSC == NULL", __FUNCTION__);
    }


    m_dequeueSubstreamBuffer(subParms, false);

    return 0;
}

int ExynosCameraHWInterface2::m_recordCreator(StreamThread *selfThread, ExynosBuffer *srcImageBuf, nsecs_t frameTimeStamp)
{
    stream_parameters_t     *selfStreamParms = &(selfThread->m_parameters);
    substream_parameters_t  *subParms        = &m_subStreams[STREAM_ID_RECORD];
    status_t    res;
    bool found = false;
    buffer_handle_t * buf = NULL;

    ALOGV("DEBUG(%s): index(%d)",__FUNCTION__, subParms->svcBufIndex);
    for (int i = 0 ; subParms->numSvcBuffers ; i++) {
        if (subParms->svcBufStatus[subParms->svcBufIndex] == ON_HAL) {
            found = true;
            break;
        }
        subParms->svcBufIndex++;
        if (subParms->svcBufIndex >= subParms->numSvcBuffers)
            subParms->svcBufIndex = 0;
    }
    if (!found) {
        ALOGE("(%s): cannot find free svc buffer", __FUNCTION__);
        subParms->svcBufIndex++;
        return 1;
    }

    if (m_exynosVideoCSC) {
        int videoW = subParms->width, videoH = subParms->height;
        int cropX, cropY, cropW, cropH = 0;
        int previewW = selfStreamParms->width, previewH = selfStreamParms->height;
        m_getRatioSize(previewW, previewH,
                       videoW, videoH,
                       &cropX, &cropY,
                       &cropW, &cropH,
                       0);

        ALOGV("DEBUG(%s):cropX = %d, cropY = %d, cropW = %d, cropH = %d",
                 __FUNCTION__, cropX, cropY, cropW, cropH);

        csc_set_src_format(m_exynosVideoCSC,
                           ALIGN(previewW, 32), previewH,
                           cropX, cropY, cropW, cropH,
                           selfStreamParms->format,
                           0);

        csc_set_dst_format(m_exynosVideoCSC,
                           videoW, videoH,
                           0, 0, videoW, videoH,
                           subParms->format,
                           1);

        csc_set_src_buffer(m_exynosVideoCSC,
                        (void **)&srcImageBuf->fd.fd, CSC_MEMORY_DMABUF);

        csc_set_dst_buffer(m_exynosVideoCSC,
            (void **)(&(subParms->svcBuffers[subParms->svcBufIndex].fd.fd)), CSC_MEMORY_DMABUF);

        if (csc_convert(m_exynosVideoCSC) != 0) {
            ALOGE("ERR(%s):csc_convert() fail", __FUNCTION__);
        }
        else {
            ALOGV("(%s):csc_convert() SUCCESS", __FUNCTION__);
        }
    }
    else {
        ALOGE("ERR(%s):m_exynosVideoCSC == NULL", __FUNCTION__);
    }

    res = subParms->streamOps->enqueue_buffer(subParms->streamOps, frameTimeStamp, &(subParms->svcBufHandle[subParms->svcBufIndex]));

    ALOGV("DEBUG(%s): streamthread[%d] enqueue_buffer index(%d) to svc done res(%d)",
            __FUNCTION__, selfThread->m_index, subParms->svcBufIndex, res);
    if (res == 0) {
        subParms->svcBufStatus[subParms->svcBufIndex] = ON_SERVICE;
        subParms->numSvcBufsInHal--;
    }
    else {
        subParms->svcBufStatus[subParms->svcBufIndex] = ON_HAL;
    }
    m_dequeueSubstreamBuffer(subParms, false);
    return 0;
}

int ExynosCameraHWInterface2::m_prvcbCreator(StreamThread *selfThread, ExynosBuffer *srcImageBuf, nsecs_t frameTimeStamp)
{
    stream_parameters_t     *selfStreamParms = &(selfThread->m_parameters);
    substream_parameters_t  *subParms        = &m_subStreams[STREAM_ID_PRVCB];
    status_t    res;
    bool found = false;
    int cropX, cropY, cropW, cropH = 0;
    buffer_handle_t * buf = NULL;

    ALOGV("DEBUG(%s): index(%d)",__FUNCTION__, subParms->svcBufIndex);
    for (int i = 0 ; subParms->numSvcBuffers ; i++) {
        if (subParms->svcBufStatus[subParms->svcBufIndex] == ON_HAL) {
            found = true;
            break;
        }
        subParms->svcBufIndex++;
        if (subParms->svcBufIndex >= subParms->numSvcBuffers)
            subParms->svcBufIndex = 0;
    }
    if (!found) {
        ALOGE("(%s): cannot find free svc buffer", __FUNCTION__);
        subParms->svcBufIndex++;
        return 1;
    }

    if (subParms->format == HAL_PIXEL_FORMAT_YCrCb_420_SP) {
        if (m_exynosVideoCSC) {
            int previewCbW = subParms->width, previewCbH = subParms->height;
            int cropX, cropY, cropW, cropH = 0;
            int previewW = selfStreamParms->width, previewH = selfStreamParms->height;
            m_getRatioSize(previewW, previewH,
                           previewCbW, previewCbH,
                           &cropX, &cropY,
                           &cropW, &cropH,
                           0);

            ALOGV("DEBUG(%s):cropX = %d, cropY = %d, cropW = %d, cropH = %d",
                     __FUNCTION__, cropX, cropY, cropW, cropH);
            csc_set_src_format(m_exynosVideoCSC,
                               ALIGN(previewW, 32), previewH,
                               cropX, cropY, cropW, cropH,
                               selfStreamParms->format,
                               0);

            csc_set_dst_format(m_exynosVideoCSC,
                               previewCbW, previewCbH,
                               0, 0, previewCbW, previewCbH,
                               subParms->internalFormat,
                               1);

            csc_set_src_buffer(m_exynosVideoCSC,
                        (void **)&srcImageBuf->fd.fd, CSC_MEMORY_DMABUF);

            csc_set_dst_buffer(m_exynosVideoCSC,
                (void **)(&(m_previewCbBuf.fd.fd)), CSC_MEMORY_DMABUF);

            if (csc_convert(m_exynosVideoCSC) != 0) {
                ALOGE("ERR(%s):previewcb csc_convert() fail", __FUNCTION__);
            }
            else {
                ALOGV("(%s):previewcb csc_convert() SUCCESS", __FUNCTION__);
            }
            if (previewCbW == ALIGN(previewCbW, 16)) {
                memcpy(subParms->svcBuffers[subParms->svcBufIndex].virt.extP[0],
                    m_previewCbBuf.virt.extP[0], previewCbW * previewCbH);
                memcpy(subParms->svcBuffers[subParms->svcBufIndex].virt.extP[0] + previewCbW * previewCbH,
                    m_previewCbBuf.virt.extP[1], previewCbW * previewCbH / 2 );
            }
            else {
                // TODO : copy line by line ?
            }
        }
        else {
            ALOGE("ERR(%s):m_exynosVideoCSC == NULL", __FUNCTION__);
        }
    }
    else if (subParms->format == HAL_PIXEL_FORMAT_YV12) {
        int previewCbW = subParms->width, previewCbH = subParms->height;
        int stride = ALIGN(previewCbW, 16);
        int uv_stride = ALIGN(previewCbW/2, 16);
        int c_stride = ALIGN(stride / 2, 16);

        if (previewCbW == ALIGN(previewCbW, 32)) {
            memcpy(subParms->svcBuffers[subParms->svcBufIndex].virt.extP[0],
                srcImageBuf->virt.extP[0], stride * previewCbH);
            memcpy(subParms->svcBuffers[subParms->svcBufIndex].virt.extP[0] + stride * previewCbH,
                srcImageBuf->virt.extP[1], c_stride * previewCbH / 2 );
            memcpy(subParms->svcBuffers[subParms->svcBufIndex].virt.extP[0] + (stride * previewCbH) + (c_stride * previewCbH / 2),
                srcImageBuf->virt.extP[2], c_stride * previewCbH / 2 );
        } else {
            char * dstAddr = (char *)(subParms->svcBuffers[subParms->svcBufIndex].virt.extP[0]);
            char * srcAddr = (char *)(srcImageBuf->virt.extP[0]);
            for (int i = 0 ; i < previewCbH ; i++) {
                memcpy(dstAddr, srcAddr, previewCbW);
                dstAddr += stride;
                srcAddr += ALIGN(stride, 32);
            }
            dstAddr = (char *)(subParms->svcBuffers[subParms->svcBufIndex].virt.extP[0] + stride * previewCbH);
            srcAddr = (char *)(srcImageBuf->virt.extP[1]);
            for (int i = 0 ; i < previewCbH/2 ; i++) {
                memcpy(dstAddr, srcAddr, previewCbW/2);
                dstAddr += c_stride;
                srcAddr += uv_stride;
            }
            srcAddr = (char *)(srcImageBuf->virt.extP[2]);
            for (int i = 0 ; i < previewCbH/2 ; i++) {
                memcpy(dstAddr, srcAddr, previewCbW/2);
                dstAddr += c_stride;
                srcAddr += uv_stride;
            }
        }
    }
    res = subParms->streamOps->enqueue_buffer(subParms->streamOps, frameTimeStamp, &(subParms->svcBufHandle[subParms->svcBufIndex]));

    ALOGV("DEBUG(%s): streamthread[%d] enqueue_buffer index(%d) to svc done res(%d)",
            __FUNCTION__, selfThread->m_index, subParms->svcBufIndex, res);
    if (res == 0) {
        subParms->svcBufStatus[subParms->svcBufIndex] = ON_SERVICE;
        subParms->numSvcBufsInHal--;
    }
    else {
        subParms->svcBufStatus[subParms->svcBufIndex] = ON_HAL;
    }
    m_dequeueSubstreamBuffer(subParms, false);
    return 0;
}

bool ExynosCameraHWInterface2::dumpImage(struct ExynosBuffer *buffer, char *filename, int num, char *type)
{
    FILE *fd = NULL;
    struct timeval tv;
    struct tm tm;
    char filepath[64];
    char teePath[32];
    /* file creation */
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);
    strftime(teePath, sizeof(teePath), "%Y%m%d_%H%M%S", &tm);
    sprintf(filepath, "/data/%s_%s_%d.%s", filename, teePath, num, type);
    fd = fopen(filepath, "wb");
    if (fd == NULL) {
        ALOGE("dump image file open error");
        return false;
    }

    ALOGD("%s : addr = %#x, size = %ld", filepath, buffer->virt.extP[0], buffer->size.extS[0]);
    fflush(stdout);

    fwrite(buffer->virt.extP[0], 1, buffer->size.extS[0], fd);
    fflush(fd);

    if (fd)
        fclose(fd);

    return true;
}

int ExynosCameraHWInterface2::m_dequeueSubstreamBuffer(substream_parameters_t  *subParms, bool dequeueOnlyOne)
{
    bool found = false;
    int checkingIndex = 0;
    status_t    res;
    buffer_handle_t * buf = NULL;
    int dequeuedCount = 0;

    ALOGV("DEBUG(%s): START substream(%d) currentBuf#(%d) index(%d)", __FUNCTION__,
                subParms->type, subParms->numSvcBufsInHal, subParms->svcBufIndex);

    while (subParms->numSvcBufsInHal <= subParms->minUndequedBuffer) {
        res = subParms->streamOps->dequeue_buffer(subParms->streamOps, &buf);
        if (res != NO_ERROR || buf == NULL) {
            ALOGV("DEBUG(%s): substream(%d) dequeue_buffer fail res(%d)",__FUNCTION__ , subParms->type, res);
            break;
        }
        const private_handle_t *priv_handle = reinterpret_cast<const private_handle_t *>(*buf);
        subParms->numSvcBufsInHal++;
        ALOGV("DEBUG(%s): substream(%d) got buf(%x) numBufInHal(%d) version(%d), numFds(%d), numInts(%d)", __FUNCTION__,
                (uint32_t)(*buf), subParms->type, subParms->numSvcBufsInHal,
                ((native_handle_t*)(*buf))->version, ((native_handle_t*)(*buf))->numFds, ((native_handle_t*)(*buf))->numInts);

        for (checkingIndex = 0; checkingIndex < subParms->numSvcBuffers ; checkingIndex++) {
            if (priv_handle->fd == subParms->svcBuffers[checkingIndex].fd.extFd[0] ) {
                found = true;
                break;
            }
        }
        ALOGV("DEBUG(%s): substream(%d) dequeueed_buffer found(%d) index(%d)", __FUNCTION__, subParms->type, found, checkingIndex);

        if (!found)
             break;

        subParms->svcBufIndex = checkingIndex;
        if (subParms->svcBufStatus[subParms->svcBufIndex] == ON_SERVICE)
            subParms->svcBufStatus[subParms->svcBufIndex] = ON_HAL;
        else
            ALOGV("DEBUG(%s): substream(%d) bufstatus abnormal [%d]  status = %d", __FUNCTION__, subParms->type,
                subParms->svcBufIndex,  subParms->svcBufStatus[subParms->svcBufIndex]);
        dequeuedCount++;
        if (dequeueOnlyOne)
            break;
    }
    ALOGV("DEBUG(%s): END substream(%d) currentBuf#(%d) index(%d)", __FUNCTION__,
                subParms->type, subParms->numSvcBufsInHal, subParms->svcBufIndex);
    return dequeuedCount;
}

bool ExynosCameraHWInterface2::yuv2Jpeg(ExynosBuffer *yuvBuf, ExynosBuffer *jpegBuf,
                                        ExynosRect *rect, exif_attribute_t *exifInfo,
                                        int jpegQuality, int thumbQuality)
{
    ExynosJpegEncoderForCamera jpegEnc;
    bool ret = false;
    int res = 0;

    unsigned int *yuvSize = yuvBuf->size.extS;

    if (jpegEnc.create()) {
        ALOGE("ERR(%s):jpegEnc.create() fail", __FUNCTION__);
        goto jpeg_encode_done;
    }

    if (jpegEnc.setQuality(jpegQuality)) {
        ALOGE("ERR(%s):jpegEnc.setQuality() fail", __FUNCTION__);
        goto jpeg_encode_done;
    }

    if (jpegEnc.setSize(rect->w, rect->h)) {
        ALOGE("ERR(%s):jpegEnc.setSize() fail", __FUNCTION__);
        goto jpeg_encode_done;
    }
    ALOGV("%s : width = %d , height = %d\n", __FUNCTION__, rect->w, rect->h);

    if (jpegEnc.setColorFormat(rect->colorFormat)) {
        ALOGE("ERR(%s):jpegEnc.setColorFormat() fail", __FUNCTION__);
        goto jpeg_encode_done;
    }

    if (jpegEnc.setJpegFormat(V4L2_PIX_FMT_JPEG_422)) {
        ALOGE("ERR(%s):jpegEnc.setJpegFormat() fail", __FUNCTION__);
        goto jpeg_encode_done;
    }

    if (jpegEnc.setThumbnailSize(exifInfo->widthThumb, exifInfo->heightThumb)) {
        ALOGE("ERR(%s):jpegEnc.setThumbnailSize(%d, %d) fail", __FUNCTION__,
            exifInfo->widthThumb, exifInfo->heightThumb);
        goto jpeg_encode_done;
    }

    if (jpegEnc.setThumbnailQuality(thumbQuality)) {
        ALOGE("ERR(%s):jpegEnc.setThumbnailQuality fail", __FUNCTION__);
        goto jpeg_encode_done;
    }

    ALOGV("DEBUG(%s):calling jpegEnc.setInBuf() yuvSize(%d)", __FUNCTION__, *yuvSize);
    if (jpegEnc.setInBuf((int *)&(yuvBuf->fd.fd), &(yuvBuf->virt.p), (int *)yuvSize)) {
        ALOGE("ERR(%s):jpegEnc.setInBuf() fail", __FUNCTION__);
        goto jpeg_encode_done;
    }
    if (jpegEnc.setOutBuf(jpegBuf->fd.fd, jpegBuf->virt.p, jpegBuf->size.extS[0] + jpegBuf->size.extS[1] + jpegBuf->size.extS[2])) {
        ALOGE("ERR(%s):jpegEnc.setOutBuf() fail", __FUNCTION__);
        goto jpeg_encode_done;
    }

    if (jpegEnc.updateConfig()) {
        ALOGE("ERR(%s):jpegEnc.updateConfig() fail", __FUNCTION__);
        goto jpeg_encode_done;
    }

    if ((res = jpegEnc.encode((int *)&jpegBuf->size.s, exifInfo)) < 0 ) {
        ALOGE("ERR(%s):jpegEnc.encode() fail ret(%d)", __FUNCTION__, res);
        goto jpeg_encode_done;
    }

    ret = true;

jpeg_encode_done:

    if (jpegEnc.flagCreate() == true)
        jpegEnc.destroy();

    return ret;
}

void ExynosCameraHWInterface2::OnFlashListenerISP(struct camera2_shot_ext *shot_ext)
{
    int nextState = FLASH_STATE_NO_TRANSITION;

    Mutex::Autolock lock(m_TriggerLock);

    /* Flash */
    switch (m_ctlInfo.flash.m_flashState) {
    case FLASH_STATE_ON_WAIT:
        if (shot_ext->shot.dm.flash.decision > 0) {
            /* store decision result to skip capture sequenece */
            ALOGV("(%s): [Flash] IS_FLASH_ON, decision - %d", __FUNCTION__, shot_ext->shot.dm.flash.decision);
            if (shot_ext->shot.dm.flash.decision == 2)
                m_ctlInfo.flash.m_flashDecisionResult = false;
            else
                m_ctlInfo.flash.m_flashDecisionResult = true;
            nextState = FLASH_STATE_ON_DONE;
        } else {
            if (m_ctlInfo.flash.m_flashTimeOut == 0) {
                ALOGV("(%s): [Flash] Timeout IS_FLASH_ON, decision is false setting", __FUNCTION__);
                nextState = FLASH_STATE_ON_DONE;
                m_ctlInfo.flash.m_flashDecisionResult = false;
            } else {
                m_ctlInfo.flash.m_flashTimeOut--;
            }
        }
        break;
    case FLASH_STATE_AE_AWB_LOCK_WAIT:
        if (shot_ext->shot.dm.aa.awbMode == AA_AWBMODE_LOCKED) {
            ALOGV("(%s): [Flash] FLASH_AUTO_AE_AWB_LOCK_WAIT - %d", __FUNCTION__, shot_ext->shot.dm.aa.awbMode);
            nextState = FLASH_STATE_AUTO_WAIT;
        } else {
            ALOGV("(%s):  [Flash] Waiting : AA_AWBMODE_LOCKED", __FUNCTION__);
        }
        break;
    case FLASH_STATE_CAPTURE_WAIT:
        if (m_ctlInfo.flash.m_flashDecisionResult) {
            if (shot_ext->shot.dm.flash.firingStable) {
                nextState = FLASH_STATE_CAPTURE_JPEG;
            } else {
                if (m_ctlInfo.flash.m_flashTimeOut == 0) {
                    ALOGE("(%s): [Flash] Wait firingStable time-out!!", __FUNCTION__);
                    nextState = FLASH_STATE_CAPTURE_JPEG;
                } else {
                    ALOGV("(%s): [Flash] Wait firingStable - %d", __FUNCTION__, m_ctlInfo.flash.m_flashTimeOut);
                    m_ctlInfo.flash.m_flashTimeOut--;
                }
            }
        } else {
            nextState = FLASH_STATE_CAPTURE_JPEG;
        }
        break;
    }

    FLASH_LOG("(%s): [%s] => [%s]", __FUNCTION__,
                                HAL_FlashState_Name[m_ctlInfo.flash.m_flashState],
                                HAL_FlashState_Name[nextState]);

    if (nextState != FLASH_STATE_NO_TRANSITION)
        m_ctlInfo.flash.m_flashState = nextState;
}

void ExynosCameraHWInterface2::OnPrecaptureMeteringTriggerStart(int id)
{
    m_ctlInfo.flash.m_precaptureTriggerId = id;
    m_ctlInfo.ae.aeStateNoti = AE_STATE_INACTIVE;

    if ((m_ctlInfo.flash.i_flashMode >= AA_AEMODE_ON_AUTO_FLASH) && (m_cameraId == 0)) {
        switch (m_ctlInfo.flash.m_flashState) {
        case FLASH_STATE_AUTO_DONE:
        case FLASH_STATE_AUTO_OFF:
            // Flash capture sequence, AF flash was executed before
            break;
        default:
            // Full flash sequence
            m_ctlInfo.flash.m_flashState = FLASH_STATE_ON;
            m_ctlInfo.flash.m_flashEnableFlg = true;
            m_ctlInfo.flash.m_flashTimeOut = 0;
            break;
        }
    } else {
        // Skip pre-capture in case of non-flash.
        ALOGV("[PreCap] Flash OFF mode ");
        m_ctlInfo.flash.m_flashEnableFlg = false;
        m_ctlInfo.flash.m_flashState = FLASH_STATE_NONE;
    }

    m_notifyCb(CAMERA2_MSG_AUTOEXPOSURE,
                ANDROID_CONTROL_AE_STATE_PRECAPTURE,
                m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);

    m_notifyCb(CAMERA2_MSG_AUTOWB,
                ANDROID_CONTROL_AWB_STATE_CONVERGED,
                m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);

    m_ctlInfo.ae.aeStateNoti = AE_STATE_PRECAPTURE;
}

void ExynosCameraHWInterface2::OnAfTrigger(int id)
{
    m_afTriggerId = id;

    switch (m_afMode) {
        case AA_AFMODE_AUTO:
        case AA_AFMODE_MACRO:
        case AA_AFMODE_MANUAL:
            OnAfTriggerAutoMacro();
            break;
        case AA_AFMODE_CONTINUOUS_VIDEO:
        case AA_AFMODE_CONTINUOUS_PICTURE:
            OnAfTriggerCAF();
            break;
        case AA_AFMODE_OFF:
        default:
            break;
    }
}

void ExynosCameraHWInterface2::OnAfTriggerAutoMacro(void)
{
    int nextState = NO_TRANSITION;

    /* If flash is enable, Flash operation is executed before triggering AF */
    if ((m_ctlInfo.flash.i_flashMode >= AA_AEMODE_ON_AUTO_FLASH) &&
        (m_ctlInfo.flash.m_flashEnableFlg == false) &&
        (m_cameraId == 0)) {
        m_ctlInfo.flash.m_flashEnableFlg = true;
        m_ctlInfo.flash.m_flashState = FLASH_STATE_ON;
        m_ctlInfo.flash.m_flashDecisionResult = false;
        m_ctlInfo.flash.m_afFlashFired = true;
    }

    switch (m_afState) {
        case HAL_AFSTATE_INACTIVE:
        case HAL_AFSTATE_SCANNING:
        case HAL_AFSTATE_FOCUSED:
        case HAL_AFSTATE_FAILED:
            nextState = HAL_AFSTATE_TRIGGERED;
            break;

        default:
            break;
    }

    if (nextState != NO_TRANSITION) {
        AF_LOG("(%s): [%s] => [%s]", __FUNCTION__,
                HAL_AFState_Name[m_afState], HAL_AFState_Name[nextState]);
        m_afState = nextState;
    }
}

void ExynosCameraHWInterface2::OnAfTriggerCAF(void)
{
    int nextState = NO_TRANSITION;

    switch (m_afState) {
    case HAL_AFSTATE_INACTIVE:
        nextState = NO_TRANSITION;
        SetAfStateForService(ANDROID_CONTROL_AF_STATE_NOT_FOCUSED_LOCKED);
        break;
    case HAL_AFSTATE_STARTED:
        nextState = HAL_AFSTATE_TRIGGERED;
        break;
    case HAL_AFSTATE_SCANNING:
        nextState = HAL_AFSTATE_TRIGGERED;

        // If flash is enable, Flash operation is executed before triggering AF
        if ((m_ctlInfo.flash.i_flashMode >= AA_AEMODE_ON_AUTO_FLASH)
                && (m_ctlInfo.flash.m_flashEnableFlg == false)
                && (m_cameraId == 0)) {
            ALOGV("[AF Flash] AF Flash start with Mode (%d) state (%d) id (%d)", m_afMode, m_afState, m_cameraId);
            m_ctlInfo.flash.m_flashEnableFlg = true;
            m_ctlInfo.flash.m_flashState = FLASH_STATE_ON;
            m_ctlInfo.flash.m_flashDecisionResult = false;
            m_ctlInfo.flash.m_afFlashFired = true;
        }
        break;
    case HAL_AFSTATE_FOCUSED:
    case HAL_AFSTATE_FAILED:
        m_IsAfLockRequired = true;
        if(m_afState == HAL_AFSTATE_FAILED) {
            SetAfStateForService(ANDROID_CONTROL_AF_STATE_NOT_FOCUSED_LOCKED);
            nextState = NO_TRANSITION;
        } else {
            SetAfStateForService(ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED);
            nextState = NO_TRANSITION;
        }
        break;
    default:
        break;
    }

    if (nextState != NO_TRANSITION) {
        AF_LOG("(%s): [%s] => [%s]", __FUNCTION__,
                HAL_AFState_Name[m_afState], HAL_AFState_Name[nextState]);
        m_afState = nextState;
    }
}

void ExynosCameraHWInterface2::OnPrecaptureMeteringNotificationISP()
{
    Mutex::Autolock lock(m_TriggerLock);

    if (m_ctlInfo.flash.m_precaptureTriggerId > 0) {
        if (m_ctlInfo.flash.m_flashEnableFlg) {
            // flash case
            switch (m_ctlInfo.flash.m_flashState) {
            case FLASH_STATE_AUTO_DONE:
            case FLASH_STATE_AUTO_OFF:
                if (m_ctlInfo.ae.aeStateNoti == AE_STATE_PRECAPTURE) {
                    // End notification
                    m_notifyCb(CAMERA2_MSG_AUTOEXPOSURE,
                                    ANDROID_CONTROL_AE_STATE_CONVERGED,
                                    m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);
                    ALOGD("(%s) ANDROID_CONTROL_AE_STATE_CONVERGED (%d)", __FUNCTION__, m_ctlInfo.flash.m_flashState);
                    m_notifyCb(CAMERA2_MSG_AUTOWB,
                                    ANDROID_CONTROL_AWB_STATE_CONVERGED,
                                    m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);
                    m_ctlInfo.flash.m_precaptureTriggerId = 0;
                } else {
                    m_notifyCb(CAMERA2_MSG_AUTOEXPOSURE,
                                    ANDROID_CONTROL_AE_STATE_PRECAPTURE,
                                    m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);
                    ALOGD("(%s) ANDROID_CONTROL_AE_STATE_PRECAPTURE (%d)", __FUNCTION__, m_ctlInfo.flash.m_flashState);
                    m_notifyCb(CAMERA2_MSG_AUTOWB,
                                    ANDROID_CONTROL_AWB_STATE_CONVERGED,
                                    m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);
                    m_ctlInfo.ae.aeStateNoti = AE_STATE_PRECAPTURE;
                }
                break;
            case FLASH_STATE_CAPTURE:
            case FLASH_STATE_CAPTURE_WAIT:
            case FLASH_STATE_CAPTURE_JPEG:
            case FLASH_STATE_CAPTURE_END:
                ALOGE("(%s) INVALID flash state count. (%d)", __FUNCTION__, (int)m_ctlInfo.flash.m_flashState);
                m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_DONE;
                m_notifyCb(CAMERA2_MSG_AUTOEXPOSURE,
                        ANDROID_CONTROL_AE_STATE_CONVERGED,
                        m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);
                m_notifyCb(CAMERA2_MSG_AUTOWB,
                        ANDROID_CONTROL_AWB_STATE_CONVERGED,
                        m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);
                m_ctlInfo.flash.m_precaptureTriggerId = 0;
                break;
            }
        } else {
            // non-flash case
            if (m_ctlInfo.ae.aeStateNoti == AE_STATE_PRECAPTURE) {
                m_notifyCb(CAMERA2_MSG_AUTOEXPOSURE,
                                ANDROID_CONTROL_AE_STATE_CONVERGED,
                                m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);
                ALOGD("(%s) ANDROID_CONTROL_AE_STATE_CONVERGED (%d)", __FUNCTION__, m_ctlInfo.flash.m_flashState);
                m_notifyCb(CAMERA2_MSG_AUTOWB,
                                ANDROID_CONTROL_AWB_STATE_CONVERGED,
                                m_ctlInfo.flash.m_precaptureTriggerId, 0, m_callbackCookie);
                m_ctlInfo.flash.m_precaptureTriggerId = 0;
            }
        }
    }
}

void ExynosCameraHWInterface2::OnAfNotification(enum aa_afstate noti)
{
    Mutex::Autolock lock(m_TriggerLock);

    switch (m_afMode) {
    case AA_AFMODE_AUTO:
    case AA_AFMODE_MACRO:
        OnAfNotificationAutoMacro(noti);
        break;
    case AA_AFMODE_CONTINUOUS_VIDEO:
    case AA_AFMODE_CONTINUOUS_PICTURE:
        OnAfNotificationCAF(noti);
        break;
    default:
        break;
    }
}

void ExynosCameraHWInterface2::OnAfNotificationAutoMacro(enum aa_afstate noti)
{
    int nextState = NO_TRANSITION;
    bool bWrongTransition = false;

    if (m_afState == HAL_AFSTATE_FOCUSED ||
        m_afState == HAL_AFSTATE_FAILED ||
        m_afState == HAL_AFSTATE_INACTIVE ||
        m_afState == HAL_AFSTATE_TRIGGERED) {
        AF_LOG("(%s): SKIP State [%s] ", __FUNCTION__, HAL_AFState_Name[m_afState]);
        return;
    }

    if (m_afState == HAL_AFSTATE_STARTED) {
        switch (noti) {
        case AA_AFSTATE_INACTIVE:
            nextState = NO_TRANSITION;
            break;
        case AA_AFSTATE_ACTIVE_SCAN:
            nextState = HAL_AFSTATE_SCANNING;
            SetAfStateForService(ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN);
            break;
        case AA_AFSTATE_AF_ACQUIRED_FOCUS:
            nextState = NO_TRANSITION;
            break;
        case AA_AFSTATE_AF_FAILED_FOCUS:
            nextState = NO_TRANSITION;
            break;
        default:
            bWrongTransition = true;
            break;
        }
    } else if (m_afState == HAL_AFSTATE_SCANNING) {
        switch (noti) {
        case AA_AFSTATE_INACTIVE:
            bWrongTransition = true;
            break;
        case AA_AFSTATE_ACTIVE_SCAN:
            nextState = NO_TRANSITION;
            break;
        case AA_AFSTATE_AF_ACQUIRED_FOCUS:
            // If Flash mode is enable, after AF execute pre-capture metering
            if (m_ctlInfo.flash.m_flashEnableFlg && m_ctlInfo.flash.m_afFlashFired) {
                switch (m_ctlInfo.flash.m_flashState) {
                case FLASH_STATE_ON_DONE:
                    m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_AE_AWB_LOCK;
                    nextState = NO_TRANSITION;
                    break;
                case FLASH_STATE_AUTO_DONE:
                    m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_OFF;
                    nextState = HAL_AFSTATE_FOCUSED;
                    SetAfStateForService(ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED);
                    break;
                default:
                    nextState = NO_TRANSITION;
                }
            } else {
                nextState = HAL_AFSTATE_FOCUSED;
                SetAfStateForService(ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED);
            }
            break;
        case AA_AFSTATE_AF_FAILED_FOCUS:
            // If Flash mode is enable, after AF execute pre-capture metering
            if (m_ctlInfo.flash.m_flashEnableFlg && m_ctlInfo.flash.m_afFlashFired) {
                switch (m_ctlInfo.flash.m_flashState) {
                case FLASH_STATE_ON_DONE:
                    m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_AE_AWB_LOCK;
                    nextState = NO_TRANSITION;
                    break;
                case FLASH_STATE_AUTO_DONE:
                    m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_OFF;
                    nextState = HAL_AFSTATE_FAILED;
                    SetAfStateForService(ANDROID_CONTROL_AF_STATE_NOT_FOCUSED_LOCKED);
                    break;
                default:
                    nextState = NO_TRANSITION;
                }
            } else {
                nextState = HAL_AFSTATE_FAILED;
                SetAfStateForService(ANDROID_CONTROL_AF_STATE_NOT_FOCUSED_LOCKED);
            }
            break;
        default:
            bWrongTransition = true;
            break;
        }
    }

    if (bWrongTransition) {
        ALOGE("(%s): Wrong Transition state[%d] noti(%d)", __FUNCTION__, m_afState, noti);
        return;
    }

    if (nextState != NO_TRANSITION) {
        AF_LOG("(%s): [%s] => [%s] : Noti(%d)", __FUNCTION__,
                HAL_AFState_Name[m_afState], HAL_AFState_Name[nextState], noti);
        m_afState = nextState;
    }
}

void ExynosCameraHWInterface2::OnAfNotificationCAF(enum aa_afstate noti)
{
    int nextState = NO_TRANSITION;
    bool bWrongTransition = false;

    AF_LOGV("(%s): State [%s], Noti [%d]", __FUNCTION__, HAL_AFState_Name[m_afState], noti);
    if (m_afState == HAL_AFSTATE_INACTIVE) {
        AF_LOG("(%s): SKIP State [%s] ", __FUNCTION__, HAL_AFState_Name[m_afState]);
        return;
    }

    if (m_afState == HAL_AFSTATE_STARTED) {
        switch (noti) {
        case AA_AFSTATE_ACTIVE_SCAN:
            nextState = HAL_AFSTATE_SCANNING;
            SetAfStateForService(ANDROID_CONTROL_AF_STATE_PASSIVE_SCAN);
            break;
        case AA_AFSTATE_INACTIVE:
        case AA_AFSTATE_AF_ACQUIRED_FOCUS:
        case AA_AFSTATE_AF_FAILED_FOCUS:
            nextState = NO_TRANSITION;
            break;
        default:
            bWrongTransition = true;
            break;
        }
    } else if (m_afState == HAL_AFSTATE_SCANNING) {
        switch (noti) {
        case AA_AFSTATE_INACTIVE:
        case AA_AFSTATE_ACTIVE_SCAN:
            nextState = NO_TRANSITION;
            break;
        case AA_AFSTATE_AF_ACQUIRED_FOCUS:
            nextState = HAL_AFSTATE_FOCUSED;
            SetAfStateForService(ANDROID_CONTROL_AF_STATE_PASSIVE_FOCUSED);
            break;
        case AA_AFSTATE_AF_FAILED_FOCUS:
            nextState = HAL_AFSTATE_FAILED;
            SetAfStateForService(ANDROID_CONTROL_AF_STATE_PASSIVE_FOCUSED);
            break;
        default:
            bWrongTransition = true;
            break;
        }
    } else if (m_afState == HAL_AFSTATE_FOCUSED || m_afState == HAL_AFSTATE_FAILED) {
        switch (noti) {
        case AA_AFSTATE_INACTIVE:
        case AA_AFSTATE_AF_ACQUIRED_FOCUS:
        case AA_AFSTATE_AF_FAILED_FOCUS:
            nextState = NO_TRANSITION;
            break;
        case AA_AFSTATE_ACTIVE_SCAN:
            nextState = HAL_AFSTATE_SCANNING;
            SetAfStateForService(ANDROID_CONTROL_AF_STATE_PASSIVE_SCAN);
            break;
        default:
            bWrongTransition = true;
            break;
        }
    } else if (m_afState == HAL_AFSTATE_TRIGGERED) {
        //Skip notification in case of flash, wait the end of flash on
        if (m_ctlInfo.flash.m_flashEnableFlg && m_ctlInfo.flash.m_afFlashFired) {
            if (m_ctlInfo.flash.m_flashState < FLASH_STATE_ON_DONE)
                return;
        }
        switch (noti) {
        case AA_AFSTATE_INACTIVE:
        case AA_AFSTATE_ACTIVE_SCAN:
            nextState = NO_TRANSITION;
            break;
        case AA_AFSTATE_AF_ACQUIRED_FOCUS:
            // If Flash mode is enable, after AF execute pre-capture metering
            if (m_ctlInfo.flash.m_flashEnableFlg && m_ctlInfo.flash.m_afFlashFired) {
                switch (m_ctlInfo.flash.m_flashState) {
                case FLASH_STATE_ON_DONE:
                    m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_AE_AWB_LOCK;
                    nextState = NO_TRANSITION;
                    break;
                case FLASH_STATE_AUTO_DONE:
                    m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_OFF;
                    m_IsAfLockRequired = true;
                    nextState = HAL_AFSTATE_FOCUSED;
                    SetAfStateForService(ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED);
                    break;
                default:
                    nextState = NO_TRANSITION;
                }
            } else {
                m_IsAfLockRequired = true;
                nextState = HAL_AFSTATE_FOCUSED;
                SetAfStateForService(ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED);
            }
            break;
        case AA_AFSTATE_AF_FAILED_FOCUS:
            // If Flash mode is enable, after AF execute pre-capture metering
            if (m_ctlInfo.flash.m_flashEnableFlg && m_ctlInfo.flash.m_afFlashFired) {
                switch (m_ctlInfo.flash.m_flashState) {
                case FLASH_STATE_ON_DONE:
                    m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_AE_AWB_LOCK;
                    nextState = NO_TRANSITION;
                    break;
                case FLASH_STATE_AUTO_DONE:
                    m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_OFF;
                    m_IsAfLockRequired = true;
                    nextState = HAL_AFSTATE_FAILED;
                    SetAfStateForService(ANDROID_CONTROL_AF_STATE_NOT_FOCUSED_LOCKED);
                    break;
                default:
                    nextState = NO_TRANSITION;
                }
            } else {
                m_IsAfLockRequired = true;
                nextState = HAL_AFSTATE_FAILED;
                SetAfStateForService(ANDROID_CONTROL_AF_STATE_NOT_FOCUSED_LOCKED);
            }
            break;
        default:
            bWrongTransition = true;
            break;
        }
    }

    if (bWrongTransition) {
        ALOGE("(%s): Wrong Transition state[%d] noti(%d) mode (%d)", __FUNCTION__, m_afState, noti, m_afMode);
        return;
    }

    if (nextState != NO_TRANSITION) {
        AF_LOG("(%s): [%s] => [%s] : Noti(%d)", __FUNCTION__,
                HAL_AFState_Name[m_afState], HAL_AFState_Name[nextState], noti);
        m_afState = nextState;
    }
}

void ExynosCameraHWInterface2::OnAfCancel(int id)
{
    int nextState = NO_TRANSITION;
    enum aa_afmode previous_af_mode;

    m_afTriggerId = id;

    if (m_ctlInfo.flash.m_flashEnableFlg  && m_ctlInfo.flash.m_afFlashFired)
        m_ctlInfo.flash.m_flashState = FLASH_STATE_AUTO_OFF;

    switch (m_afState) {
    case HAL_AFSTATE_INACTIVE:
        nextState = NO_TRANSITION;
        if(m_afMode == AA_AFMODE_AUTO || m_afMode == AA_AFMODE_MACRO ||
            m_afMode == AA_AFMODE_MANUAL || m_afMode == AA_AFMODE_OFF)
            SetAfStateForService(ANDROID_CONTROL_AF_STATE_INACTIVE);
        break;
    case HAL_AFSTATE_TRIGGERED:
    case HAL_AFSTATE_STARTED:
    case HAL_AFSTATE_SCANNING:
    case HAL_AFSTATE_FOCUSED:
    case HAL_AFSTATE_FAILED:
        previous_af_mode = m_afMode;
        SetAfMode(AA_AFMODE_OFF);

        if(AF_MODE_CAF(previous_af_mode))
            SetAfMode(previous_af_mode);
        break;
    default:
        break;
    }

    if (nextState != NO_TRANSITION) {
        AF_LOG("(%s): [%s] => [%s]", __FUNCTION__,
                HAL_AFState_Name[m_afState], HAL_AFState_Name[nextState]);
        m_afState = nextState;
    }
}

void ExynosCameraHWInterface2::SetAfStateForService(int newState)
{
    if (m_serviceAfState != newState || newState == ANDROID_CONTROL_AF_STATE_INACTIVE)
        m_notifyCb(CAMERA2_MSG_AUTOFOCUS, newState, m_afTriggerId, 0, m_callbackCookie);
    m_serviceAfState = newState;
}

int ExynosCameraHWInterface2::GetAfStateForService(void)
{
   return m_serviceAfState;
}

void ExynosCameraHWInterface2::SetAfMode(enum aa_afmode afMode)
{
    if (m_afMode != afMode) {
        if (m_IsAfModeUpdateRequired && m_afMode != AA_AFMODE_OFF) {
            m_afMode2 = afMode;
            ALOGD("(%s): Pending(%d) => New(%d)", __FUNCTION__, m_afMode, afMode);
        } else {
            m_IsAfModeUpdateRequired = true;
            m_afMode = afMode;
            SetAfStateForService(ANDROID_CONTROL_AF_STATE_INACTIVE);
            m_afState = HAL_AFSTATE_INACTIVE;

            AF_LOG("(%s): Set AF Mode[%d], State [%s]",
                    __FUNCTION__, m_afMode, HAL_AFState_Name[m_afState]);
        }
    }
}

void ExynosCameraHWInterface2::m_setExifFixedAttribute(void)
{
    char property[PROPERTY_VALUE_MAX];

    //2 0th IFD TIFF Tags
    //3 Maker
    property_get("ro.product.brand", property, EXIF_DEF_MAKER);
    strncpy((char *)mExifInfo.maker, property,
                sizeof(mExifInfo.maker) - 1);
    mExifInfo.maker[sizeof(mExifInfo.maker) - 1] = '\0';
    //3 Model
    property_get("ro.product.model", property, EXIF_DEF_MODEL);
    strncpy((char *)mExifInfo.model, property,
                sizeof(mExifInfo.model) - 1);
    mExifInfo.model[sizeof(mExifInfo.model) - 1] = '\0';
    //3 Software
    property_get("ro.build.id", property, EXIF_DEF_SOFTWARE);
    strncpy((char *)mExifInfo.software, property,
                sizeof(mExifInfo.software) - 1);
    mExifInfo.software[sizeof(mExifInfo.software) - 1] = '\0';

    //3 YCbCr Positioning
    mExifInfo.ycbcr_positioning = EXIF_DEF_YCBCR_POSITIONING;

    //2 0th IFD Exif Private Tags
    //3 F Number
    mExifInfo.fnumber.num = (uint32_t)(m_camera2->m_curCameraInfo->fnumber * EXIF_DEF_FNUMBER_DEN);
    mExifInfo.fnumber.den = EXIF_DEF_FNUMBER_DEN;
    //3 Exposure Program
    mExifInfo.exposure_program = EXIF_DEF_EXPOSURE_PROGRAM;
    //3 Exif Version
    memcpy(mExifInfo.exif_version, EXIF_DEF_EXIF_VERSION, sizeof(mExifInfo.exif_version));
    //3 Aperture
    double av = APEX_FNUM_TO_APERTURE((double)mExifInfo.fnumber.num/mExifInfo.fnumber.den);
    mExifInfo.aperture.num = (uint32_t)(av*EXIF_DEF_APEX_DEN);
    mExifInfo.aperture.den = EXIF_DEF_APEX_DEN;
    //3 Maximum lens aperture
    mExifInfo.max_aperture.num = mExifInfo.aperture.num;
    mExifInfo.max_aperture.den = mExifInfo.aperture.den;
    //3 Lens Focal Length
    mExifInfo.focal_length.num = (uint32_t)(m_camera2->m_curCameraInfo->focalLength * 100);

    mExifInfo.focal_length.den = EXIF_DEF_FOCAL_LEN_DEN;
    //3 User Comments
    strcpy((char *)mExifInfo.user_comment, EXIF_DEF_USERCOMMENTS);
    //3 Color Space information
    mExifInfo.color_space = EXIF_DEF_COLOR_SPACE;
    //3 Exposure Mode
    mExifInfo.exposure_mode = EXIF_DEF_EXPOSURE_MODE;

    //2 0th IFD GPS Info Tags
    unsigned char gps_version[4] = { 0x02, 0x02, 0x00, 0x00 };
    memcpy(mExifInfo.gps_version_id, gps_version, sizeof(gps_version));

    //2 1th IFD TIFF Tags
    mExifInfo.compression_scheme = EXIF_DEF_COMPRESSION;
    mExifInfo.x_resolution.num = EXIF_DEF_RESOLUTION_NUM;
    mExifInfo.x_resolution.den = EXIF_DEF_RESOLUTION_DEN;
    mExifInfo.y_resolution.num = EXIF_DEF_RESOLUTION_NUM;
    mExifInfo.y_resolution.den = EXIF_DEF_RESOLUTION_DEN;
    mExifInfo.resolution_unit = EXIF_DEF_RESOLUTION_UNIT;
}

void ExynosCameraHWInterface2::m_setExifChangedAttribute(exif_attribute_t *exifInfo, ExynosRect *rect,
	camera2_shot_ext *currentEntry)
{
    camera2_dm *dm = &(currentEntry->shot.dm);
    camera2_ctl *ctl = &(currentEntry->shot.ctl);

    ALOGV("(%s): framecnt(%d) exp(%lld) iso(%d)", __FUNCTION__, ctl->request.frameCount, dm->sensor.exposureTime,dm->aa.isoValue );
    if (!ctl->request.frameCount)
       return;
    //2 0th IFD TIFF Tags
    //3 Width
    exifInfo->width = rect->w;
    //3 Height
    exifInfo->height = rect->h;
    //3 Orientation
    switch (ctl->jpeg.orientation) {
    case 90:
        exifInfo->orientation = EXIF_ORIENTATION_90;
        break;
    case 180:
        exifInfo->orientation = EXIF_ORIENTATION_180;
        break;
    case 270:
        exifInfo->orientation = EXIF_ORIENTATION_270;
        break;
    case 0:
    default:
        exifInfo->orientation = EXIF_ORIENTATION_UP;
        break;
    }

    //3 Date time
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime((char *)exifInfo->date_time, 20, "%Y:%m:%d %H:%M:%S", timeinfo);

    //2 0th IFD Exif Private Tags
    //3 Exposure Time
    int shutterSpeed = (dm->sensor.exposureTime/1000);

    // To display exposure time just above 500ms as 1/2sec, not 1 sec.
    if (shutterSpeed > 500000)
        shutterSpeed -=  100000;

    if (shutterSpeed < 0) {
        shutterSpeed = 100;
    }

    exifInfo->exposure_time.num = 1;
    // x us -> 1/x s */
    //exifInfo->exposure_time.den = (uint32_t)(1000000 / shutterSpeed);
    exifInfo->exposure_time.den = (uint32_t)((double)1000000 / shutterSpeed);

    //3 ISO Speed Rating
    exifInfo->iso_speed_rating = dm->aa.isoValue;

    uint32_t av, tv, bv, sv, ev;
    av = APEX_FNUM_TO_APERTURE((double)exifInfo->fnumber.num / exifInfo->fnumber.den);
    tv = APEX_EXPOSURE_TO_SHUTTER((double)exifInfo->exposure_time.num / exifInfo->exposure_time.den);
    sv = APEX_ISO_TO_FILMSENSITIVITY(exifInfo->iso_speed_rating);
    bv = av + tv - sv;
    ev = av + tv;
    //ALOGD("Shutter speed=%d us, iso=%d", shutterSpeed, exifInfo->iso_speed_rating);
    ALOGV("AV=%d, TV=%d, SV=%d", av, tv, sv);

    //3 Shutter Speed
    exifInfo->shutter_speed.num = tv * EXIF_DEF_APEX_DEN;
    exifInfo->shutter_speed.den = EXIF_DEF_APEX_DEN;
    //3 Brightness
    exifInfo->brightness.num = bv*EXIF_DEF_APEX_DEN;
    exifInfo->brightness.den = EXIF_DEF_APEX_DEN;
    //3 Exposure Bias
    if (ctl->aa.sceneMode== AA_SCENE_MODE_BEACH||
        ctl->aa.sceneMode== AA_SCENE_MODE_SNOW) {
        exifInfo->exposure_bias.num = EXIF_DEF_APEX_DEN;
        exifInfo->exposure_bias.den = EXIF_DEF_APEX_DEN;
    } else {
        exifInfo->exposure_bias.num = 0;
        exifInfo->exposure_bias.den = 0;
    }
    //3 Metering Mode
    switch (m_ctlInfo.ae.aeMeteringMode) {
    case AA_AEMODE_METERING_NONE:
    case AA_AEMODE_CENTER:
        exifInfo->metering_mode = EXIF_METERING_CENTER;
        break;
    case AA_AEMODE_AVERAGE:
        exifInfo->metering_mode = EXIF_METERING_AVERAGE;
        break;
    case AA_AEMODE_MATRIX:
        exifInfo->metering_mode = EXIF_METERING_AVERAGE;
        break;
    case AA_AEMODE_SPOT:
        exifInfo->metering_mode = EXIF_METERING_SPOT;
        break;
    default:
        exifInfo->metering_mode = EXIF_METERING_CENTER;
        break;
    }

    //3 Flash
    if (m_ctlInfo.flash.m_flashDecisionResult)
        exifInfo->flash = 1;
    else
        exifInfo->flash = EXIF_DEF_FLASH;

    //3 White Balance
    if (currentEntry->awb_mode_dm == AA_AWBMODE_WB_AUTO)
        exifInfo->white_balance = EXIF_WB_AUTO;
    else
        exifInfo->white_balance = EXIF_WB_MANUAL;

    //3 Scene Capture Type
    switch (ctl->aa.sceneMode) {
    case AA_SCENE_MODE_PORTRAIT:
        exifInfo->scene_capture_type = EXIF_SCENE_PORTRAIT;
        break;
    case AA_SCENE_MODE_LANDSCAPE:
        exifInfo->scene_capture_type = EXIF_SCENE_LANDSCAPE;
        break;
    case AA_SCENE_MODE_NIGHT_PORTRAIT:
        exifInfo->scene_capture_type = EXIF_SCENE_NIGHT;
        break;
    default:
        exifInfo->scene_capture_type = EXIF_SCENE_STANDARD;
        break;
    }

    //2 0th IFD GPS Info Tags
    if (ctl->jpeg.gpsCoordinates[0] != 0 && ctl->jpeg.gpsCoordinates[1] != 0) {

        if (ctl->jpeg.gpsCoordinates[0] > 0)
            strcpy((char *)exifInfo->gps_latitude_ref, "N");
        else
            strcpy((char *)exifInfo->gps_latitude_ref, "S");

        if (ctl->jpeg.gpsCoordinates[1] > 0)
            strcpy((char *)exifInfo->gps_longitude_ref, "E");
        else
            strcpy((char *)exifInfo->gps_longitude_ref, "W");

        if (ctl->jpeg.gpsCoordinates[2] > 0)
            exifInfo->gps_altitude_ref = 0;
        else
            exifInfo->gps_altitude_ref = 1;

        double latitude = fabs(ctl->jpeg.gpsCoordinates[0]);
        double longitude = fabs(ctl->jpeg.gpsCoordinates[1]);
        double altitude = fabs(ctl->jpeg.gpsCoordinates[2]);

        exifInfo->gps_latitude[0].num = (uint32_t)latitude;
        exifInfo->gps_latitude[0].den = 1;
        exifInfo->gps_latitude[1].num = (uint32_t)((latitude - exifInfo->gps_latitude[0].num) * 60);
        exifInfo->gps_latitude[1].den = 1;
        exifInfo->gps_latitude[2].num = (uint32_t)round((((latitude - exifInfo->gps_latitude[0].num) * 60)
                                        - exifInfo->gps_latitude[1].num) * 60);
        exifInfo->gps_latitude[2].den = 1;

        exifInfo->gps_longitude[0].num = (uint32_t)longitude;
        exifInfo->gps_longitude[0].den = 1;
        exifInfo->gps_longitude[1].num = (uint32_t)((longitude - exifInfo->gps_longitude[0].num) * 60);
        exifInfo->gps_longitude[1].den = 1;
        exifInfo->gps_longitude[2].num = (uint32_t)round((((longitude - exifInfo->gps_longitude[0].num) * 60)
                                        - exifInfo->gps_longitude[1].num) * 60);
        exifInfo->gps_longitude[2].den = 1;

        exifInfo->gps_altitude.num = (uint32_t)round(altitude);
        exifInfo->gps_altitude.den = 1;

        struct tm tm_data;
        long timestamp;
        timestamp = (long)ctl->jpeg.gpsTimestamp;
        gmtime_r(&timestamp, &tm_data);
        exifInfo->gps_timestamp[0].num = tm_data.tm_hour;
        exifInfo->gps_timestamp[0].den = 1;
        exifInfo->gps_timestamp[1].num = tm_data.tm_min;
        exifInfo->gps_timestamp[1].den = 1;
        exifInfo->gps_timestamp[2].num = tm_data.tm_sec;
        exifInfo->gps_timestamp[2].den = 1;
        snprintf((char*)exifInfo->gps_datestamp, sizeof(exifInfo->gps_datestamp),
                "%04d:%02d:%02d", tm_data.tm_year + 1900, tm_data.tm_mon + 1, tm_data.tm_mday);

        memset(exifInfo->gps_processing_method, 0, 100);
        memcpy(exifInfo->gps_processing_method, currentEntry->gpsProcessingMethod, 32);
        exifInfo->enableGps = true;
    } else {
        exifInfo->enableGps = false;
    }

    //2 1th IFD TIFF Tags
    mExifInfo.widthThumb  = 0;
    mExifInfo.heightThumb = 0;
    mExifInfo.enableThumb = false;
    if (m_camera2->isSupportedThumbnailResolution(ctl->jpeg.thumbnailSize[0], ctl->jpeg.thumbnailSize[1])) {
        mExifInfo.enableThumb = true;
        mExifInfo.widthThumb  = ctl->jpeg.thumbnailSize[0];
        mExifInfo.heightThumb = ctl->jpeg.thumbnailSize[1];
        ALOGV("(%s) widthThumb = %d, heightThumb = %d", __FUNCTION__, mExifInfo.widthThumb, mExifInfo.heightThumb);
    }
}

ExynosCameraHWInterface2::MainThread::~MainThread()
{
    ALOGV("(%s):", __FUNCTION__);
}

void ExynosCameraHWInterface2::MainThread::release()
{
    ALOGV("(%s):", __func__);
    SetSignal(SIGNAL_THREAD_RELEASE);
}

ExynosCameraHWInterface2::SensorThread::~SensorThread()
{
    ALOGV("(%s):", __FUNCTION__);
}

void ExynosCameraHWInterface2::SensorThread::release()
{
    ALOGV("(%s):", __func__);
    SetSignal(SIGNAL_THREAD_RELEASE);
}

ExynosCameraHWInterface2::StreamThread::~StreamThread()
{
    ALOGV("(%s):", __FUNCTION__);
}

void ExynosCameraHWInterface2::StreamThread::setParameter(stream_parameters_t * new_parameters)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    memcpy(&m_parameters, new_parameters, sizeof(stream_parameters_t));
}

void ExynosCameraHWInterface2::StreamThread::release()
{
    ALOGV("(%s):", __func__);
    SetSignal(SIGNAL_THREAD_RELEASE);
}

int ExynosCameraHWInterface2::StreamThread::findBufferIndex(void * bufAddr)
{
    int index;
    for (index = 0 ; index < m_parameters.numSvcBuffers ; index++) {
        if (m_parameters.svcBuffers[index].virt.extP[0] == bufAddr)
            return index;
    }
    return -1;
}

int ExynosCameraHWInterface2::StreamThread::findBufferIndex(buffer_handle_t * bufHandle)
{
    int index;
    for (index = 0 ; index < m_parameters.numSvcBuffers ; index++) {
        if (m_parameters.svcBufHandle[index] == *bufHandle)
            return index;
    }
    return -1;
}

status_t ExynosCameraHWInterface2::StreamThread::attachSubStream(int stream_id, int priority)
{
    ALOGV("(%s): substream_id(%d)", __FUNCTION__, stream_id);
    int index, vacantIndex;
    bool vacancy = false;

    for (index = 0 ; index < NUM_MAX_SUBSTREAM ; index++) {
        if (!vacancy && m_attachedSubStreams[index].streamId == -1) {
            vacancy = true;
            vacantIndex = index;
        } else if (m_attachedSubStreams[index].streamId == stream_id) {
            return BAD_VALUE;
        }
    }
    if (!vacancy)
        return NO_MEMORY;
    m_attachedSubStreams[vacantIndex].streamId = stream_id;
    m_attachedSubStreams[vacantIndex].priority = priority;
    m_numRegisteredStream++;
    return NO_ERROR;
}

status_t ExynosCameraHWInterface2::StreamThread::detachSubStream(int stream_id)
{
    ALOGV("(%s): substream_id(%d)", __FUNCTION__, stream_id);
    int index;
    bool found = false;

    for (index = 0 ; index < NUM_MAX_SUBSTREAM ; index++) {
        if (m_attachedSubStreams[index].streamId == stream_id) {
            found = true;
            break;
        }
    }
    if (!found)
        return BAD_VALUE;
    m_attachedSubStreams[index].streamId = -1;
    m_attachedSubStreams[index].priority = 0;
    m_numRegisteredStream--;
    return NO_ERROR;
}

ExynosCameraHWInterface2::JpegEncThread::JpegEncThread()
{
    ALOGV("(%s):", __FUNCTION__);
}

void ExynosCameraHWInterface2::JpegEncThread::release()
{
    ALOGV("(%s):", __func__);
    SetSignal(SIGNAL_THREAD_RELEASE);
}

uint8_t ExynosCameraHWInterface2::JpegEncThread::getInProgressCount()
{
    return m_inProgressCount;
}

void ExynosCameraHWInterface2::JpegEncThread::enqueueEncoding(jpegEnc_parameters_t *parameters)
{
    /* temporary code before implementing work queue */
    ALOGV("(%s): START - inProgressCount = %d", __FUNCTION__, m_inProgressCount);
    while (m_inProgressCount)
        usleep(1000);

    m_inProgressCount++;
    ALOGV("(%s): updated inProgressCount = %d", __FUNCTION__, m_inProgressCount);
    memcpy(&(mHardware->m_jpegEncParameters), parameters, sizeof(jpegEnc_parameters_t));
    SetSignal(SIGNAL_JPEG_START_ENCODING);
}

ExynosCameraHWInterface2::MiscThread::MiscThread()
{
    ALOGV("(%s):", __FUNCTION__);
}

void ExynosCameraHWInterface2::MiscThread::release()
{
    ALOGV("(%s):", __func__);
    SetSignal(SIGNAL_THREAD_RELEASE);
}

int ExynosCameraHWInterface2::createIonClient(ion_client ionClient)
{
    if (ionClient == 0) {
        ionClient = ion_client_create();
        if (ionClient < 0) {
            ALOGE("[%s]src ion client create failed, value = %d\n", __FUNCTION__, ionClient);
            return 0;
        }
    }
    return ionClient;
}

int ExynosCameraHWInterface2::deleteIonClient(ion_client ionClient)
{
    if (ionClient != 0) {
        if (ionClient > 0) {
            ion_client_destroy(ionClient);
        }
        ionClient = 0;
    }
    return ionClient;
}

int ExynosCameraHWInterface2::allocCameraMemory(ion_client ionClient, ExynosBuffer *buf, int iMemoryNum)
{
    return allocCameraMemory(ionClient, buf, iMemoryNum, 0);
}

int ExynosCameraHWInterface2::allocCameraMemory(ion_client ionClient, ExynosBuffer *buf, int iMemoryNum, int cacheFlag)
{
    int ret = 0;
    int i = 0;
    int flag = 0;

    if (ionClient == 0) {
        ALOGE("[%s] ionClient is zero (%d)\n", __FUNCTION__, ionClient);
        return -1;
    }

    for (i = 0 ; i < iMemoryNum ; i++) {
        if (buf->size.extS[i] == 0) {
            break;
        }
        if (1 << i & cacheFlag)
            flag = ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC | ION_FLAG_PRESERVE_KMAP;
        else
            flag = 0;
        buf->fd.extFd[i] = ion_alloc(ionClient, buf->size.extS[i], 0, ION_HEAP_SYSTEM_MASK, flag);
        if ((buf->fd.extFd[i] == -1) ||(buf->fd.extFd[i] == 0)) {
            ALOGE("[%s]ion_alloc(%d) failed\n", __FUNCTION__, buf->size.extS[i]);
            buf->fd.extFd[i] = -1;
            freeCameraMemory(buf, iMemoryNum);
            return -1;
        }

        buf->virt.extP[i] = (char *)ion_map(buf->fd.extFd[i], \
                                        buf->size.extS[i], 0);
        if ((buf->virt.extP[i] == (char *)MAP_FAILED) || (buf->virt.extP[i] == NULL)) {
            ALOGE("[%s]src ion map failed(%d)\n", __FUNCTION__, buf->size.extS[i]);
            buf->virt.extP[i] = (char *)MAP_FAILED;
            freeCameraMemory(buf, iMemoryNum);
            return -1;
        }
        ALOGV("allocCameraMem : [%d][0x%08x] size(%d) flag(%d)", i, (unsigned int)(buf->virt.extP[i]), buf->size.extS[i], flag);
    }

    return ret;
}

void ExynosCameraHWInterface2::freeCameraMemory(ExynosBuffer *buf, int iMemoryNum)
{

    int i = 0 ;
    int ret = 0;

    for (i=0;i<iMemoryNum;i++) {
        if (buf->fd.extFd[i] != -1) {
            if (buf->virt.extP[i] != (char *)MAP_FAILED) {
                ret = ion_unmap(buf->virt.extP[i], buf->size.extS[i]);
                if (ret < 0)
                    ALOGE("ERR(%s)", __FUNCTION__);
            }
            ion_free(buf->fd.extFd[i]);
        ALOGV("freeCameraMemory : [%d][0x%08x] size(%d)", i, (unsigned int)(buf->virt.extP[i]), buf->size.extS[i]);
        }
        buf->fd.extFd[i] = -1;
        buf->virt.extP[i] = (char *)MAP_FAILED;
        buf->size.extS[i] = 0;
    }
}

void ExynosCameraHWInterface2::initCameraMemory(ExynosBuffer *buf, int iMemoryNum)
{
    int i =0 ;
    for (i=0;i<iMemoryNum;i++) {
        buf->virt.extP[i] = (char *)MAP_FAILED;
        buf->fd.extFd[i] = -1;
        buf->size.extS[i] = 0;
    }
}




static camera2_device_t *g_cam2_device = NULL;
static bool g_camera_valid = false;
static Mutex g_camera_mutex;
ExynosCamera2 * g_camera2[2] = { NULL, NULL };

static int HAL2_camera_device_close(struct hw_device_t* device)
{
    Mutex::Autolock lock(g_camera_mutex);
    ALOGD("(%s): ENTER", __FUNCTION__);
    if (device) {

        camera2_device_t *cam_device = (camera2_device_t *)device;
        ALOGV("cam_device(0x%08x):", (unsigned int)cam_device);
        ALOGV("g_cam2_device(0x%08x):", (unsigned int)g_cam2_device);
        delete static_cast<ExynosCameraHWInterface2 *>(cam_device->priv);
        free(cam_device);
        g_camera_valid = false;
        g_cam2_device = NULL;
    }

    ALOGD("(%s): EXIT", __FUNCTION__);
    return 0;
}

static inline ExynosCameraHWInterface2 *obj(const struct camera2_device *dev)
{
    return reinterpret_cast<ExynosCameraHWInterface2 *>(dev->priv);
}

static int HAL2_device_set_request_queue_src_ops(const struct camera2_device *dev,
            const camera2_request_queue_src_ops_t *request_src_ops)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->setRequestQueueSrcOps(request_src_ops);
}

static int HAL2_device_notify_request_queue_not_empty(const struct camera2_device *dev)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->notifyRequestQueueNotEmpty();
}

static int HAL2_device_set_frame_queue_dst_ops(const struct camera2_device *dev,
            const camera2_frame_queue_dst_ops_t *frame_dst_ops)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->setFrameQueueDstOps(frame_dst_ops);
}

static int HAL2_device_get_in_progress_count(const struct camera2_device *dev)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->getInProgressCount();
}

static int HAL2_device_flush_captures_in_progress(const struct camera2_device *dev)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->flushCapturesInProgress();
}

static int HAL2_device_construct_default_request(const struct camera2_device *dev,
            int request_template, camera_metadata_t **request)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->constructDefaultRequest(request_template, request);
}

static int HAL2_device_allocate_stream(
            const struct camera2_device *dev,
            // inputs
            uint32_t width,
            uint32_t height,
            int      format,
            const camera2_stream_ops_t *stream_ops,
            // outputs
            uint32_t *stream_id,
            uint32_t *format_actual,
            uint32_t *usage,
            uint32_t *max_buffers)
{
    ALOGV("(%s): ", __FUNCTION__);
    return obj(dev)->allocateStream(width, height, format, stream_ops,
                                    stream_id, format_actual, usage, max_buffers);
}

static int HAL2_device_register_stream_buffers(const struct camera2_device *dev,
            uint32_t stream_id,
            int num_buffers,
            buffer_handle_t *buffers)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->registerStreamBuffers(stream_id, num_buffers, buffers);
}

static int HAL2_device_release_stream(
        const struct camera2_device *dev,
            uint32_t stream_id)
{
    ALOGV("DEBUG(%s)(id: %d):", __FUNCTION__, stream_id);
    if (!g_camera_valid)
        return 0;
    return obj(dev)->releaseStream(stream_id);
}

static int HAL2_device_allocate_reprocess_stream(
           const struct camera2_device *dev,
            uint32_t width,
            uint32_t height,
            uint32_t format,
            const camera2_stream_in_ops_t *reprocess_stream_ops,
            // outputs
            uint32_t *stream_id,
            uint32_t *consumer_usage,
            uint32_t *max_buffers)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->allocateReprocessStream(width, height, format, reprocess_stream_ops,
                                    stream_id, consumer_usage, max_buffers);
}

static int HAL2_device_allocate_reprocess_stream_from_stream(
           const struct camera2_device *dev,
            uint32_t output_stream_id,
            const camera2_stream_in_ops_t *reprocess_stream_ops,
            // outputs
            uint32_t *stream_id)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->allocateReprocessStreamFromStream(output_stream_id,
                                    reprocess_stream_ops, stream_id);
}

static int HAL2_device_release_reprocess_stream(
        const struct camera2_device *dev,
            uint32_t stream_id)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->releaseReprocessStream(stream_id);
}

static int HAL2_device_trigger_action(const struct camera2_device *dev,
           uint32_t trigger_id,
            int ext1,
            int ext2)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    if (!g_camera_valid)
        return 0;
    return obj(dev)->triggerAction(trigger_id, ext1, ext2);
}

static int HAL2_device_set_notify_callback(const struct camera2_device *dev,
            camera2_notify_callback notify_cb,
            void *user)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->setNotifyCallback(notify_cb, user);
}

static int HAL2_device_get_metadata_vendor_tag_ops(const struct camera2_device*dev,
            vendor_tag_query_ops_t **ops)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->getMetadataVendorTagOps(ops);
}

static int HAL2_device_dump(const struct camera2_device *dev, int fd)
{
    ALOGV("DEBUG(%s):", __FUNCTION__);
    return obj(dev)->dump(fd);
}





static int HAL2_getNumberOfCameras()
{
    ALOGV("(%s): returning 2", __FUNCTION__);
    return 2;
}


static int HAL2_getCameraInfo(int cameraId, struct camera_info *info)
{
    ALOGV("DEBUG(%s): cameraID: %d", __FUNCTION__, cameraId);
    static camera_metadata_t * mCameraInfo[2] = {NULL, NULL};

    status_t res;

    if (cameraId == 0) {
        info->facing = CAMERA_FACING_BACK;
        if (!g_camera2[0])
            g_camera2[0] = new ExynosCamera2(0);
    }
    else if (cameraId == 1) {
        info->facing = CAMERA_FACING_FRONT;
        if (!g_camera2[1])
            g_camera2[1] = new ExynosCamera2(1);
    }
    else
        return BAD_VALUE;

    info->orientation = 0;
    info->device_version = HARDWARE_DEVICE_API_VERSION(2, 0);
    if (mCameraInfo[cameraId] == NULL) {
        res = g_camera2[cameraId]->constructStaticInfo(&(mCameraInfo[cameraId]), cameraId, true);
        if (res != OK) {
            ALOGE("%s: Unable to allocate static info: %s (%d)",
                    __FUNCTION__, strerror(-res), res);
            return res;
        }
        res = g_camera2[cameraId]->constructStaticInfo(&(mCameraInfo[cameraId]), cameraId, false);
        if (res != OK) {
            ALOGE("%s: Unable to fill in static info: %s (%d)",
                    __FUNCTION__, strerror(-res), res);
            return res;
        }
    }
    info->static_camera_characteristics = mCameraInfo[cameraId];
    return NO_ERROR;
}

#define SET_METHOD(m) m : HAL2_device_##m

static camera2_device_ops_t camera2_device_ops = {
        SET_METHOD(set_request_queue_src_ops),
        SET_METHOD(notify_request_queue_not_empty),
        SET_METHOD(set_frame_queue_dst_ops),
        SET_METHOD(get_in_progress_count),
        SET_METHOD(flush_captures_in_progress),
        SET_METHOD(construct_default_request),
        SET_METHOD(allocate_stream),
        SET_METHOD(register_stream_buffers),
        SET_METHOD(release_stream),
        SET_METHOD(allocate_reprocess_stream),
        SET_METHOD(allocate_reprocess_stream_from_stream),
        SET_METHOD(release_reprocess_stream),
        SET_METHOD(trigger_action),
        SET_METHOD(set_notify_callback),
        SET_METHOD(get_metadata_vendor_tag_ops),
        SET_METHOD(dump),
};

#undef SET_METHOD


static int HAL2_camera_device_open(const struct hw_module_t* module,
                                  const char *id,
                                  struct hw_device_t** device)
{
    int cameraId = atoi(id);
    int openInvalid = 0;

    Mutex::Autolock lock(g_camera_mutex);
    if (g_camera_valid) {
        ALOGE("ERR(%s): Can't open, other camera is in use", __FUNCTION__);
        return -EBUSY;
    }
    g_camera_valid = false;
    ALOGD("\n\n>>> I'm Samsung's CameraHAL_2(ID:%d) <<<\n\n", cameraId);
    if (cameraId < 0 || cameraId >= HAL2_getNumberOfCameras()) {
        ALOGE("ERR(%s):Invalid camera ID %s", __FUNCTION__, id);
        return -EINVAL;
    }

    ALOGD("g_cam2_device : 0x%08x", (unsigned int)g_cam2_device);
    if (g_cam2_device) {
        if (obj(g_cam2_device)->getCameraId() == cameraId) {
            ALOGD("DEBUG(%s):returning existing camera ID %s", __FUNCTION__, id);
            goto done;
        } else {
            ALOGD("(%s): START waiting for cam device free", __FUNCTION__);
            while (g_cam2_device)
                usleep(SIG_WAITING_TICK);
            ALOGD("(%s): END   waiting for cam device free", __FUNCTION__);
        }
    }

    g_cam2_device = (camera2_device_t *)malloc(sizeof(camera2_device_t));
    ALOGV("g_cam2_device : 0x%08x", (unsigned int)g_cam2_device);

    if (!g_cam2_device)
        return -ENOMEM;

    g_cam2_device->common.tag     = HARDWARE_DEVICE_TAG;
    g_cam2_device->common.version = CAMERA_DEVICE_API_VERSION_2_0;
    g_cam2_device->common.module  = const_cast<hw_module_t *>(module);
    g_cam2_device->common.close   = HAL2_camera_device_close;

    g_cam2_device->ops = &camera2_device_ops;

    ALOGV("DEBUG(%s):open camera2 %s", __FUNCTION__, id);

    g_cam2_device->priv = new ExynosCameraHWInterface2(cameraId, g_cam2_device, g_camera2[cameraId], &openInvalid);
    if (!openInvalid) {
        ALOGE("DEBUG(%s): ExynosCameraHWInterface2 creation failed", __FUNCTION__);
        return -ENODEV;
    }
done:
    *device = (hw_device_t *)g_cam2_device;
    ALOGV("DEBUG(%s):opened camera2 %s (%p)", __FUNCTION__, id, *device);
    g_camera_valid = true;

    return 0;
}


static hw_module_methods_t camera_module_methods = {
            open : HAL2_camera_device_open
};

extern "C" {
    struct camera_module HAL_MODULE_INFO_SYM = {
      common : {
          tag                : HARDWARE_MODULE_TAG,
          module_api_version : CAMERA_MODULE_API_VERSION_2_0,
          hal_api_version    : HARDWARE_HAL_API_VERSION,
          id                 : CAMERA_HARDWARE_MODULE_ID,
          name               : "Exynos Camera HAL2",
          author             : "Samsung Corporation",
          methods            : &camera_module_methods,
          dso:                NULL,
          reserved:           {0},
      },
      get_number_of_cameras : HAL2_getNumberOfCameras,
      get_camera_info       : HAL2_getCameraInfo
    };
}

}; // namespace android
