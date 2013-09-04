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
 * \file      ExynosCameraHWInterface2.h
 * \brief     header file for Android Camera API 2.0 HAL
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

#ifndef EXYNOS_CAMERA_HW_INTERFACE_2_H
#define EXYNOS_CAMERA_HW_INTERFACE_2_H

#include <hardware/camera2.h>
#include <camera/Camera.h>
#include <camera/CameraParameters.h>
#include <utils/List.h>
#include "SignalDrivenThread.h"
#include "MetadataConverter.h"
#include "exynos_v4l2.h"
#include "ExynosRect.h"
#include "ExynosBuffer.h"
#include "videodev2_exynos_camera.h"
#include "gralloc_priv.h"
#include "ExynosJpegEncoderForCamera.h"
#include <fcntl.h>
#include "fimc-is-metadata.h"
#include "ion.h"
#include "ExynosExif.h"
#include "csc.h"
#include "ExynosCamera2.h"
#include "cutils/properties.h"

namespace android {

#define NODE_PREFIX     "/dev/video"

#define NUM_MAX_STREAM_THREAD       (5)
#define NUM_MAX_REQUEST_MGR_ENTRY   (6)
#define NUM_MAX_CAMERA_BUFFERS      (16)
#define NUM_BAYER_BUFFERS           (8)
#define NUM_SCC_BUFFERS             (8)
#define NUM_SCP_BUFFERS             (8)
#define NUM_MIN_SENSOR_QBUF         (3)
#define NUM_MAX_SUBSTREAM           (4)

#define PICTURE_GSC_NODE_NUM (2)
#define VIDEO_GSC_NODE_NUM (1)

#define STREAM_TYPE_DIRECT   (0)
#define STREAM_TYPE_INDIRECT (1)


#define SIGNAL_MAIN_STREAM_OUTPUT_DONE          (SIGNAL_THREAD_COMMON_LAST<<3)
#define SIGNAL_SENSOR_START_REQ_PROCESSING      (SIGNAL_THREAD_COMMON_LAST<<4)

#define SIGNAL_JPEG_START_ENCODING              (SIGNAL_THREAD_COMMON_LAST<<5)

#define SIGNAL_THREAD_RELEASE                   (SIGNAL_THREAD_COMMON_LAST<<8)

#define SIGNAL_STREAM_REPROCESSING_START        (SIGNAL_THREAD_COMMON_LAST<<14)
#define SIGNAL_STREAM_DATA_COMING               (SIGNAL_THREAD_COMMON_LAST<<15)

#define STREAM_ID_PREVIEW           (0)
#define STREAM_MASK_PREVIEW         (1<<STREAM_ID_PREVIEW)
#define STREAM_ID_RECORD            (1)
#define STREAM_MASK_RECORD          (1<<STREAM_ID_RECORD)
#define STREAM_ID_PRVCB             (2)
#define STREAM_MASK_PRVCB           (1<<STREAM_ID_PRVCB)
#define STREAM_ID_JPEG              (4)
#define STREAM_MASK_JPEG            (1<<STREAM_ID_JPEG)
#define STREAM_ID_ZSL               (5)
#define STREAM_MASK_ZSL             (1<<STREAM_ID_ZSL)

#define STREAM_ID_JPEG_REPROCESS    (8)
#define STREAM_ID_LAST              STREAM_ID_JPEG_REPROCESS

#define MASK_OUTPUT_SCP             (STREAM_MASK_PREVIEW|STREAM_MASK_RECORD|STREAM_MASK_PRVCB)
#define MASK_OUTPUT_SCC             (STREAM_MASK_JPEG|STREAM_MASK_ZSL)

#define SUBSTREAM_TYPE_NONE         (0)
#define SUBSTREAM_TYPE_JPEG         (1)
#define SUBSTREAM_TYPE_RECORD       (2)
#define SUBSTREAM_TYPE_PRVCB        (3)
#define FLASH_STABLE_WAIT_TIMEOUT        (10)

#define SIG_WAITING_TICK            (5000)

enum sensor_name {
    SENSOR_NAME_S5K3H2  = 1,
    SENSOR_NAME_S5K6A3  = 2,
    SENSOR_NAME_S5K4E5  = 3,
    SENSOR_NAME_S5K3H7  = 4,
    SENSOR_NAME_CUSTOM  = 5,
    SENSOR_NAME_END
};

enum is_subscenario_id {
	ISS_SUB_SCENARIO_STILL,
	ISS_SUB_SCENARIO_VIDEO,
	ISS_SUB_SCENARIO_SCENE1,
	ISS_SUB_SCENARIO_SCENE2,
	ISS_SUB_SCENARIO_SCENE3,
	ISS_SUB_END
};

typedef struct node_info {
    int fd;
    uint32_t width;
    uint32_t height;
    int format;
    int planes;
    int buffers;
    enum v4l2_memory memory;
    enum v4l2_buf_type type;
    ExynosBuffer buffer[NUM_MAX_CAMERA_BUFFERS];
    int status;
} node_info_t;


typedef struct camera_hw_info {
    int sensor_id;

    node_info_t sensor;
    node_info_t isp;
    node_info_t capture;
    node_info_t scp;

    /*shot*/  // temp
    struct camera2_shot_ext dummy_shot;

} camera_hw_info_t;

typedef enum request_entry_status {
    EMPTY,
    REGISTERED,
    REQUESTED,
    CAPTURED,
    METADONE,
    COMPLETED
} request_entry_status_t;

typedef struct request_manager_entry {
    request_entry_status_t      status;
    camera_metadata_t           *original_request;
    struct camera2_shot_ext     internal_shot;
    int                         output_stream_count;
} request_manager_entry_t;

typedef struct flash_control_info {
    // UI flash mode indicator
    enum aa_aemode    i_flashMode;
    // AF flash
    bool        m_afFlashFired;
    // Capture flash
    bool        m_flashEnableFlg;
    int         m_flashFrameCount;
    int         m_flashState;
    int        m_flashTimeOut;
    // Flash decision
    // At flash auto mode only : 1 -> flash is needed, 0 -> normal case
    bool        m_flashDecisionResult;
    // torch indicator. this will be replaced by flashMode meta
    bool        m_flashTorchMode;
    // for precapture metering
    int        m_precaptureState;
    int        m_precaptureTriggerId;
} ctl_flash_info_t;

typedef struct ae_control_info {
    // pre-capture notification state
    enum ae_state    aeStateNoti;
    enum aa_metering_aemode aeMeteringMode;
} ctl_ae_info_t;

typedef struct scene_control_info {
    // pre-capture notification state
    enum aa_scene_mode    prevSceneMode;
} ctl_scene_info_t;

typedef struct request_control_info {
    ctl_flash_info_t flash;
    ctl_ae_info_t ae;
    ctl_scene_info_t scene;
} ctl_request_info_t;

class RequestManager {
public:
    RequestManager(SignalDrivenThread* main_thread);
    ~RequestManager();
    void    ResetEntry();
    int     GetNumEntries();
    bool    IsRequestQueueFull();

    void    RegisterRequest(camera_metadata_t *new_request, int * afMode, uint32_t * afRegion);
    void    DeregisterRequest(camera_metadata_t **deregistered_request);
    bool    PrepareFrame(size_t *num_entries, size_t *frame_size,
                camera_metadata_t **prepared_frame, int afState);
    int     MarkProcessingRequest(ExynosBuffer * buf);
    void    NotifyStreamOutput(uint32_t frameCnt);
    void    ApplyDynamicMetadata(struct camera2_shot_ext *shot_ext);
    void    CheckCompleted(int index);
    void    UpdateIspParameters(struct camera2_shot_ext *shot_ext, int frameCnt, ctl_request_info_t *ctl_info);
    void    RegisterTimestamp(uint32_t frameCnt, nsecs_t *frameTime);
    nsecs_t  GetTimestampByFrameCnt(uint32_t frameCnt);
    nsecs_t  GetTimestamp(int index);
    uint8_t  GetOutputStreamByFrameCnt(uint32_t frameCnt);
    uint8_t  GetOutputStream(int index);
    camera2_shot_ext *  GetInternalShotExtByFrameCnt(uint32_t frameCnt);
    camera2_shot_ext *  GetInternalShotExt(int index);
    int     FindFrameCnt(struct camera2_shot_ext * shot_ext);
    bool    IsVdisEnable(void);
    int     FindEntryIndexByFrameCnt(uint32_t frameCnt);
    void    Dump(void);
    int     GetNextIndex(int index);
    int     GetPrevIndex(int index);
    void    SetDefaultParameters(int cropX);
    void    SetInitialSkip(int count);
    int     GetSkipCnt();
    void    DecreaseSkipCnt();
    int     GetCompletedIndex();
    void    pushSensorQ(int index);
    int     popSensorQ();
    void    releaseSensorQ();
private:

    MetadataConverter               *m_metadataConverter;
    SignalDrivenThread              *m_mainThread;
    int                             m_numOfEntries;
    int                             m_entryInsertionIndex;
    int                             m_entryProcessingIndex;
    int                             m_entryFrameOutputIndex;
    request_manager_entry_t         entries[NUM_MAX_REQUEST_MGR_ENTRY];
    int                             m_completedIndex;

    Mutex                           m_requestMutex;

    //TODO : alloc dynamically
    char                            m_tempFrameMetadataBuf[2000];
    camera_metadata_t               *m_tempFrameMetadata;

    int                             m_sensorPipelineSkipCnt;
    int                             m_cropX;
    uint32_t                         m_lastCompletedFrameCnt;
    int                             m_lastAeMode;
    int                             m_lastAaMode;
    int                             m_lastAwbMode;
    int                             m_lastAeComp;
    bool                            m_vdisBubbleEn;
    nsecs_t                         m_lastTimeStamp;
    List<int>                   m_sensorQ;
};

#define NOT_AVAILABLE           (0)
#define REQUIRES_DQ_FROM_SVC    (1)
#define ON_DRIVER               (2)
#define ON_HAL                  (3)
#define ON_SERVICE              (4)

typedef struct stream_parameters {
            uint32_t                width;
            uint32_t                height;
            int                     format;
    const   camera2_stream_ops_t*   streamOps;
            uint32_t                usage;
            int                     numHwBuffers;
            int                     numSvcBuffers;
            int                     numOwnSvcBuffers;
            int                     planes;
            int                     metaPlanes;
            int                     numSvcBufsInHal;
            buffer_handle_t         svcBufHandle[NUM_MAX_CAMERA_BUFFERS];
            ExynosBuffer            svcBuffers[NUM_MAX_CAMERA_BUFFERS];
            ExynosBuffer            metaBuffers[NUM_MAX_CAMERA_BUFFERS];
            int                     svcBufStatus[NUM_MAX_CAMERA_BUFFERS];
            int                     bufIndex;
            node_info_t             *node;
            int                     minUndequedBuffer;
            bool                    needsIonMap;
} stream_parameters_t;

typedef struct substream_parameters {
            int                     type;
            uint32_t                width;
            uint32_t                height;
            int                     format;
    const   camera2_stream_ops_t*   streamOps;
            uint32_t                usage;
            int                     numSvcBuffers;
            int                     numOwnSvcBuffers;
            int                     internalFormat;
            int                     internalPlanes;
            int                     svcPlanes;
            buffer_handle_t         svcBufHandle[NUM_MAX_CAMERA_BUFFERS];
            ExynosBuffer            svcBuffers[NUM_MAX_CAMERA_BUFFERS];
            int                     svcBufStatus[NUM_MAX_CAMERA_BUFFERS];
            int                     svcBufIndex;
            int                     numSvcBufsInHal;
            bool                    needBufferInit;
            int                     minUndequedBuffer;
} substream_parameters_t;

typedef struct substream_entry {
    int                     priority;
    int                     streamId;
} substream_entry_t;

typedef struct jpegEnc_parameters {
    ExynosBuffer        m_yuvBuf;
    ExynosBuffer        m_jpegBuf;
    ExynosRect          m_rect;
    exif_attribute_t    m_exifInfo;
    int                 m_jpegQuality;
    int                 m_thumbQuality;
    nsecs_t             m_frameTimeStamp;
    int                 m_svcBufIndex;
} jpegEnc_parameters_t;

class ExynosCameraHWInterface2 : public virtual RefBase {
public:
    ExynosCameraHWInterface2(int cameraId, camera2_device_t *dev, ExynosCamera2 * camera, int *openInvalid);
    virtual             ~ExynosCameraHWInterface2();

    virtual void        release();

    inline  int         getCameraId() const;

    virtual int         setRequestQueueSrcOps(const camera2_request_queue_src_ops_t *request_src_ops);
    virtual int         notifyRequestQueueNotEmpty();
    virtual int         setFrameQueueDstOps(const camera2_frame_queue_dst_ops_t *frame_dst_ops);
    virtual int         getInProgressCount();
    virtual int         flushCapturesInProgress();
    virtual int         constructDefaultRequest(int request_template, camera_metadata_t **request);
    virtual int         allocateStream(uint32_t width, uint32_t height,
                                    int format, const camera2_stream_ops_t *stream_ops,
                                    uint32_t *stream_id, uint32_t *format_actual, uint32_t *usage, uint32_t *max_buffers);
    virtual int         registerStreamBuffers(uint32_t stream_id, int num_buffers, buffer_handle_t *buffers);
    virtual int         releaseStream(uint32_t stream_id);
    virtual int         allocateReprocessStream(uint32_t width, uint32_t height,
                                    uint32_t format, const camera2_stream_in_ops_t *reprocess_stream_ops,
                                    uint32_t *stream_id, uint32_t *consumer_usage, uint32_t *max_buffers);
    virtual int         allocateReprocessStreamFromStream(uint32_t output_stream_id,
                                const camera2_stream_in_ops_t *reprocess_stream_ops, uint32_t *stream_id);
    virtual int         releaseReprocessStream(uint32_t stream_id);
    virtual int         triggerAction(uint32_t trigger_id, int ext1, int ext2);
    virtual int         setNotifyCallback(camera2_notify_callback notify_cb, void *user);
    virtual int         getMetadataVendorTagOps(vendor_tag_query_ops_t **ops);
    virtual int         dump(int fd);
private:
class MainThread : public SignalDrivenThread {
        ExynosCameraHWInterface2 *mHardware;
    public:
        MainThread(ExynosCameraHWInterface2 *hw):
            SignalDrivenThread(),
            mHardware(hw) { }
        ~MainThread();
        void threadFunctionInternal()
	    {
            mHardware->m_mainThreadFunc(this);
            return;
        }
        void        release(void);
        bool        m_releasing;
    };

    class SensorThread : public SignalDrivenThread {
        ExynosCameraHWInterface2 *mHardware;
    public:
        SensorThread(ExynosCameraHWInterface2 *hw):
            SignalDrivenThread(),
            mHardware(hw) { }
        ~SensorThread();
        void threadFunctionInternal() {
            mHardware->m_sensorThreadFunc(this);
            return;
        }
        void            release(void);
    //private:
        bool            m_releasing;
    };

    class StreamThread : public SignalDrivenThread {
        ExynosCameraHWInterface2 *mHardware;
    public:
        StreamThread(ExynosCameraHWInterface2 *hw, uint8_t new_index):
            SignalDrivenThread(),
            mHardware(hw),
            m_index(new_index) { }
        ~StreamThread();
        void threadFunctionInternal() {
            mHardware->m_streamThreadFunc(this);
            return;
        }
        void        setParameter(stream_parameters_t * new_parameters);
        status_t    attachSubStream(int stream_id, int priority);
        status_t    detachSubStream(int stream_id);
        void        release(void);
        int         findBufferIndex(void * bufAddr);
        int         findBufferIndex(buffer_handle_t * bufHandle);

        uint8_t                         m_index;
        bool                            m_activated;
    //private:
        stream_parameters_t             m_parameters;
        stream_parameters_t             *m_tempParameters;
        substream_entry_t               m_attachedSubStreams[NUM_MAX_SUBSTREAM];
        bool                            m_isBufferInit;
        bool                            m_releasing;
        int                             streamType;
        int                             m_numRegisteredStream;
     };

    class JpegEncThread : public SignalDrivenThread {
        ExynosCameraHWInterface2 *mHardware;
    public:
        JpegEncThread(ExynosCameraHWInterface2 *hw):
            SignalDrivenThread(),
            mHardware(hw),
            m_inProgressCount(0) { }
        JpegEncThread();
        void threadFunctionInternal() {
            mHardware->m_jpegEncThreadFunc(this);
            return;
        }
        void            release(void);
        uint8_t         getInProgressCount();
        void            enqueueEncoding(jpegEnc_parameters_t *parameters);

        uint8_t         m_inProgressCount;

    };

    class MiscThread : public SignalDrivenThread {
        ExynosCameraHWInterface2 *mHardware;
    public:
        MiscThread(ExynosCameraHWInterface2 *hw):
            SignalDrivenThread(),
            mHardware(hw) { }
        MiscThread();
        void threadFunctionInternal() {
            mHardware->m_miscThreadFunc(this);
            return;
        }
        void            release(void);
    };

    sp<MainThread>      m_mainThread;
    sp<SensorThread>    m_sensorThread;
    sp<StreamThread>    m_streamThreads[NUM_MAX_STREAM_THREAD];
    sp<JpegEncThread>   m_jpegEncThread;
    sp<MiscThread>      m_miscThread;
    substream_parameters_t  m_subStreams[STREAM_ID_LAST+1];

    RequestManager      *m_requestManager;
    ExynosCamera2       *m_camera2;

    void                m_mainThreadFunc(SignalDrivenThread * self);
    void                m_sensorThreadFunc(SignalDrivenThread * self);
    void                m_streamThreadFunc(SignalDrivenThread * self);
    void                m_jpegEncThreadFunc(SignalDrivenThread * self);
    void                m_miscThreadFunc(SignalDrivenThread * self);

    void                m_streamThreadInitialize(SignalDrivenThread * self);

    void                m_streamFunc_direct(SignalDrivenThread *self);
    void                m_streamFunc_indirect(SignalDrivenThread *self);

    void                m_streamBufferInit(SignalDrivenThread *self);

    int                 m_runSubStreamFunc(StreamThread *selfThread, ExynosBuffer *srcImageBuf,
                            int stream_id, nsecs_t frameTimeStamp);
    int                 m_jpegCreator(StreamThread *selfThread, ExynosBuffer *srcImageBuf, nsecs_t frameTimeStamp);
    int                 m_recordCreator(StreamThread *selfThread, ExynosBuffer *srcImageBuf, nsecs_t frameTimeStamp);
    int                 m_prvcbCreator(StreamThread *selfThread, ExynosBuffer *srcImageBuf, nsecs_t frameTimeStamp);
    void                m_getAlignedYUVSize(int colorFormat, int w, int h,
                                                ExynosBuffer *buf);
    bool                m_getRatioSize(int  src_w,  int   src_h,
                                             int  dst_w,  int   dst_h,
                                             int *crop_x, int *crop_y,
                                             int *crop_w, int *crop_h,
                                             int zoom);
	int				createIonClient(ion_client ionClient);
	int					deleteIonClient(ion_client ionClient);

    int				allocCameraMemory(ion_client ionClient, ExynosBuffer *buf, int iMemoryNum);
    int             allocCameraMemory(ion_client ionClient, ExynosBuffer *buf, int iMemoryNum, int cacheFlag);
	void				freeCameraMemory(ExynosBuffer *buf, int iMemoryNum);
	void				initCameraMemory(ExynosBuffer *buf, int iMemoryNum);

    void            DumpInfoWithShot(struct camera2_shot_ext * shot_ext);
    int             m_dequeueSubstreamBuffer(substream_parameters_t  *subParms, bool dequeueOnlyOne);
    bool            dumpImage(struct ExynosBuffer *buffer, char *filename, int num, char *type);
    bool            yuv2Jpeg(ExynosBuffer *yuvBuf, ExynosBuffer *jpegBuf, ExynosRect *rect,
                                exif_attribute_t *exifInfo, int jpegQuality, int thumbQuality);
    int             InitializeISPChain();
    void            StartISP();
    void            StartSCCThread(bool threadExists);
    void            SetAfMode(enum aa_afmode afMode);
    void            OnAfTrigger(int id);
    void            OnAfTriggerAutoMacro(void);
    void            OnAfTriggerCAF(void);
    void            OnPrecaptureMeteringTriggerStart(int id);
    void            OnAfCancel(int id);
    void            OnPrecaptureMeteringNotificationISP();
    void            OnAfNotification(enum aa_afstate noti);
    void            OnAfNotificationAutoMacro(enum aa_afstate noti);
    void            OnAfNotificationCAF(enum aa_afstate noti);
    void            OnFlashListenerISP(struct camera2_shot_ext *shot_ext);
    void            SetAfStateForService(int newState);
    int             GetAfStateForService(void);
    exif_attribute_t    mExifInfo;
    void            m_setExifFixedAttribute(void);
    void            m_setExifChangedAttribute(exif_attribute_t *exifInfo, ExynosRect *rect,
                         camera2_shot_ext *currentEntry);

    void            *m_exynosPictureCSC;
    void            *m_exynosVideoCSC;

    void            m_setShotZoom(struct camera2_shot_ext *shot_ext);
    void            m_setShotAFMode(struct camera2_shot_ext *shot_ext);
    void            m_setShotAFTrigger(struct camera2_shot_ext *shot_ext);
    void            m_setShotAFRegion(struct camera2_shot_ext * shot_ext);
    void            m_setShotNightshot(struct camera2_shot_ext *shot_ext, int &matchedFrameCnt);
    void            m_setShotFrameDuration(struct camera2_shot_ext *shot_ext);
    void            m_setShotSensorMode(struct camera2_shot_ext *shot_ext);
    void            m_setShotReprocessing(struct camera2_shot_ext *shot_ext);
    void            m_setShotFlash(struct camera2_shot_ext *shot_ext);

    void            m_controlFlashTorch(struct camera2_shot_ext *shot_ext);
    void            m_controlFlashCapture(struct camera2_shot_ext *shot_ext, int &matchedFrameCnt);
    void            m_setDmAeState(struct camera2_shot_ext * shot_ext);
    void            m_setDmFaceRectangles(struct camera2_shot_ext *shot_ext);

    camera2_request_queue_src_ops_t     *m_requestQueueOps;
    camera2_frame_queue_dst_ops_t       *m_frameQueueOps;
    camera2_notify_callback             m_notifyCb;
    void                                *m_callbackCookie;

    bool                                m_isRequestQueueNull;
    camera2_device_t                    *m_halDevice;
    static gralloc_module_t const*      m_grallocHal;


    camera_hw_info_t                     m_camera_info;

	ion_client m_ionCameraClient;

    bool                                m_isIspStarted;

    ExynosBuffer                        m_sccLocalBuffer[NUM_MAX_CAMERA_BUFFERS];
    bool                                m_sccLocalBufferValid;

    ExynosBuffer                        m_resizeBuf;
    int                                 m_currentReprocessOutStreams;
    ExynosBuffer                        m_previewCbBuf;
    int             				    m_cameraId;
    bool                                m_wideAspect;
    uint32_t                            m_currentAfRegion[4];
    float                               m_zoomRatio;

    int                                 m_vdisBubbleCnt;
    int                                 m_vdisDupFrame;
    bool                                m_isAlreadyRegistered;

    mutable Mutex                       m_jpegEncoderLock;
    mutable Mutex                       m_TriggerLock;
    mutable Mutex                       m_streamInitLock;

    bool                                m_scpForceSuspended;
    int                                 m_afState;
    int                                 m_afTriggerId;
    enum aa_afmode                      m_afMode;
    enum aa_afmode                      m_afMode2;
    bool                                m_IsAfModeUpdateRequired;
    bool                                m_IsAfLockRequired;
    int                                 m_serviceAfState;
    struct camera2_shot_ext             m_jpegMetadata;
    int                                 m_nightCaptureCnt;
    int                                 m_nightCaptureFrameCnt;
    int                                 m_lastSceneMode;
    int                                 m_reprocessStreamId;
    const camera2_stream_in_ops_t *     m_reprocessOps;
    int                                 m_reprocessOutputStreamId;
    int                                 m_reprocessingFrameCnt;
    ctl_request_info_t        m_ctlInfo;

     jpegEnc_parameters_t           m_jpegEncParameters;
};

}; // namespace android

#endif
