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

#ifdef _EXYNOS_CAMERA_H_
#define _EXYNOS_CAMERA_H_

#include <hardware/camera.h>

#include "ExynosCameraInfo.h"

namespace android {

class ExynosCamera : public virtual RefBase {

private:
    ExynosCamera();
    virtual ~ExynosCamera();

public:
    static ExynosCamera *createInstance(void)
    {
        static ExynosCamera singleton;
        return &singleton;
    }

    bool                    create(int cameraId);
    bool                    destroy(void);
    int                     getCameraId(void);

private:
    int                     initializeIspChain(void);

    int                     m_cameraId;
    ExynosCameraInfo        *m_cameraInfo;
};

}; // namespace android

#endif /* _EXYNOS_CAMERA_H_ */
