/*
 * Copyright (C) 2019-2020 The LineageOS Project
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

#define LOG_TAG "SunlightEnhancementService"

#include <fstream>

#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <utils/Errors.h>

#include "SunlightEnhancement.h"

namespace vendor {
namespace lineage {
namespace livedisplay {
namespace V2_0 {
namespace implementation {

static constexpr const char* kHbmStatusPath =
        "/sys/devices/platform/soc/ae00000.qcom,mdss_mdp/drm/card0/card0-DSI-1/hbm_status";

bool hasAmoledPanel() {
    std::string device = android::base::GetProperty("ro.product.device", "");
    return device == "davinci" || device == "davinciin" || device == "tucana";
}

bool SunlightEnhancement::isSupported() {
    if (hasAmoledPanel()) {
        std::ifstream hbm_status_file(kHbmStatusPath);
        if (!hbm_status_file.is_open()) {
            LOG(ERROR) << "Failed to open " << kHbmStatusPath << ", error=" << errno
                       << " (" << strerror(errno) << ")";
        }
        return !hbm_status_file.fail();
    }
    return false;
}

Return<bool> SunlightEnhancement::isEnabled() {
    std::ifstream hbm_status_file(kHbmStatusPath);
    int result = -1;
    hbm_status_file >> result;
    return !hbm_status_file.fail() && result > 0;
}

Return<bool> SunlightEnhancement::setEnabled(bool enabled) {
    xiaomiDisplayFeatureService = IDisplayFeature::getService();
    if (enabled) {
        xiaomiDisplayFeatureService->setFeature(0, 11, 1, 3);
    } else {
        xiaomiDisplayFeatureService->setFeature(0, 11, 0, 3);
    }
    return false;
}

}  // namespace implementation
}  // namespace V2_0
}  // namespace livedisplay
}  // namespace lineage
}  // namespace vendor
