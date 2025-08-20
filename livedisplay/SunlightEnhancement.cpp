/*
 * SPDX-FileCopyrightText: 2019-2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "SunlightEnhancementService"

#include <android-base/logging.h>
#include <SunlightEnhancement.h>
#include <fstream>

namespace aidl {
namespace vendor {
namespace lineage {
namespace livedisplay {

static constexpr const char* kHbmPath = "/sys/devices/platform/soc/soc:qcom,dsi-display/hbm";

ndk::ScopedAStatus SunlightEnhancement::getEnabled(bool* _aidl_return) {
    std::ifstream file(kHbmPath);
    int result = -1;
    file >> result;
    LOG(DEBUG) << "Got result " << result << " fail " << file.fail();
    if (file.fail()) {
        LOG(ERROR) << "Failed to read current SunlightEnhancement state";
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }
    *_aidl_return = result > 0;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus SunlightEnhancement::setEnabled(bool enabled) {
    bool isEnabled;
    if (auto status = getEnabled(&isEnabled); !status.isOk()) {
        return status;
    }
    if (isEnabled != enabled) {
        std::ofstream file(kHbmPath);
        file << (enabled ? "1" : "0");
        LOG(DEBUG) << "setEnabled fail " << file.fail();
        if (file.fail()) {
            LOG(ERROR) << "Failed to set SunlightEnhancement state";
            return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
        }
    }
    return ndk::ScopedAStatus::ok();
}

}  // namespace livedisplay
}  // namespace lineage
}  // namespace vendor
}  // namespace aidl
