/*
 * Copyright (C) 2019 The LineageOS Project
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

package org.lineageos.settings.popupcamera;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.UserHandle;

import org.lineageos.settings.sensors.SensorsUtils;

public class PopupCameraUtils {

    private static final String TAG = "PopupCameraUtils";
    private static final boolean DEBUG = false;

    private static final boolean isPopUpMotorAvailable(Context context) {
        SensorManager mSensorManager = context.getSystemService(SensorManager.class);
        Sensor mRaiseCameraSensor = SensorsUtils.getSensor(mSensorManager, "xiaomi.sensor.raise_camera");
        return mRaiseCameraSensor != null ? true : false;
    }

    public static void startService(Context context) {
        final boolean enable = isPopUpMotorAvailable(context);
        context.startServiceAsUser(new Intent(context, PopupCameraService.class),
                UserHandle.CURRENT);
        PackageManager pm = context.getPackageManager();
        pm.setComponentEnabledSetting(
                new ComponentName(context, PopupCameraSettingsActivity.class),
                enable ? pm.COMPONENT_ENABLED_STATE_ENABLED : pm.COMPONENT_ENABLED_STATE_DEFAULT, pm.SYNCHRONOUS);
    }
}
