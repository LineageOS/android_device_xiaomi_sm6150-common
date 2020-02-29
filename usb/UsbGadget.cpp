/*
 * Copyright (C) 2018, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright (C) 2018 The Android Open Source Project
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

#define LOG_TAG "android.hardware.usb.gadget@1.0-service-qti"

#include "UsbGadget.h"
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/inotify.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

constexpr int BUFFER_SIZE = 512;
constexpr int MAX_FILE_PATH_LENGTH = 256;
constexpr int EPOLL_EVENTS = 10;
constexpr bool DEBUG = false;
constexpr int DISCONNECT_WAIT_US = 10000;

#define GADGET_PATH "/config/usb_gadget/g1/"
#define PULLUP_PATH GADGET_PATH "UDC"
#define VENDOR_ID_PATH GADGET_PATH "idVendor"
#define PRODUCT_ID_PATH GADGET_PATH "idProduct"
#define DEVICE_CLASS_PATH GADGET_PATH "bDeviceClass"
#define DEVICE_SUB_CLASS_PATH GADGET_PATH "bDeviceSubClass"
#define DEVICE_PROTOCOL_PATH GADGET_PATH "bDeviceProtocol"
#define DESC_USE_PATH GADGET_PATH "os_desc/use"
#define OS_DESC_PATH GADGET_PATH "os_desc/b.1"
#define CONFIG_PATH GADGET_PATH "configs/b.1/"
#define FUNCTIONS_PATH GADGET_PATH "functions/"
#define FUNCTION_NAME "function"
#define FUNCTION_PATH CONFIG_PATH FUNCTION_NAME
#define ESOC_DEVICE_PATH "/sys/bus/esoc/devices"
#define SOC_MACHINE_PATH "/sys/devices/soc0/machine"
#define USB_CONTROLLER_PROP "vendor.usb.controller"
#define RNDIS_FUNC_NAME_PROP "vendor.usb.rndis.func.name"
#define RMNET_FUNC_NAME_PROP "vendor.usb.rmnet.func.name"
#define RMNET_INST_NAME_PROP "vendor.usb.rmnet.inst.name"
#define DPL_INST_NAME_PROP "vendor.usb.dpl.inst.name"
#define PERSIST_VENDOR_USB_PROP "persist.vendor.usb.config"

enum mdmType {
  INTERNAL,
  EXTERNAL,
  INTERNAL_EXTERNAL,
  NONE,
};

namespace android {
namespace hardware {
namespace usb {
namespace gadget {
namespace V1_0 {
namespace implementation {

volatile bool gadgetPullup;

// Used for debug.
static void displayInotifyEvent(struct inotify_event *i) {
  ALOGE("    wd =%2d; ", i->wd);
  if (i->cookie > 0) ALOGE("cookie =%4d; ", i->cookie);

  ALOGE("mask = ");
  if (i->mask & IN_ACCESS) ALOGE("IN_ACCESS ");
  if (i->mask & IN_ATTRIB) ALOGE("IN_ATTRIB ");
  if (i->mask & IN_CLOSE_NOWRITE) ALOGE("IN_CLOSE_NOWRITE ");
  if (i->mask & IN_CLOSE_WRITE) ALOGE("IN_CLOSE_WRITE ");
  if (i->mask & IN_CREATE) ALOGE("IN_CREATE ");
  if (i->mask & IN_DELETE) ALOGE("IN_DELETE ");
  if (i->mask & IN_DELETE_SELF) ALOGE("IN_DELETE_SELF ");
  if (i->mask & IN_IGNORED) ALOGE("IN_IGNORED ");
  if (i->mask & IN_ISDIR) ALOGE("IN_ISDIR ");
  if (i->mask & IN_MODIFY) ALOGE("IN_MODIFY ");
  if (i->mask & IN_MOVE_SELF) ALOGE("IN_MOVE_SELF ");
  if (i->mask & IN_MOVED_FROM) ALOGE("IN_MOVED_FROM ");
  if (i->mask & IN_MOVED_TO) ALOGE("IN_MOVED_TO ");
  if (i->mask & IN_OPEN) ALOGE("IN_OPEN ");
  if (i->mask & IN_Q_OVERFLOW) ALOGE("IN_Q_OVERFLOW ");
  if (i->mask & IN_UNMOUNT) ALOGE("IN_UNMOUNT ");
  ALOGE("\n");

  if (i->len > 0) ALOGE("        name = %s\n", i->name);
}

static void *monitorFfs(void *param) {
  UsbGadget *usbGadget = (UsbGadget *)param;
  char buf[BUFFER_SIZE];
  bool writeUdc = true, stopMonitor = false;
  struct epoll_event events[EPOLL_EVENTS];
  std::string gadgetName = GetProperty(USB_CONTROLLER_PROP, "");

  if (gadgetName.empty()) {
    ALOGE("UDC name not defined");
    return NULL;
  }

  bool descriptorWritten = true;
  for (int i = 0; i < static_cast<int>(usbGadget->mEndpointList.size()); i++) {
    if (access(usbGadget->mEndpointList.at(i).c_str(), R_OK)) {
      descriptorWritten = false;
      break;
    }
  }

  // notify here if the endpoints are already present.
  if (descriptorWritten && !!WriteStringToFile(gadgetName, PULLUP_PATH)) {
    lock_guard<mutex> lock(usbGadget->mLock);
    usbGadget->mCurrentUsbFunctionsApplied = true;
    gadgetPullup = true;
    usbGadget->mCv.notify_all();
  }

  while (!stopMonitor) {
    int nrEvents = epoll_wait(usbGadget->mEpollFd, events, EPOLL_EVENTS, -1);
    if (nrEvents <= 0) {
      ALOGE("epoll wait did not return descriptor number");
      continue;
    }

    for (int i = 0; i < nrEvents; i++) {
      ALOGI("event=%u on fd=%d\n", events[i].events, events[i].data.fd);

      if (events[i].data.fd == usbGadget->mInotifyFd) {
        // Process all of the events in buffer returned by read().
        int numRead = read(usbGadget->mInotifyFd, buf, BUFFER_SIZE);
        for (char *p = buf; p < buf + numRead;) {
          struct inotify_event *event = (struct inotify_event *)p;
          if (DEBUG) displayInotifyEvent(event);

          p += sizeof(struct inotify_event) + event->len;

          bool descriptorPresent = true;
          for (int j = 0; j < static_cast<int>(usbGadget->mEndpointList.size());
               j++) {
            if (access(usbGadget->mEndpointList.at(j).c_str(), R_OK)) {
              if (DEBUG)
                ALOGI("%s absent", usbGadget->mEndpointList.at(j).c_str());
              descriptorPresent = false;
              break;
            }
          }

          if (!descriptorPresent && !writeUdc) {
            if (DEBUG) ALOGI("endpoints not up");
            writeUdc = true;
          } else if (descriptorPresent && writeUdc &&
                     !!WriteStringToFile(gadgetName, PULLUP_PATH)) {
            lock_guard<mutex> lock(usbGadget->mLock);
            usbGadget->mCurrentUsbFunctionsApplied = true;
            ALOGI("GADGET pulled up");
            writeUdc = false;
            gadgetPullup = true;
            // notify the main thread to signal userspace.
            usbGadget->mCv.notify_all();
          }
        }
      } else {
        uint64_t flag;
        read(usbGadget->mEventFd, &flag, sizeof(flag));
        if (flag == 100) {
          stopMonitor = true;
          break;
        }
      }
    }
  }
  return NULL;
}

UsbGadget::UsbGadget()
    : mMonitorCreated(false),
      mCurrentUsbFunctionsApplied(false) {
  if (access(OS_DESC_PATH, R_OK) != 0)
    ALOGE("configfs setup not done yet");
}

static int unlinkFunctions(const char *path) {
  DIR *config = opendir(path);
  struct dirent *function;
  char filepath[MAX_FILE_PATH_LENGTH];
  int ret = 0;

  if (config == NULL) return -1;

  // d_type does not seems to be supported in /config
  // so filtering by name.
  while (((function = readdir(config)) != NULL)) {
    if ((strstr(function->d_name, FUNCTION_NAME) == NULL)) continue;
    // build the path for each file in the folder.
    snprintf(filepath, sizeof(filepath), "%s/%s", path, function->d_name);
    ret = remove(filepath);
    if (ret) {
      ALOGE("Unable  remove file %s errno:%d", filepath, errno);
      break;
    }
  }

  closedir(config);
  return ret;
}

static int addEpollFd(const unique_fd &epfd, const unique_fd &fd) {
  struct epoll_event event;
  int ret;

  event.data.fd = fd;
  event.events = EPOLLIN;

  ret = epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &event);
  if (ret) ALOGE("epoll_ctl error %d", errno);

  return ret;
}

Return<void> UsbGadget::getCurrentUsbFunctions(
    const sp<V1_0::IUsbGadgetCallback> &callback) {
  Return<void> ret = callback->getCurrentUsbFunctionsCb(
      mCurrentUsbFunctions, mCurrentUsbFunctionsApplied
                                ? Status::FUNCTIONS_APPLIED
                                : Status::FUNCTIONS_NOT_APPLIED);
  if (!ret.isOk())
    ALOGE("Call to getCurrentUsbFunctionsCb failed %s",
          ret.description().c_str());

  return Void();
}

V1_0::Status UsbGadget::tearDownGadget() {
  ALOGI("setCurrentUsbFunctions None");

  if (!WriteStringToFile("none", PULLUP_PATH))
    ALOGI("Gadget cannot be pulled down");

  if (!WriteStringToFile("0", DEVICE_CLASS_PATH)) return Status::ERROR;

  if (!WriteStringToFile("0", DEVICE_SUB_CLASS_PATH)) return Status::ERROR;

  if (!WriteStringToFile("0", DEVICE_PROTOCOL_PATH)) return Status::ERROR;

  if (!WriteStringToFile("0", DESC_USE_PATH)) return Status::ERROR;

  if (unlinkFunctions(CONFIG_PATH)) return Status::ERROR;

  if (mMonitorCreated) {
    uint64_t flag = 100;
    // Stop the monitor thread by writing into signal fd.
    write(mEventFd, &flag, sizeof(flag));
    mMonitor->join();
    mMonitorCreated = false;
    ALOGI("mMonitor destroyed");
  } else {
    ALOGI("mMonitor not running");
  }

  mInotifyFd.reset(-1);
  mEventFd.reset(-1);
  mEpollFd.reset(-1);
  mEndpointList.clear();
  return Status::SUCCESS;
}

static int linkFunction(const char *function, int index) {
  char functionPath[MAX_FILE_PATH_LENGTH];
  char link[MAX_FILE_PATH_LENGTH];

  snprintf(functionPath, sizeof(functionPath), "%s%s", FUNCTIONS_PATH, function);
  snprintf(link, sizeof(link), "%s%d", FUNCTION_PATH, index);
  if (symlink(functionPath, link)) {
    ALOGE("Cannot create symlink %s -> %s errno:%d", link, functionPath, errno);
    return -1;
  }
  return 0;
}

static V1_0::Status setVidPid(const char *vid, const char *pid) {
  if (!WriteStringToFile(vid, VENDOR_ID_PATH)) return Status::ERROR;

  if (!WriteStringToFile(pid, PRODUCT_ID_PATH)) return Status::ERROR;

  return Status::SUCCESS;
}

static V1_0::Status validateAndSetVidPid(uint64_t functions) {
  V1_0::Status ret = Status::SUCCESS;
  switch (functions) {
    case static_cast<uint64_t>(GadgetFunction::ADB):
      ret = setVidPid("0x18d1", "0x4ee7");
      break;
    case static_cast<uint64_t>(GadgetFunction::MTP):
      ret = setVidPid("0x18d1", "0x4ee1");
      break;
    case GadgetFunction::ADB | GadgetFunction::MTP:
      ret = setVidPid("0x18d1", "0x4ee2");
      break;
    case static_cast<uint64_t>(GadgetFunction::RNDIS):
      ret = setVidPid("0x18d1", "0x4ee3");
      break;
    case GadgetFunction::ADB | GadgetFunction::RNDIS:
      ret = setVidPid("0x18d1", "0x4ee4");
      break;
    case static_cast<uint64_t>(GadgetFunction::PTP):
      ret = setVidPid("0x18d1", "0x4ee5");
      break;
    case GadgetFunction::ADB | GadgetFunction::PTP:
      ret = setVidPid("0x18d1", "0x4ee6");
      break;
    case static_cast<uint64_t>(GadgetFunction::MIDI):
      ret = setVidPid("0x18d1", "0x4ee8");
      break;
    case GadgetFunction::ADB | GadgetFunction::MIDI:
      ret = setVidPid("0x18d1", "0x4ee9");
      break;
    case static_cast<uint64_t>(GadgetFunction::ACCESSORY):
      ret = setVidPid("0x18d1", "0x2d00");
      break;
    case GadgetFunction::ADB | GadgetFunction::ACCESSORY:
      ret = setVidPid("0x18d1", "0x2d01");
      break;
    case static_cast<uint64_t>(GadgetFunction::AUDIO_SOURCE):
      ret = setVidPid("0x18d1", "0x2d02");
      break;
    case GadgetFunction::ADB | GadgetFunction::AUDIO_SOURCE:
      ret = setVidPid("0x18d1", "0x2d03");
      break;
    case GadgetFunction::ACCESSORY | GadgetFunction::AUDIO_SOURCE:
      ret = setVidPid("0x18d1", "0x2d04");
      break;
    case GadgetFunction::ADB | GadgetFunction::ACCESSORY |
	    GadgetFunction::AUDIO_SOURCE:
      ret = setVidPid("0x18d1", "0x2d05");
      break;
    default:
      ALOGE("Combination not supported");
      ret = Status::CONFIGURATION_NOT_SUPPORTED;
  }
  return ret;
}

static enum mdmType getModemType() {
  struct dirent* entry;
  enum mdmType mtype = INTERNAL;
  size_t pos_sda, pos_p, length;
  std::unique_ptr<DIR, int(*)(DIR*)> dir(opendir(ESOC_DEVICE_PATH), closedir);
  std::string esoc_name, path, soc_machine, esoc_dev_path = ESOC_DEVICE_PATH;

 /* On some platforms, /sys/bus/esoc/ director may not exists.*/
  if (dir == NULL)
      return mtype;

  while ((entry = readdir(dir.get())) != NULL) {
    if (entry->d_name[0] == '.')
      continue;
    path = esoc_dev_path + "/" + entry->d_name + "/esoc_name";
    if (ReadFileToString(path, &esoc_name)) {
      if (esoc_name.find("MDM") != std::string::npos ||
        esoc_name.find("SDX") != std::string::npos) {
        mtype = EXTERNAL;
        break;
      }
    }
  }
  if (ReadFileToString(SOC_MACHINE_PATH, &soc_machine)) {
    pos_sda = soc_machine.find("SDA");
    pos_p = soc_machine.find_last_of('P');
    length = soc_machine.length();
    if (pos_sda != std::string::npos || pos_p == length - 1) {
      mtype = mtype ? mtype : NONE;
      goto done;
    }
    if (mtype)
      mtype = INTERNAL_EXTERNAL;
  }
done:
  ALOGI("getModemType %d", mtype);
  return mtype;
}

V1_0::Status UsbGadget::setupFunctions(
    uint64_t functions, const sp<V1_0::IUsbGadgetCallback> &callback,
    uint64_t timeout) {
  std::unique_lock<std::mutex> lk(mLock);

  unique_fd inotifyFd(inotify_init());
  if (inotifyFd < 0) {
    ALOGE("inotify init failed");
    return Status::ERROR;
  }

  bool ffsEnabled = false;
  int i = 0;
  enum mdmType mtype;
  std::string gadgetName = GetProperty(USB_CONTROLLER_PROP, "");
  std::string rndisFunc = GetProperty(RNDIS_FUNC_NAME_PROP, "");
  std::string rmnetFunc = GetProperty(RMNET_FUNC_NAME_PROP, "");
  std::string rmnetInst = GetProperty(RMNET_INST_NAME_PROP, "");
  std::string dplInst = GetProperty(DPL_INST_NAME_PROP, "");
  std::string vendorProp = GetProperty(PERSIST_VENDOR_USB_PROP, "");

  if (gadgetName.empty()) {
    ALOGE("UDC name not defined");
    return Status::ERROR;
  }

  if (rmnetInst.empty()) {
    ALOGE("rmnetinstance not defined");
    rmnetInst = "rmnet";
  }

  if (dplInst.empty()) {
    ALOGE("dplinstance not defined");
    dplInst = "dpl";
  }

  rmnetInst = rmnetFunc + "." + rmnetInst;
  dplInst = rmnetFunc + "." + dplInst;

  if ((functions & GadgetFunction::MTP) != 0) {
    ALOGI("setCurrentUsbFunctions mtp");
    if (!WriteStringToFile("1", DESC_USE_PATH)) return Status::ERROR;
    if (linkFunction("mtp.gs0", i++)) return Status::ERROR;
  }

  if ((functions & GadgetFunction::PTP) != 0) {
    ALOGI("setCurrentUsbFunctions ptp");
    if (!WriteStringToFile("1", DESC_USE_PATH)) return Status::ERROR;
    if (linkFunction("ptp.gs1", i++)) return Status::ERROR;
  }

  if ((functions & GadgetFunction::MIDI) != 0) {
    ALOGI("setCurrentUsbFunctions MIDI");
    if (linkFunction("midi.gs5", i++)) return Status::ERROR;;
  }

  if ((functions & GadgetFunction::ACCESSORY) != 0) {
    ALOGI("setCurrentUsbFunctions Accessory");
    if (linkFunction("accessory.gs2", i++)) return Status::ERROR;
  }

  if ((functions & GadgetFunction::AUDIO_SOURCE) != 0) {
    ALOGI("setCurrentUsbFunctions Audio Source");
    if (linkFunction("audio_source.gs3", i++)) return Status::ERROR;
  }

  mtype = getModemType();
  if ((functions & GadgetFunction::RNDIS) != 0) {
    ALOGI("setCurrentUsbFunctions rndis");
    if (linkFunction((rndisFunc + ".rndis").c_str(), i++))
      return Status::ERROR;
    if (functions & GadgetFunction::ADB) {
      if (mtype == EXTERNAL || mtype == INTERNAL_EXTERNAL) {
        ALOGI("esoc RNDIS default composition");
        if (linkFunction("diag.diag", i++)) return Status::ERROR;
        if (linkFunction("diag.diag_mdm", i++)) return Status::ERROR;
        if (linkFunction("qdss.qdss", i++)) return Status::ERROR;
        if (linkFunction("qdss.qdss_mdm", i++)) return Status::ERROR;
        if (linkFunction("cser.dun.0", i++)) return Status::ERROR;
        if (linkFunction(dplInst.c_str(), i++))
          return Status::ERROR;
        if (setVidPid("0x05c6", "0x90e7") != Status::SUCCESS)
          return Status::ERROR;
      } else if (mtype == INTERNAL) {
        ALOGI("RNDIS default composition");
        if (linkFunction("diag.diag", i++)) return Status::ERROR;
        if (linkFunction("qdss.qdss", i++)) return Status::ERROR;
        if (linkFunction("cser.dun.0", i++)) return Status::ERROR;
        if (linkFunction(dplInst.c_str(), i++))
          return Status::ERROR;
        if (setVidPid("0x05c6", "0x90e9") != Status::SUCCESS)
          return Status::ERROR;
      }
    }
  }

  /* override adb-only with additional QTI functions */
  if (i == 0 && functions & GadgetFunction::ADB) {
    /* vendor defined functions if any run from vendor rc file */
    if (!vendorProp.empty()) {
      ALOGI("enable vendor usb config composition");
      SetProperty("vendor.usb.config", vendorProp);
      return Status::SUCCESS;
    }

    if (mtype == EXTERNAL || mtype == INTERNAL_EXTERNAL) {
      ALOGI("esoc default composition");
      if (linkFunction("diag.diag", i++)) return Status::ERROR;
      if (linkFunction("diag.diag_mdm", i++)) return Status::ERROR;
      if (linkFunction("qdss.qdss", i++)) return Status::ERROR;
      if (linkFunction("qdss.qdss_mdm", i++)) return Status::ERROR;
      if (linkFunction("cser.dun.0", i++)) return Status::ERROR;
      if (linkFunction(dplInst.c_str(), i++))
        return Status::ERROR;
      if (linkFunction(rmnetInst.c_str(), i++))
        return Status::ERROR;
      if (setVidPid("0x05c6", "0x90e5") != Status::SUCCESS)
        return Status::ERROR;
    } else if (mtype == NONE) {
      ALOGI("enable APQ default composition");
      if (linkFunction("diag.diag", i++)) return Status::ERROR;
      if (setVidPid("0x05c6", "0x901d") != Status::SUCCESS)
        return Status::ERROR;
    } else {
      ALOGI("enable QC default composition");
      if (linkFunction("diag.diag", i++)) return Status::ERROR;
      if (linkFunction("cser.dun.0", i++)) return Status::ERROR;
      if (linkFunction(rmnetInst.c_str(), i++))
        return Status::ERROR;
      if (linkFunction(dplInst.c_str(), i++))
        return Status::ERROR;
      if (linkFunction("qdss.qdss", i++)) return Status::ERROR;
      if (setVidPid("0x05c6", "0x90db") != Status::SUCCESS)
        return Status::ERROR;
    }
  }

  if ((functions & GadgetFunction::ADB) != 0) {
    ffsEnabled = true;
    ALOGI("setCurrentUsbFunctions Adb");
    if (inotify_add_watch(inotifyFd, "/dev/usb-ffs/adb/", IN_ALL_EVENTS) == -1)
      return Status::ERROR;

    if (linkFunction("ffs.adb", i++)) return Status::ERROR;
    mEndpointList.push_back("/dev/usb-ffs/adb/ep1");
    mEndpointList.push_back("/dev/usb-ffs/adb/ep2");
    ALOGI("Service started");
  }

  // Pull up the gadget right away when there are no ffs functions.
  if (!ffsEnabled) {
    if (!WriteStringToFile(gadgetName, PULLUP_PATH)) return Status::ERROR;
    mCurrentUsbFunctionsApplied = true;
    if (callback)
      callback->setCurrentUsbFunctionsCb(functions, Status::SUCCESS);
    return Status::SUCCESS;
  }

  unique_fd eventFd(eventfd(0, 0));
  if (eventFd == -1) {
    ALOGE("mEventFd failed to create %d", errno);
    return Status::ERROR;
  }

  unique_fd epollFd(epoll_create(2));
  if (epollFd == -1) {
    ALOGE("mEpollFd failed to create %d", errno);
    return Status::ERROR;
  }

  if (addEpollFd(epollFd, inotifyFd) == -1) return Status::ERROR;

  if (addEpollFd(epollFd, eventFd) == -1) return Status::ERROR;

  mEpollFd = move(epollFd);
  mInotifyFd = move(inotifyFd);
  mEventFd = move(eventFd);
  gadgetPullup = false;

  // Monitors the ffs paths to pull up the gadget when descriptors are written.
  // Also takes of the pulling up the gadget again if the userspace process
  // dies and restarts.
  mMonitor = unique_ptr<thread>(new thread(monitorFfs, this));
  mMonitorCreated = true;
  if (DEBUG) ALOGI("Mainthread in Cv");

  if (callback) {
    if (mCv.wait_for(lk, timeout * 1ms, [] { return gadgetPullup; })) {
      ALOGI("monitorFfs signalled true");
    } else {
      ALOGI("monitorFfs signalled error");
      // continue monitoring as the descriptors might be written at a later
      // point.
    }
    Return<void> ret = callback->setCurrentUsbFunctionsCb(
        functions, gadgetPullup ? Status::SUCCESS : Status::ERROR);
    if (!ret.isOk())
      ALOGE("setCurrentUsbFunctionsCb error %s", ret.description().c_str());
  }

  return Status::SUCCESS;
}

Return<void> UsbGadget::setCurrentUsbFunctions(
    uint64_t functions, const sp<V1_0::IUsbGadgetCallback> &callback,
    uint64_t timeout) {
  std::unique_lock<std::mutex> lk(mLockSetCurrentFunction);

  mCurrentUsbFunctions = functions;
  mCurrentUsbFunctionsApplied = false;

  // Unlink the gadget and stop the monitor if running.
  V1_0::Status status = tearDownGadget();
  if (status != Status::SUCCESS) {
    goto error;
  }

  // Leave the gadget pulled down to give time for the host to sense disconnect.
  usleep(DISCONNECT_WAIT_US);

  if (functions == static_cast<uint64_t>(GadgetFunction::NONE)) {
    if (callback == NULL) return Void();
    Return<void> ret =
        callback->setCurrentUsbFunctionsCb(functions, Status::SUCCESS);
    if (!ret.isOk())
      ALOGE("Error while calling setCurrentUsbFunctionsCb %s",
            ret.description().c_str());
    return Void();
  }

  status = validateAndSetVidPid(functions);

  if (status != Status::SUCCESS) {
    goto error;
  }

  status = setupFunctions(functions, callback, timeout);
  if (status != Status::SUCCESS) {
    goto error;
  }

  ALOGI("Usb Gadget setcurrent functions called successfully");
  return Void();

error:
  ALOGI("Usb Gadget setcurrent functions failed");
  if (callback == NULL) return Void();
  Return<void> ret = callback->setCurrentUsbFunctionsCb(functions, status);
  if (!ret.isOk())
    ALOGE("Error while calling setCurrentUsbFunctionsCb %s",
          ret.description().c_str());
  return Void();
}
}  // namespace implementation
}  // namespace V1_0
}  // namespace gadget
}  // namespace usb
}  // namespace hardware
}  // namespace android
