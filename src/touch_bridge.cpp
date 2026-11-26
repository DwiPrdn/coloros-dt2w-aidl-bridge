#include <android/binder_ibinder.h>
#include <android/binder_parcel.h>
#include <android/binder_status.h>
#include <android/log.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#define LOG_TAG "OplusTouchAidlBridge"
#include "bridge_log.h"

// Platform LL-NDK libbinder_ndk symbols
extern "C" {
binder_status_t AServiceManager_addService(AIBinder* binder, const char* instance);
bool ABinderProcess_setThreadPoolMaxThreadCount(uint32_t numThreads);
void ABinderProcess_startThreadPool(void);
void ABinderProcess_joinThreadPool(void);
void AIBinder_markVintfStability(AIBinder* binder);
}

// ============================================================================
// Xiaomi /dev/xiaomi-touch + ft3519t.ko Gesture Control
//   1. /dev/xiaomi-touch ioctl 0x5400 mode 14 (Touch_Doubletap_Mode) -> gesture_mode bit 0.
//   2. ft3519t.ko fts_gesture_switch (input_dev->event on /dev/input/event* "fts_ts"):
//      EV_SYN (0) / SYN_CONFIG (1) / value=5 (on) or 4 (off) -> sets [fts_data + 0x5a8]
//      (double_click_on), which fts_ts_suspend() requires to keep 0xD0/0xCF enabled when
//      FOD (fod_status >= 1) is active.
//   3. During SOFOD/AOD (STATE_DOZE with ro.vendor.mtk_aod_support=0), SurfaceFlinger maps
//      STATE_DOZE to HWC POWER_MODE_ON, waking ft3519t.ko out of hardware gesture suspend so
//      it emits raw EV_ABS touchscreen events that InputDispatcher drops ("no touchable
//      window"). While gInDoze && dt2wEnabled, DozeTapMonitorThread detects crisp single-
//      finger double-taps on fts_ts and injects KEY_WAKEUP (143 -> KEYCODE_WAKEUP 224).
// ============================================================================
static constexpr const char* kXiaomiTouchDev = "/dev/xiaomi-touch";
static constexpr unsigned long TOUCH_IOC_SET_CUR_VALUE = 0x5400UL;
static constexpr int32_t TOUCH_MODE_DOUBLETAP = 14;  // Touch_Doubletap_Mode

static constexpr int32_t kLogicalWidth = 1080;
static constexpr int32_t kLogicalHeight = 2400;

// ============================================================================
// ColorOS 17 vendor.oplus.hardware.touch.IOplusTouch V2 AIDL Constants
// Verified from /system_ext/framework/vendor.oplus.hardware.touch-V2-java.jar
// and oplus-services.jar ColorTouchNodeManagerService / ColorTouchNodeController
// ============================================================================
static constexpr const char* kIOplusTouchDescriptor = "vendor.oplus.hardware.touch.IOplusTouch";
static constexpr const char* kIOplusTouchInstance = "vendor.oplus.hardware.touch.IOplusTouch/default";
static constexpr int32_t kAidlTouchVersion = 2;
static constexpr const char* kAidlTouchHash = "ef41c5fab372bb5ad6a417f606845c87cf0e9b17";

static constexpr transaction_code_t kTxInitialize = 1;
static constexpr transaction_code_t kTxIsTouchNodeSupport = 2;
static constexpr transaction_code_t kTxTouchReadNodeFile = 3;
static constexpr transaction_code_t kTxTouchWriteNodeFile = 4;
static constexpr transaction_code_t kTxTouchWriteBtInfo = 5;
static constexpr transaction_code_t kTxTouchWriteNodeFileOneWay = 6;
static constexpr transaction_code_t kTxTouchNotifyClient = 7;
static constexpr transaction_code_t kTxRegisterEventCallback = 8;
static constexpr transaction_code_t kTxUnregisterEventCallback = 9;
static constexpr transaction_code_t kTxGetInterfaceVersion = 16777215;
static constexpr transaction_code_t kTxGetInterfaceHash = 16777214;

// OplusTouchNodeManager nodeFlag constants (verified in ColorTouchNodeController & OplusScreenOffGestureUtil)
static constexpr int32_t kNodeDoubleTapEnable = 1;  // /proc/touchpanel/double_tap_enable
static constexpr int32_t kNodeCoordinate = 2;       // /proc/touchpanel/coordinate
static constexpr int32_t kNodeDoubleTapIndep = 21;  // /proc/touchpanel/double_tap_enable_indep

namespace {

static std::mutex gStateMutex;
static std::map<int32_t, std::string> gNodeValues;
static int32_t gLastDoubleTapMode = -1;
static int32_t gLastGestureSwitchMode = -1;
static uint32_t gLastIndepMask = 0;
static bool gNode1Enabled = false;
static bool gNode1ExplicitOff = false;
static bool gIndepDt2wEnabled = false;
static bool gDryRun = false;

// Doze/SOFOD monitor coordination (0% CPU when screen is awake/interactive)
static std::mutex gDozeMutex;
static std::condition_variable gDozeCv;
static bool gInDoze = false;
static bool gDt2wActive = false;
static std::atomic<bool> gMonitorRunning{false};

static int64_t getMonoMillis() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000LL + static_cast<int64_t>(ts.tv_nsec) / 1000000LL;
}

static bool testBit(const unsigned long* arr, int bit) {
    return (arr[bit / (sizeof(unsigned long) * 8)] &
            (1UL << (bit % (sizeof(unsigned long) * 8)))) != 0;
}

// ============================================================================
// Pure, Self-Testable In-Doze Software Double-Tap Detector
//   - Operates on logical 1080x2400 coordinates normalized from EVIOCGABS.
//   - Triggers on tap 2 release (UP) to reject FOD press-and-hold retries cleanly.
//   - Rejects multi-touch, swipes, long holds (>260ms), distant taps (>200px).
//   - Automatically resets & debounces if firmware hardware DT2W (KEY_POWER) fires.
// ============================================================================
struct DozeDoubleTapDetector {
    static constexpr int64_t kMaxTapDurationMs = 260;
    static constexpr int64_t kMinGapFromUpMs = 10;
    static constexpr int64_t kMaxGapFromUpMs = 380;
    static constexpr int64_t kMaxGapFromDownMs = 500;
    static constexpr int64_t kWakeDebounceMs = 500;
    static constexpr int32_t kMaxTapDriftPx = 90;
    static constexpr int32_t kMaxPairDistPx = 200;
    static constexpr int kMaxSlots = 10;

    int32_t absMaxX = kLogicalWidth;
    int32_t absMaxY = kLogicalHeight;

    int32_t curSlot = 0;
    int32_t slotId[kMaxSlots];
    int32_t slotX[kMaxSlots];
    int32_t slotY[kMaxSlots];

    bool inTap = false;
    bool tapMultiTouch = false;
    int64_t tapDownMs = 0;
    int32_t tapStartX = 0;
    int32_t tapStartY = 0;
    int64_t tapMaxDriftSq = 0;

    bool hasFirstTap = false;
    int64_t firstTapDownMs = 0;
    int64_t firstTapUpMs = 0;
    int32_t firstTapX = 0;
    int32_t firstTapY = 0;

    int64_t lastWakeMs = -10000;

    DozeDoubleTapDetector() { resetAll(); }

    void setAbsRange(int32_t maxX, int32_t maxY) {
        absMaxX = (maxX > 0) ? maxX : kLogicalWidth;
        absMaxY = (maxY > 0) ? maxY : kLogicalHeight;
    }

    void resetTouchState() {
        curSlot = 0;
        for (int i = 0; i < kMaxSlots; ++i) {
            slotId[i] = -1;
            slotX[i] = 0;
            slotY[i] = 0;
        }
        inTap = false;
        tapMultiTouch = false;
        tapDownMs = 0;
        tapStartX = 0;
        tapStartY = 0;
        tapMaxDriftSq = 0;
    }

    void resetAll() {
        resetTouchState();
        hasFirstTap = false;
        firstTapDownMs = 0;
        firstTapUpMs = 0;
        firstTapX = 0;
        firstTapY = 0;
    }

    void onHardwareWake(int64_t nowMs) {
        resetAll();
        lastWakeMs = nowMs;
    }

    int32_t toLogicalX(int32_t rawX) const {
        return static_cast<int32_t>((static_cast<int64_t>(rawX) * kLogicalWidth) / absMaxX);
    }

    int32_t toLogicalY(int32_t rawY) const {
        return static_cast<int32_t>((static_cast<int64_t>(rawY) * kLogicalHeight) / absMaxY);
    }

    // Feed a Linux input_event; returns true iff a clean double-tap just completed.
    bool processEvent(uint16_t type, uint16_t code, int32_t value, int64_t nowMs) {
        if (type == EV_KEY) {
            if (code == KEY_POWER && value == 1) {
                // Hardware firmware DT2W already woke the device; clear state & debounce.
                onHardwareWake(nowMs);
            }
            return false;
        }
        if (type == EV_ABS) {
            if (code == ABS_MT_SLOT) {
                if (value >= 0 && value < kMaxSlots) {
                    curSlot = value;
                }
            } else if (code == ABS_MT_TRACKING_ID) {
                if (curSlot >= 0 && curSlot < kMaxSlots) {
                    slotId[curSlot] = value;
                }
            } else if (code == ABS_MT_POSITION_X) {
                if (curSlot >= 0 && curSlot < kMaxSlots) {
                    slotX[curSlot] = toLogicalX(value);
                }
            } else if (code == ABS_MT_POSITION_Y) {
                if (curSlot >= 0 && curSlot < kMaxSlots) {
                    slotY[curSlot] = toLogicalY(value);
                }
            }
            return false;
        }
        if (type == EV_SYN) {
            if (code == SYN_DROPPED) {
                resetAll();
                return false;
            }
            if (code != SYN_REPORT) {
                return false;
            }
            int activeCount = 0;
            int activeIdx = -1;
            for (int i = 0; i < kMaxSlots; ++i) {
                if (slotId[i] >= 0) {
                    ++activeCount;
                    if (activeIdx < 0) activeIdx = i;
                }
            }
            if (activeCount > 0) {
                if (!inTap) {
                    inTap = true;
                    tapMultiTouch = (activeCount > 1);
                    tapDownMs = nowMs;
                    tapStartX = slotX[activeIdx];
                    tapStartY = slotY[activeIdx];
                    tapMaxDriftSq = 0;
                } else {
                    if (activeCount > 1) {
                        tapMultiTouch = true;
                    } else if (!tapMultiTouch && activeIdx >= 0) {
                        int64_t dx = static_cast<int64_t>(slotX[activeIdx]) - tapStartX;
                        int64_t dy = static_cast<int64_t>(slotY[activeIdx]) - tapStartY;
                        int64_t d2 = dx * dx + dy * dy;
                        if (d2 > tapMaxDriftSq) tapMaxDriftSq = d2;
                    }
                }
                return false;
            }

            // activeCount == 0: check if a touch just ended
            if (!inTap) {
                return false;
            }
            inTap = false;
            int64_t dur = nowMs - tapDownMs;
            int64_t maxDriftSq = static_cast<int64_t>(kMaxTapDriftPx) * kMaxTapDriftPx;
            bool validSingleTap = !tapMultiTouch &&
                                  dur >= 0 && dur <= kMaxTapDurationMs &&
                                  tapMaxDriftSq <= maxDriftSq;
            if (!validSingleTap) {
                hasFirstTap = false;
                return false;
            }

            if (hasFirstTap) {
                int64_t gapFromUp = tapDownMs - firstTapUpMs;
                int64_t gapFromDown = tapDownMs - firstTapDownMs;
                int64_t dx = static_cast<int64_t>(tapStartX) - firstTapX;
                int64_t dy = static_cast<int64_t>(tapStartY) - firstTapY;
                int64_t pairDistSq = dx * dx + dy * dy;
                int64_t maxPairDistSq = static_cast<int64_t>(kMaxPairDistPx) * kMaxPairDistPx;

                if (gapFromUp >= kMinGapFromUpMs && gapFromUp <= kMaxGapFromUpMs &&
                    gapFromDown <= kMaxGapFromDownMs && pairDistSq <= maxPairDistSq) {
                    hasFirstTap = false;
                    if (nowMs - lastWakeMs >= kWakeDebounceMs) {
                        lastWakeMs = nowMs;
                        return true;
                    }
                    return false;
                }
            }

            // Record this valid single tap as candidate first tap
            hasFirstTap = true;
            firstTapDownMs = tapDownMs;
            firstTapUpMs = nowMs;
            firstTapX = tapStartX;
            firstTapY = tapStartY;
            return false;
        }
        return false;
    }
};

// ============================================================================
// fts_ts Evdev Discovery, fts_gesture_switch, and KEY_WAKEUP Injection
// ============================================================================
static int openFtsInputDevice(int flags, int32_t* outMaxX, int32_t* outMaxY,
                              bool* outSupportsWakeup, std::string* outPath) {
    char path[64];
    int fallbackFd = -1;
    int32_t fbMaxX = kLogicalWidth, fbMaxY = kLogicalHeight;
    bool fbWake = false;
    std::string fbPath;

    for (int i = 0; i < 16; ++i) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, flags | O_CLOEXEC);
        if (fd < 0) continue;

        char name[128] = {0};
        ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name);

        unsigned long evBits[(EV_MAX / (sizeof(unsigned long) * 8)) + 1] = {0};
        unsigned long absBits[(ABS_MAX / (sizeof(unsigned long) * 8)) + 1] = {0};
        unsigned long keyBits[(KEY_MAX / (sizeof(unsigned long) * 8)) + 1] = {0};

        bool hasAbs = false;
        if (ioctl(fd, EVIOCGBIT(0, sizeof(evBits)), evBits) >= 0 && testBit(evBits, EV_ABS) &&
            ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(absBits)), absBits) >= 0) {
            hasAbs = testBit(absBits, ABS_MT_POSITION_X) && testBit(absBits, ABS_MT_POSITION_Y);
        }
        if (!hasAbs) {
            close(fd);
            continue;
        }

        input_absinfo ax{}, ay{};
        ioctl(fd, EVIOCGABS(ABS_MT_POSITION_X), &ax);
        ioctl(fd, EVIOCGABS(ABS_MT_POSITION_Y), &ay);
        int32_t mx = (ax.maximum > 0) ? ax.maximum : kLogicalWidth;
        int32_t my = (ay.maximum > 0) ? ay.maximum : kLogicalHeight;

        bool hasWake = false;
        if (testBit(evBits, EV_KEY) &&
            ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keyBits)), keyBits) >= 0) {
            hasWake = testBit(keyBits, KEY_WAKEUP);
        }

        if (strstr(name, "fts_ts") != nullptr) {
            if (fallbackFd >= 0) close(fallbackFd);
            if (outMaxX) *outMaxX = mx;
            if (outMaxY) *outMaxY = my;
            if (outSupportsWakeup) *outSupportsWakeup = hasWake;
            if (outPath) *outPath = path;
            return fd;
        }
        if (fallbackFd < 0) {
            fallbackFd = fd;
            fbMaxX = mx;
            fbMaxY = my;
            fbWake = hasWake;
            fbPath = path;
        } else {
            close(fd);
        }
    }

    if (fallbackFd >= 0) {
        if (outMaxX) *outMaxX = fbMaxX;
        if (outMaxY) *outMaxY = fbMaxY;
        if (outSupportsWakeup) *outSupportsWakeup = fbWake;
        if (outPath) *outPath = fbPath;
    }
    return fallbackFd;
}

static bool sendFtsGestureSwitch(bool enable) {
    int32_t swVal = enable ? 5 : 4;
    if (gDryRun) {
        gLastGestureSwitchMode = swVal;
        return true;
    }
    std::string path;
    int fd = openFtsInputDevice(O_WRONLY | O_NONBLOCK, nullptr, nullptr, nullptr, &path);
    if (fd < 0) {
        ALOGW("sendFtsGestureSwitch(%d): could not open fts_ts input device (errno=%d: %s)",
              swVal, errno, strerror(errno));
        return false;
    }
    struct input_event ev{};
    ev.type = EV_SYN;
    ev.code = SYN_CONFIG;
    ev.value = swVal;
    ssize_t n = write(fd, &ev, sizeof(ev));
    int savedErrno = errno;
    close(fd);
    if (n != static_cast<ssize_t>(sizeof(ev))) {
        ALOGW("sendFtsGestureSwitch(%s, SYN_CONFIG=%d) failed: n=%zd errno=%d (%s)",
              path.c_str(), swVal, n, savedErrno, strerror(savedErrno));
        return false;
    }
    gLastGestureSwitchMode = swVal;
    ALOGI("fts_gesture_switch(%s, SYN_CONFIG=%d) OK", path.c_str(), swVal);
    return true;
}

static int gUinputFd = -1;

static int ensureUinputWakeDevice() {
    if (gUinputFd >= 0) return gUinputFd;
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        ALOGW("open(/dev/uinput) failed: errno=%d (%s)", errno, strerror(errno));
        return -1;
    }
    if (ioctl(fd, UI_SET_EVBIT, EV_KEY) < 0 ||
        ioctl(fd, UI_SET_EVBIT, EV_SYN) < 0 ||
        ioctl(fd, UI_SET_KEYBIT, KEY_WAKEUP) < 0) {
        ALOGW("uinput UI_SET_* failed: errno=%d (%s)", errno, strerror(errno));
        close(fd);
        return -1;
    }
    struct uinput_setup usetup{};
    snprintf(usetup.name, sizeof(usetup.name), "cos_dt2w_wake");
    usetup.id.bustype = BUS_VIRTUAL;
    usetup.id.vendor = 0x18d1;
    usetup.id.product = 0xd720;
    usetup.id.version = 1;
    if (ioctl(fd, UI_DEV_SETUP, &usetup) < 0 || ioctl(fd, UI_DEV_CREATE) < 0) {
        ALOGW("uinput UI_DEV_SETUP/CREATE failed: errno=%d (%s)", errno, strerror(errno));
        close(fd);
        return -1;
    }
    // Give EventHub a brief moment to open the new uinput node via inotify
    usleep(80000);
    gUinputFd = fd;
    ALOGI("Created fallback /dev/uinput wake device 'cos_dt2w_wake' (fd=%d)", fd);
    return fd;
}

static bool writeWakeKeyPulse(int fd, const char* tag) {
    struct input_event evs[2]{};
    evs[0].type = EV_KEY;
    evs[0].code = KEY_WAKEUP;
    evs[0].value = 1;
    evs[1].type = EV_SYN;
    evs[1].code = SYN_REPORT;
    evs[1].value = 0;
    if (write(fd, evs, sizeof(evs)) != static_cast<ssize_t>(sizeof(evs))) {
        ALOGW("writeWakeKeyPulse(%s) DOWN failed: errno=%d (%s)", tag, errno, strerror(errno));
        return false;
    }
    usleep(12000);  // 12ms key press pulse
    evs[0].value = 0;
    if (write(fd, evs, sizeof(evs)) != static_cast<ssize_t>(sizeof(evs))) {
        ALOGW("writeWakeKeyPulse(%s) UP failed: errno=%d (%s)", tag, errno, strerror(errno));
        return false;
    }
    ALOGI("DozeTapMonitor: injected KEY_WAKEUP (143) via %s OK", tag);
    return true;
}

static void injectWakeupKey(int ftsRwFd, bool ftsSupportsWakeup) {
    if (ftsRwFd >= 0 && ftsSupportsWakeup) {
        if (writeWakeKeyPulse(ftsRwFd, "fts_ts")) {
            return;
        }
    }
    int ufd = ensureUinputWakeDevice();
    if (ufd >= 0) {
        writeWakeKeyPulse(ufd, "uinput");
    }
}

static void updateDozeMonitorState(bool setDoze, bool dozeVal, bool setDt2w, bool dt2wVal) {
    bool changed = false;
    {
        std::lock_guard<std::mutex> lk(gDozeMutex);
        if (setDoze && gInDoze != dozeVal) {
            gInDoze = dozeVal;
            changed = true;
        }
        if (setDt2w && gDt2wActive != dt2wVal) {
            gDt2wActive = dt2wVal;
            changed = true;
        }
    }
    if (changed) {
        gDozeCv.notify_all();
    }
}

static void dozeTapMonitorLoop() {
    ALOGI("DozeTapMonitorThread started (idle until Doze/SOFOD + DT2W active)");
    DozeDoubleTapDetector detector;

    while (true) {
        {
            std::unique_lock<std::mutex> lk(gDozeMutex);
            gDozeCv.wait(lk, []() { return gInDoze && gDt2wActive; });
        }

        int32_t maxX = kLogicalWidth, maxY = kLogicalHeight;
        bool supportsWake = false;
        std::string devPath;
        int fd = openFtsInputDevice(O_RDWR | O_NONBLOCK, &maxX, &maxY, &supportsWake, &devPath);
        if (fd < 0) {
            // Fallback to O_RDONLY if write permission unavailable, then use uinput for wake
            fd = openFtsInputDevice(O_RDONLY | O_NONBLOCK, &maxX, &maxY, &supportsWake, &devPath);
            supportsWake = false;
        }
        if (fd < 0) {
            ALOGW("DozeTapMonitor: failed to open fts_ts (errno=%d: %s), retrying...",
                  errno, strerror(errno));
            usleep(500000);
            continue;
        }

        detector.setAbsRange(maxX, maxY);
        detector.resetAll();

        // Drain any stale pre-Doze touch events already queued before we started monitoring
        struct input_event drainBuf[32];
        while (read(fd, drainBuf, sizeof(drainBuf)) > 0) {}

        ALOGI("DozeTapMonitor: active on %s (absMax=%dx%d, wakeKey=%d)",
              devPath.c_str(), maxX, maxY, supportsWake ? 1 : 0);

        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        struct input_event evs[32];
        while (true) {
            {
                std::lock_guard<std::mutex> lk(gDozeMutex);
                if (!gInDoze || !gDt2wActive) {
                    break;
                }
            }
            pfd.revents = 0;
            int pr = poll(&pfd, 1, 200);
            if (pr < 0) {
                if (errno == EINTR) continue;
                ALOGW("DozeTapMonitor: poll error errno=%d (%s)", errno, strerror(errno));
                break;
            }
            if (pr == 0) {
                continue;  // Periodic re-check of gInDoze
            }
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                ALOGW("DozeTapMonitor: fd revents=0x%x, reopening", pfd.revents);
                break;
            }
            if (pfd.revents & POLLIN) {
                while (true) {
                    ssize_t n = read(fd, evs, sizeof(evs));
                    if (n <= 0) break;
                    size_t count = static_cast<size_t>(n) / sizeof(struct input_event);
                    int64_t nowMs = getMonoMillis();
                    for (size_t i = 0; i < count; ++i) {
                        if (detector.processEvent(evs[i].type, evs[i].code, evs[i].value, nowMs)) {
                            ALOGI("DozeTapMonitor: SOFOD/AOD double-tap detected! Waking screen...");
                            injectWakeupKey(fd, supportsWake);
                        }
                    }
                }
            }
        }

        close(fd);
        ALOGI("DozeTapMonitor: inactive (exited Doze or DT2W disabled)");
    }
}

static void startDozeTapMonitorOnce() {
    bool expected = false;
    if (gMonitorRunning.compare_exchange_strong(expected, true)) {
        std::thread(dozeTapMonitorLoop).detach();
    }
}

// ============================================================================
// Binder Helpers & IOplusTouch Node Logic
// ============================================================================
static void writeOkStatus(AParcel* out) {
    AStatus* st = AStatus_newOk();
    AParcel_writeStatusHeader(out, st);
    AStatus_delete(st);
}

static bool readStringStd(const AParcel* in, std::string& out) {
    auto alloc = [](void* strPtr, int32_t len, char** buf) -> bool {
        auto* s = static_cast<std::string*>(strPtr);
        if (len <= 0) {
            s->clear();
            *buf = nullptr;
            return true;
        }
        s->resize(static_cast<size_t>(len - 1));
        *buf = s->data();
        return true;
    };
    return AParcel_readString(in, &out, alloc) == STATUS_OK;
}

static bool skipParcelable(const AParcel* in) {
    int32_t nonNull = 0;
    if (AParcel_readInt32(in, &nonNull) != STATUS_OK) return false;
    if (nonNull == 0) return true;
    int32_t startPos = AParcel_getDataPosition(in);
    int32_t size = 0;
    if (AParcel_readInt32(in, &size) != STATUS_OK || size < 4) return false;
    return AParcel_setDataPosition(in, startPos + size) == STATUS_OK;
}

static bool setXiaomiDoubleTapMode(bool enable) {
    int32_t val = enable ? 1 : 0;
    updateDozeMonitorState(false, false, true, enable);

    // Always also drive ft3519t.ko fts_gesture_switch (EV_SYN/SYN_CONFIG 5/4) so
    // [fts_data + 0x5a8] (double_click_on) stays synchronized even with FOD enabled.
    sendFtsGestureSwitch(enable);

    if (gDryRun) {
        gLastDoubleTapMode = val;
        return true;
    }
    int fd = open(kXiaomiTouchDev, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        ALOGW("open(%s) failed: errno=%d (%s)", kXiaomiTouchDev, errno, strerror(errno));
        return false;
    }
    int32_t buf[4] = {TOUCH_MODE_DOUBLETAP, val, 0, 0};
    int ret = ioctl(fd, TOUCH_IOC_SET_CUR_VALUE, buf);
    close(fd);
    if (ret != 0) {
        ALOGW("ioctl(%s, 0x5400, {14, %d}) failed: ret=%d errno=%d (%s)",
              kXiaomiTouchDev, val, ret, errno, strerror(errno));
        return false;
    }
    gLastDoubleTapMode = val;
    ALOGI("Touch_Doubletap_Mode -> %d OK", val);
    return true;
}

static long parseNodeNumericValue(const std::string& info) {
    if (info.empty()) return 0;
    char* endp = nullptr;
    long v = strtol(info.c_str(), &endp, 0);
    if (endp != info.c_str()) {
        return v;
    }
    return (info[0] != '0') ? 1L : 0L;
}

static int32_t handleWriteNode(int32_t deviceId, int32_t nodeFlag, const std::string& info) {
    ALOGI("touchWriteNode(dId=%d, nodeFlag=%d, info='%s')", deviceId, nodeFlag, info.c_str());
    if (deviceId != 0) {
        return -2;
    }
    std::lock_guard<std::mutex> lock(gStateMutex);
    if (nodeFlag == kNodeDoubleTapIndep) {
        // ColorOS ScreenOffGestureService.B1 writes decimal bitmask string, while R0 reads it back
        // via Integer.parseInt(val, 16). Store canonical hex so R0 round-trips accurately.
        // Bit 1 (0x02) = double-tap; bit 0 (0x01) = legacy double-tap; bit 16 (0x10000) = Doze/AOD.
        // Verified timeline on ColorOS 17:
        //   - On entering sleep/Doze (OplusDozeService onDreamingStarted): writes "65536" (0x10000).
        //   - On waking up interactive (onDreamingStopped / onScreenOn): writes "2" (0x2).
        //   - Power-saving AOD fadeout writes "0" after "65536" while master DT2W node1=1 stays ON.
        uint32_t mask = static_cast<uint32_t>(parseNodeNumericValue(info));
        uint32_t prevMask = gLastIndepMask;
        gLastIndepMask = mask;
        char hexBuf[16];
        snprintf(hexBuf, sizeof(hexBuf), "%x", mask);
        gNodeValues[nodeFlag] = hexBuf;
        if ((mask & 0x3u) != 0) {
            gIndepDt2wEnabled = true;
            // Pure DT2W mask (e.g. 0x2) means screen woke interactive; 0x10002 means AOD+DT2W.
            bool inDozeNow = ((mask & 0x10000u) != 0);
            updateDozeMonitorState(true, inDozeNow, false, false);
            if (!gNode1ExplicitOff) {
                setXiaomiDoubleTapMode(true);
            }
        } else if ((mask & 0x10000u) != 0) {
            // Entering Doze/SOFOD/AOD (0x10000 = "65536"): preserve DT2W AND arm Doze tap monitor.
            updateDozeMonitorState(true, true, false, false);
            ALOGI("double_tap_enable_indep AOD/Doze mask=0x%x: preserving Touch_Doubletap_Mode=%d (doze=1)",
                  mask, gLastDoubleTapMode);
        } else if (mask == 0) {
            if (prevMask == 0x10000u && gNode1Enabled) {
                // Power-saving AOD fading out after 0x10000 while master DT2W nodeFlag=1 is ON.
                // Keep doze=1 until actual interactive wakeup writes mask=2 (0% CPU in STATE_OFF).
                ALOGI("double_tap_enable_indep AOD-off (0 after 0x10000) with node1=1: preserving DT2W=%d",
                      gLastDoubleTapMode);
            } else {
                gIndepDt2wEnabled = false;
                if (!gNode1Enabled) {
                    updateDozeMonitorState(true, false, false, false);
                    setXiaomiDoubleTapMode(false);
                }
            }
        } else {
            // Non-zero mask without DT2W bits (e.g. another gesture without DT2W)
            gIndepDt2wEnabled = false;
            if (!gNode1Enabled) {
                updateDozeMonitorState(true, false, false, false);
                setXiaomiDoubleTapMode(false);
            }
        }
        return 0;
    }

    gNodeValues[nodeFlag] = info;
    if (nodeFlag == kNodeDoubleTapEnable) {
        bool en = (parseNodeNumericValue(info) != 0);
        gNode1Enabled = en;
        gNode1ExplicitOff = !en;
        if (!en) {
            gIndepDt2wEnabled = false;
            updateDozeMonitorState(true, false, false, false);
        }
        setXiaomiDoubleTapMode(en);
    }
    return 0;
}

static std::string handleReadNode(int32_t deviceId, int32_t nodeFlag) {
    if (deviceId != 0) {
        return "";
    }
    std::lock_guard<std::mutex> lock(gStateMutex);
    auto it = gNodeValues.find(nodeFlag);
    if (it != gNodeValues.end()) {
        return it->second;
    }
    if (nodeFlag == kNodeCoordinate) {
        // Valid dummy coordinate format for OplusScreenOffGestureUtil.updateGestureInfo()
        return "1,540:1200,540:1200,540:1200,540:1200,540:1200,540:1200,0";
    }
    return "0";
}

static int32_t handleIsNodeSupport(int32_t deviceId, int32_t nodeFlag) {
    // CRITICAL: ColorTouchNodeManagerService.isTouchNodeSupport treats ret > 0 as true, <= 0 as false.
    int32_t ret = -2;
    if (deviceId == 0) {
        if (nodeFlag == kNodeDoubleTapEnable ||
            nodeFlag == kNodeCoordinate ||
            nodeFlag == kNodeDoubleTapIndep) {
            ret = 1;
        }
    }
    ALOGI("isTouchNodeSupport(dId=%d, nodeFlag=%d) -> %d", deviceId, nodeFlag, ret);
    return ret;
}

static void* onCreateTouch(void* args) { return args; }
static void onDestroyTouch(void*) {}

static binder_status_t onTransactTouch(
        AIBinder*, transaction_code_t code, const AParcel* in, AParcel* out) {
    switch (code) {
        case kTxInitialize: {
            ALOGI("IOplusTouch::initialize() -> 0");
            writeOkStatus(out);
            AParcel_writeInt32(out, 0);
            return STATUS_OK;
        }
        case kTxIsTouchNodeSupport: {
            int32_t dId = 0, nodeFlag = 0;
            AParcel_readInt32(in, &dId);
            AParcel_readInt32(in, &nodeFlag);
            int32_t ret = handleIsNodeSupport(dId, nodeFlag);
            writeOkStatus(out);
            AParcel_writeInt32(out, ret);
            return STATUS_OK;
        }
        case kTxTouchReadNodeFile: {
            int32_t dId = 0, nodeFlag = 0;
            AParcel_readInt32(in, &dId);
            AParcel_readInt32(in, &nodeFlag);
            std::string val = handleReadNode(dId, nodeFlag);
            ALOGI("touchReadNodeFile(dId=%d, nodeFlag=%d) -> '%s'", dId, nodeFlag, val.c_str());
            writeOkStatus(out);
            AParcel_writeString(out, val.c_str(), static_cast<int32_t>(val.size()));
            return STATUS_OK;
        }
        case kTxTouchWriteNodeFile: {
            int32_t dId = 0, nodeFlag = 0;
            std::string info;
            AParcel_readInt32(in, &dId);
            AParcel_readInt32(in, &nodeFlag);
            readStringStd(in, info);
            int32_t ret = handleWriteNode(dId, nodeFlag, info);
            writeOkStatus(out);
            AParcel_writeInt32(out, ret);
            return STATUS_OK;
        }
        case kTxTouchWriteBtInfo: {
            int32_t dId = 0, nodeFlag = 0;
            std::string info;
            AParcel_readInt32(in, &dId);
            AParcel_readInt32(in, &nodeFlag);
            readStringStd(in, info);
            ALOGI("touchWriteBtInfo(dId=%d, nodeFlag=%d, info='%s') -> 0",
                  dId, nodeFlag, info.c_str());
            writeOkStatus(out);
            AParcel_writeInt32(out, 0);
            return STATUS_OK;
        }
        case kTxTouchWriteNodeFileOneWay: {
            int32_t dId = 0, nodeFlag = 0;
            std::string info;
            AParcel_readInt32(in, &dId);
            AParcel_readInt32(in, &nodeFlag);
            readStringStd(in, info);
            handleWriteNode(dId, nodeFlag, info);
            return STATUS_OK;  // oneway
        }
        case kTxTouchNotifyClient: {
            int32_t clientFlag = 0;
            AParcel_readInt32(in, &clientFlag);
            skipParcelable(in);
            ALOGI("touchNotifyClient(clientFlag=%d) -> 0", clientFlag);
            writeOkStatus(out);
            AParcel_writeInt32(out, 0);
            return STATUS_OK;
        }
        case kTxRegisterEventCallback: {
            AIBinder* cb = nullptr;
            AParcel_readStrongBinder(in, &cb);
            if (cb) AIBinder_decStrong(cb);
            ALOGI("registerEventCallback() -> 0");
            writeOkStatus(out);
            AParcel_writeInt32(out, 0);
            return STATUS_OK;
        }
        case kTxUnregisterEventCallback: {
            AIBinder* cb = nullptr;
            AParcel_readStrongBinder(in, &cb);
            if (cb) AIBinder_decStrong(cb);
            ALOGI("unregisterEventCallback() -> 0");
            writeOkStatus(out);
            AParcel_writeInt32(out, 0);
            return STATUS_OK;
        }
        case kTxGetInterfaceVersion: {
            writeOkStatus(out);
            AParcel_writeInt32(out, kAidlTouchVersion);
            return STATUS_OK;
        }
        case kTxGetInterfaceHash: {
            writeOkStatus(out);
            AParcel_writeString(out, kAidlTouchHash, strlen(kAidlTouchHash));
            return STATUS_OK;
        }
        default:
            ALOGW("IOplusTouch: unknown tx=%u", code);
            return STATUS_UNKNOWN_TRANSACTION;
    }
}

static bool testDozeDoubleTapDetector() {
    DozeDoubleTapDetector det;
    // Match exact live fts_ts raw axis scale (16x logical 1080x2400)
    det.setAbsRange(17280, 38400);

    auto emitTap = [&](int32_t lx, int32_t ly, int64_t downMs, int64_t upMs,
                       int32_t endLx = -1, int32_t endLy = -1) -> bool {
        if (endLx < 0) endLx = lx;
        if (endLy < 0) endLy = ly;
        bool fired = false;
        fired |= det.processEvent(EV_ABS, ABS_MT_SLOT, 0, downMs);
        fired |= det.processEvent(EV_ABS, ABS_MT_TRACKING_ID, 42, downMs);
        fired |= det.processEvent(EV_ABS, ABS_MT_POSITION_X, lx * 16, downMs);
        fired |= det.processEvent(EV_ABS, ABS_MT_POSITION_Y, ly * 16, downMs);
        fired |= det.processEvent(EV_SYN, SYN_REPORT, 0, downMs);
        if (endLx != lx || endLy != ly) {
            int64_t midMs = (downMs + upMs) / 2;
            fired |= det.processEvent(EV_ABS, ABS_MT_POSITION_X, endLx * 16, midMs);
            fired |= det.processEvent(EV_ABS, ABS_MT_POSITION_Y, endLy * 16, midMs);
            fired |= det.processEvent(EV_SYN, SYN_REPORT, 0, midMs);
        }
        fired |= det.processEvent(EV_ABS, ABS_MT_TRACKING_ID, -1, upMs);
        fired |= det.processEvent(EV_SYN, SYN_REPORT, 0, upMs);
        return fired;
    };

    // 1. Valid crisp double-tap (tap1 0..60ms, tap2 160..220ms at ~same spot) -> MUST wake
    bool t1a = emitTap(540, 1200, 1000, 1060);
    bool t1b = emitTap(560, 1220, 1160, 1220);

    // 2. Debounce check: immediate 3rd/4th tap within 500ms -> MUST NOT double-wake
    bool t2a = emitTap(540, 1200, 1300, 1340);
    bool t2b = emitTap(540, 1200, 1400, 1450);

    // 3. Single tap only -> MUST NOT wake
    det.resetAll();
    bool t3 = emitTap(540, 1200, 3000, 3070);

    // 4. Slow pair (gap > 380ms) -> MUST NOT wake
    det.resetAll();
    bool t4a = emitTap(540, 1200, 4000, 4060);
    bool t4b = emitTap(540, 1200, 4600, 4660);

    // 5. Distant pair (> 200 logical px apart) -> MUST NOT wake
    det.resetAll();
    bool t5a = emitTap(200, 500, 5000, 5060);
    bool t5b = emitTap(800, 1800, 5180, 5240);

    // 6. FOD press-and-hold (tap2 held 350ms > 260ms) -> MUST NOT wake
    det.resetAll();
    bool t6a = emitTap(540, 2140, 6000, 6080);
    bool t6b = emitTap(540, 2140, 6200, 6550);

    // 7. Swipe (drift 300px > 90px) -> MUST NOT wake
    det.resetAll();
    bool t7a = emitTap(540, 1200, 7000, 7080, 540, 1500);
    bool t7b = emitTap(540, 1500, 7180, 7240);

    // 8. Hardware KEY_POWER arrived before tap2 -> resets state, MUST NOT wake
    det.resetAll();
    bool t8a = emitTap(540, 1200, 8000, 8050);
    det.processEvent(EV_KEY, KEY_POWER, 1, 8100);
    bool t8b = emitTap(540, 1200, 8150, 8200);

    // 9. Tap 1 slow/single, then clean tap2 + tap3 double-tap -> tap3 MUST wake
    det.resetAll();
    bool t9a = emitTap(100, 100, 10000, 10050);
    bool t9b = emitTap(540, 1200, 11000, 11060);
    bool t9c = emitTap(550, 1210, 11180, 11240);

    printf("  dozeDt2w: valid=(%d,%d) debounce=(%d,%d) single=%d slow=(%d,%d) dist=(%d,%d) fodHold=(%d,%d) swipe=(%d,%d) hwPower=(%d,%d) chain=(%d,%d,%d)\n",
           t1a, t1b, t2a, t2b, t3, t4a, t4b, t5a, t5b, t6a, t6b, t7a, t7b, t8a, t8b, t9a, t9b, t9c);

    return (!t1a && t1b && !t2a && !t2b && !t3 &&
            !t4a && !t4b && !t5a && !t5b && !t6a && !t6b &&
            !t7a && !t7b && !t8a && !t8b && !t9a && !t9b && t9c);
}

static int runSelfTest() {
    printf("[OplusTouchAidlBridge v4] Running self-test...\n");
    gDryRun = true;

    AIBinder_Class* cls = AIBinder_Class_define(
            kIOplusTouchDescriptor, onCreateTouch, onDestroyTouch, onTransactTouch);
    AIBinder* b = AIBinder_new(cls, nullptr);
    AIBinder_markVintfStability(b);

    AParcel *in = nullptr, *out = nullptr;
    AStatus* st = nullptr;
    int32_t ver = -1, sup1 = 0, sup21 = 0, supOther = 0, wret = -1;
    std::string hash, readVal1, readVal2, readIndep1, readIndep2;

    auto writeNode = [&](int32_t flag, const char* s) -> int32_t {
        AParcel *wi = nullptr, *wo = nullptr;
        AStatus* wst = nullptr;
        int32_t r = -1;
        AIBinder_prepareTransaction(b, &wi);
        AParcel_writeInt32(wi, 0);
        AParcel_writeInt32(wi, flag);
        AParcel_writeString(wi, s, static_cast<int32_t>(strlen(s)));
        AIBinder_transact(b, kTxTouchWriteNodeFile, &wi, &wo, 0);
        AParcel_readStatusHeader(wo, &wst);
        AParcel_readInt32(wo, &r);
        AStatus_delete(wst); AParcel_delete(wo);
        return r;
    };
    auto readNode = [&](int32_t flag) -> std::string {
        AParcel *ri = nullptr, *ro = nullptr;
        AStatus* rst = nullptr;
        std::string v;
        AIBinder_prepareTransaction(b, &ri);
        AParcel_writeInt32(ri, 0);
        AParcel_writeInt32(ri, flag);
        AIBinder_transact(b, kTxTouchReadNodeFile, &ri, &ro, 0);
        AParcel_readStatusHeader(ro, &rst);
        readStringStd(ro, v);
        AStatus_delete(rst); AParcel_delete(ro);
        return v;
    };

    // 1. getInterfaceVersion
    AIBinder_prepareTransaction(b, &in);
    AIBinder_transact(b, kTxGetInterfaceVersion, &in, &out, 0);
    AParcel_readStatusHeader(out, &st);
    AParcel_readInt32(out, &ver);
    AStatus_delete(st); AParcel_delete(out);

    // 2. getInterfaceHash
    in = nullptr; out = nullptr; st = nullptr;
    AIBinder_prepareTransaction(b, &in);
    AIBinder_transact(b, kTxGetInterfaceHash, &in, &out, 0);
    AParcel_readStatusHeader(out, &st);
    readStringStd(out, hash);
    AStatus_delete(st); AParcel_delete(out);

    // 3. isTouchNodeSupport(0, 1) == 1, (0, 21) == 1, (1, 1) == -2
    in = nullptr; out = nullptr; st = nullptr;
    AIBinder_prepareTransaction(b, &in);
    AParcel_writeInt32(in, 0);
    AParcel_writeInt32(in, kNodeDoubleTapEnable);
    AIBinder_transact(b, kTxIsTouchNodeSupport, &in, &out, 0);
    AParcel_readStatusHeader(out, &st);
    AParcel_readInt32(out, &sup1);
    AStatus_delete(st); AParcel_delete(out);

    in = nullptr; out = nullptr; st = nullptr;
    AIBinder_prepareTransaction(b, &in);
    AParcel_writeInt32(in, 0);
    AParcel_writeInt32(in, kNodeDoubleTapIndep);
    AIBinder_transact(b, kTxIsTouchNodeSupport, &in, &out, 0);
    AParcel_readStatusHeader(out, &st);
    AParcel_readInt32(out, &sup21);
    AStatus_delete(st); AParcel_delete(out);

    in = nullptr; out = nullptr; st = nullptr;
    AIBinder_prepareTransaction(b, &in);
    AParcel_writeInt32(in, 1);
    AParcel_writeInt32(in, kNodeDoubleTapEnable);
    AIBinder_transact(b, kTxIsTouchNodeSupport, &in, &out, 0);
    AParcel_readStatusHeader(out, &st);
    AParcel_readInt32(out, &supOther);
    AStatus_delete(st); AParcel_delete(out);

    // 4. Exact boot11 + boot19 Doze/SOFOD regression test:
    //    ScreenOffGestureService writes node21="2" & node1="1" (awake -> doze=0, sw=5),
    //    OplusDozeService/OplusAod writes node21="65536" (entering Doze -> doze=1, dt=1),
    //    cancelled wake writes node194="0" (STILL in Doze -> doze MUST stay 1!),
    //    power-saving AOD fades out with node21="0" (dt=1, doze=1),
    //    real interactive wakeup writes node21="2" (exits Doze -> doze=0, dt=1).
    wret = writeNode(kNodeDoubleTapIndep, "2");
    writeNode(kNodeDoubleTapEnable, "1");
    int32_t dtModeAfterOn = gLastDoubleTapMode;
    int32_t swAfterOn = gLastGestureSwitchMode;
    bool dozeAfterAwakeOn = gInDoze;
    readVal1 = readNode(kNodeDoubleTapEnable);

    writeNode(kNodeDoubleTapIndep, "65536");
    int32_t dtAfterAodOnly = gLastDoubleTapMode;
    bool dozeAfter65536 = gInDoze;
    readIndep1 = readNode(kNodeDoubleTapIndep);

    writeNode(194, "0");
    bool dozeAfter194 = gInDoze;

    writeNode(kNodeDoubleTapIndep, "0");
    int32_t dtAfterAodOff = gLastDoubleTapMode;
    bool dozeAfterAodOff = gInDoze;

    writeNode(kNodeDoubleTapIndep, "2");
    bool dozeAfterWake2 = gInDoze;

    // 4c. nodeFlag=21 "65538" (0x10002 = DT2W+AOD) reads hex "10002", keeps DT2W ON, doze=1;
    //     explicitly disabling DT2W via node1="0" + node21="0" turns DT2W OFF, sw=4, doze=0.
    writeNode(kNodeDoubleTapIndep, "65538");
    readIndep2 = readNode(kNodeDoubleTapIndep);
    int32_t dtAfterIndepOn = gLastDoubleTapMode;
    bool dozeAfter65538 = gInDoze;
    writeNode(kNodeDoubleTapEnable, "0");
    writeNode(kNodeDoubleTapIndep, "0");
    int32_t dtAfterIndepZero = gLastDoubleTapMode;
    int32_t swAfterOff = gLastGestureSwitchMode;
    bool dozeAfterDisable = gInDoze;

    // 5. touchWriteNodeFileOneWay(0, 1, "0") & read back
    writeNode(kNodeDoubleTapEnable, "1");
    in = nullptr; out = nullptr;
    AIBinder_prepareTransaction(b, &in);
    AParcel_writeInt32(in, 0);
    AParcel_writeInt32(in, kNodeDoubleTapEnable);
    AParcel_writeString(in, "0", 1);
    AIBinder_transact(b, kTxTouchWriteNodeFileOneWay, &in, &out, FLAG_ONEWAY);
    if (out) AParcel_delete(out);
    int32_t dtModeAfterOff = gLastDoubleTapMode;
    readVal2 = readNode(kNodeDoubleTapEnable);

    AIBinder_decStrong(b);

    bool dozeDetOk = testDozeDoubleTapDetector();

    printf("  ver=%d hash=%s sup(0,1)=%d sup(0,21)=%d sup(1,1)=%d wret=%d dtOn=%d swOn=%d r1='%s'\n",
           ver, hash.c_str(), sup1, sup21, supOther, wret, dtModeAfterOn, swAfterOn, readVal1.c_str());
    printf("  indep: aodOnly(dt=%d r='%s' doze=%d) n194(doze=%d) aodOff(dt=%d doze=%d) wake2(doze=%d)\n",
           dtAfterAodOnly, readIndep1.c_str(), dozeAfter65536, dozeAfter194,
           dtAfterAodOff, dozeAfterAodOff, dozeAfterWake2);
    printf("  indep2: both(dt=%d r='%s' doze=%d) zero(dt=%d sw=%d doze=%d) onewayOff(dt=%d r='%s') awakeDoze=%d\n",
           dtAfterIndepOn, readIndep2.c_str(), dozeAfter65538, dtAfterIndepZero, swAfterOff,
           dozeAfterDisable, dtModeAfterOff, readVal2.c_str(), dozeAfterAwakeOn);

    return (ver == 2 && hash == kAidlTouchHash && sup1 == 1 && sup21 == 1 && supOther == -2 &&
            wret == 0 && dtModeAfterOn == 1 && swAfterOn == 5 && !dozeAfterAwakeOn &&
            readVal1 == "1" && dtAfterAodOnly == 1 && dozeAfter65536 && dozeAfter194 &&
            readIndep1 == "10000" && dtAfterAodOff == 1 && dozeAfterAodOff && !dozeAfterWake2 &&
            dtAfterIndepOn == 1 && readIndep2 == "10002" && dozeAfter65538 &&
            dtAfterIndepZero == 0 && swAfterOff == 4 && !dozeAfterDisable &&
            dtModeAfterOff == 0 && readVal2 == "0" && dozeDetOk)
                   ? 0
                   : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "--self-test") == 0) {
        return runSelfTest();
    }

    BRIDGE_LOG_BOOT("OplusTouchAidlBridge v4 starting (fts_gesture_switch + SOFOD/AOD Doze DT2W -> IOplusTouch V2, debug prop=%s)...",
                    cos_bridge_log::kDebugPropName);

    startDozeTapMonitorOnce();

    ABinderProcess_setThreadPoolMaxThreadCount(4);
    ABinderProcess_startThreadPool();

    AIBinder_Class* cls = AIBinder_Class_define(
            kIOplusTouchDescriptor, onCreateTouch, onDestroyTouch, onTransactTouch);
    AIBinder* binder = AIBinder_new(cls, nullptr);
    AIBinder_markVintfStability(binder);

    binder_status_t st = AServiceManager_addService(binder, kIOplusTouchInstance);
    if (st != STATUS_OK) {
        ALOGE("AServiceManager_addService(%s) failed: %d", kIOplusTouchInstance, st);
        return 1;
    }
    BRIDGE_LOG_BOOT("Registered %s successfully!", kIOplusTouchInstance);
    ABinderProcess_joinThreadPool();
    ALOGE("ABinderProcess_joinThreadPool() exited unexpectedly!");
    return 1;
}
