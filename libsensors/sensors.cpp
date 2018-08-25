/*
 * Copyright (C) 2016 The Android Open-Source Project
 * Copyright (C) 2016 Dániel Járai
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

#define LOG_TAG "Sensors"
#ifndef FUNC_LOG
//#define FUNC_LOG ALOGV("%s", __PRETTY_FUNCTION__)
#define FUNC_LOG
#endif

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>

#include <utils/Atomic.h>
#include <utils/Log.h>

#include "sensors.h"

#include "MPLSensor.h"
#include "LightSensor.h"

#ifdef ENABLE_DMP_SCREEN_AUTO_ROTATION
#define GLOBAL_SENSORS (MPLSensor::NumSensors + 1)
#else
#define GLOBAL_SENSORS MPLSensor::NumSensors
#endif

#define LOCAL_SENSORS (1)

static struct sensor_t sSensorList[LOCAL_SENSORS + GLOBAL_SENSORS] = {
    {
        .name       = "AL3010 Ambient Light",
        .vendor     = "Lite-On",
        .version    = 1,
        .handle     = SENSORS_LIGHT_HANDLE,
        .type       = SENSOR_TYPE_LIGHT,
        .maxRange   = 72945.0f,
        .resolution = 1.0f,
        .power      = 0.18f,
        .minDelay   = 0,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount  = 0,
        .stringType = SENSOR_STRING_TYPE_LIGHT,
        .requiredPermission = "",
        .maxDelay   = 0,
        .flags      = SENSOR_FLAG_ON_CHANGE_MODE,
        .reserved   = { },
    },
};
static int numSensors = (sizeof(sSensorList) / sizeof(sensor_t));

static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device);

static int sensors__get_sensors_list(struct sensors_module_t* module __unused,
                                     struct sensor_t const** list)
{
    *list = sSensorList;
    return numSensors;
}

static struct hw_module_methods_t sensors_module_methods = {
        .open = open_sensors,
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
        .common = {
                .tag = HARDWARE_MODULE_TAG,
                .module_api_version = SENSORS_MODULE_API_VERSION_0_1,
                .hal_api_version = HARDWARE_HAL_API_VERSION,
                .id = SENSORS_HARDWARE_MODULE_ID,
                .name = "Nexus 7 Sensor module",
                .author = "Daniel Jarai",
                .methods = &sensors_module_methods,
                .dso = 0,
                .reserved = { },
        },
        .get_sensors_list = sensors__get_sensors_list,
        .set_operation_mode = NULL,
};

struct sensors_poll_context_t {
    sensors_poll_device_1_t device; // must be first

    sensors_poll_context_t();
    ~sensors_poll_context_t();
    int activate(int handle, int enabled);
    int setDelay(int handle, int64_t ns);
    int pollEvents(sensors_event_t* data, int count);
    int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
    int flush(int handle);

private:
    enum {
        mpl = 0,
        compass,
        dmpOrient,
        dmpSign,
        light,
        numSensorDrivers,       // wake pipe goes here
        numFds,
    };

    struct pollfd mPollFds[numFds];
    SensorBase* mSensors[numSensorDrivers];
    CompassSensor *mCompassSensor;

    static const size_t wake = numSensorDrivers;
    static const char WAKE_MESSAGE = 'W';
    int mWritePipeFd;

    int handleToDriver(int handle) const {
        switch (handle) {
            case ID_GY:
            case ID_RG:
            case ID_A:
            case ID_M:
            case ID_RM:
            case ID_O:
            case ID_RV:
            case ID_GRV:
            case ID_LA:
            case ID_GR:
            case ID_SM:
            case ID_SO:
                return mpl;
            case ID_L:
                return light;
        }
        return -EINVAL;
    }
};

static int accelLoadCalib(long *accel_offset)
{
    FUNC_LOG;

    FILE *fp;
    int i, s[3], v[3];

    fp = fopen("/per/sensors/KXTF9_Calibration.ini", "r");
    if (fp) {
        fscanf(fp, "%d %d %d %d %d %d",
            &s[0], &v[0], &s[1], &v[1], &s[2], &v[2]);
        fclose(fp);
        for (i = 0; i < 3; ++i)
            accel_offset[i] = (v[i] + ((s[i] - v[i]) / 2)) << 20;
        if (s[2] > v[2])
            accel_offset[2] = -accel_offset[2];
    } else {
        ALOGE("Cannot load accelerometer calibration file!");
        return 1;
    }

    return 0;
}

sensors_poll_context_t::sensors_poll_context_t()
{
    FUNC_LOG;

    mCompassSensor = new CompassSensor();
    MPLSensor *mplSensor = new MPLSensor(mCompassSensor, accelLoadCalib);

    // must clean this up early or else the destructor will make a mess
    memset(mSensors, 0, sizeof(mSensors));

    // setup the callback object for handing mpl callbacks
    setCallbackObject(mplSensor);

    // populate the sensor list
    numSensors = LOCAL_SENSORS +
        mplSensor->populateSensorList(sSensorList + LOCAL_SENSORS,
                sizeof(sSensorList[0]) * (ARRAY_SIZE(sSensorList) - LOCAL_SENSORS));

    mSensors[mpl] = mplSensor;
    mPollFds[mpl].fd = mSensors[mpl]->getFd();
    mPollFds[mpl].events = POLLIN;
    mPollFds[mpl].revents = 0;

    mSensors[compass] = mplSensor;
    mPollFds[compass].fd =  mCompassSensor->getFd();
    mPollFds[compass].events = POLLIN;
    mPollFds[compass].revents = 0;

    mSensors[dmpOrient] = mplSensor;
    mPollFds[dmpOrient].fd = ((MPLSensor*) mSensors[mpl])->getDmpOrientFd();
    mPollFds[dmpOrient].events = POLLPRI;
    mPollFds[dmpOrient].revents = 0;

    mPollFds[dmpSign].fd = ((MPLSensor*) mSensors[mpl])->getDmpSignificantMotionFd();
    mPollFds[dmpSign].events = POLLPRI;
    mPollFds[dmpSign].revents = 0;

    mSensors[light] = new LightSensor();
    mPollFds[light].fd = mSensors[light]->getFd();
    mPollFds[light].events = POLLIN;
    mPollFds[light].revents = 0;

    /* Timer based sensor initialization */
    int wakeFds[2];
    int result = pipe(wakeFds);
    ALOGE_IF(result < 0, "error creating wake pipe (%s)", strerror(errno));
    fcntl(wakeFds[0], F_SETFL, O_NONBLOCK);
    fcntl(wakeFds[1], F_SETFL, O_NONBLOCK);
    mWritePipeFd = wakeFds[1];

    mPollFds[wake].fd = wakeFds[0];
    mPollFds[wake].events = POLLIN;
    mPollFds[wake].revents = 0;
}

sensors_poll_context_t::~sensors_poll_context_t()
{
    FUNC_LOG;

    delete mCompassSensor;
    for (int i=0 ; i<numSensorDrivers ; i++) {
        delete mSensors[i];
        close(mPollFds[i].fd);
    }
    close(mWritePipeFd);
}

int sensors_poll_context_t::activate(int handle, int enabled)
{
    FUNC_LOG;

    int index = handleToDriver(handle);
    if (index < 0)
        return index;

    int err =  mSensors[index]->enable(handle, enabled);
    if (!err) {
        const char wakeMessage(WAKE_MESSAGE);
        int result = write(mWritePipeFd, &wakeMessage, 1);
        ALOGE_IF(result < 0,
                "error sending wake message (%s)", strerror(errno));
    }
    return err;
}

int sensors_poll_context_t::setDelay(int handle, int64_t ns)
{
    FUNC_LOG;

    int index = handleToDriver(handle);
    if (index < 0)
        return index;

    return mSensors[index]->setDelay(handle, ns);
}

int sensors_poll_context_t::pollEvents(sensors_event_t *data, int count)
{
    FUNC_LOG;

    int nbEvents = 0;
    int n = 0;
    int nb, polltime = -1;

    do {
        for (int i = 0; count && i < numSensorDrivers; i++) {
            SensorBase* const sensor(mSensors[i]);
            if (mPollFds[i].revents & (POLLIN | POLLPRI)) {
                nb = 0;
                if (i == mpl) {
                    ((MPLSensor*) sensor)->buildMpuEvent();
                    mPollFds[i].revents = 0;
                    nb = ((MPLSensor*) sensor)->readEvents(data, count);
                    if (nb > 0) {
                        count -= nb;
                        nbEvents += nb;
                        data += nb;
                    }
                } else if (i == compass) {
                    ((MPLSensor*) sensor)->buildCompassEvent();
                    mPollFds[i].revents = 0;
                    nb = ((MPLSensor*) sensor)->readEvents(data, count);
                    if (nb > 0) {
                        count -= nb;
                        nbEvents += nb;
                        data += nb;
                    }
                } else if (i == dmpOrient) {
                    nb = ((MPLSensor*) mSensors[mpl])->readDmpOrientEvents(data, count);
                    mPollFds[dmpOrient].revents = 0;
                    if (isDmpScreenAutoRotationEnabled() && nb > 0) {
                        count -= nb;
                        nbEvents += nb;
                        data += nb;
                    }
                } else if (i == dmpSign) {
                    ALOGI("HAL: dmpSign interrupt");
                    nb = ((MPLSensor*) mSensors[mpl])->readDmpSignificantMotionEvents(data, count);
                    mPollFds[i].revents = 0;
                    count -= nb;
                    nbEvents += nb;
                    data += nb;
                } else {
                    // LightSensor
                    nb = sensor->readEvents(data, count);
                    if (nb < count) {
                        // no more data for this sensor
                        mPollFds[i].revents = 0;
                    }
                    count -= nb;
                    nbEvents += nb;
                    data += nb;
                }
            }
        }
        if (count) {
            do {
                n = poll(mPollFds, numFds, nbEvents ? 0 : polltime);
            } while (n < 0 && errno == EINTR);
            if (n < 0) {
                ALOGE("poll() failed (%s)", strerror(errno));
                return -errno;
            }
            if (mPollFds[wake].revents & (POLLIN | POLLPRI)) {
                char msg;
                int result = read(mPollFds[wake].fd, &msg, 1);
                ALOGE_IF(result < 0, "error reading from wake pipe (%s)", strerror(errno));
                ALOGE_IF(msg != WAKE_MESSAGE, "unknown message on wake queue (0x%02x)", int(msg));
                mPollFds[wake].revents = 0;
            }
        }
    } while (n && count);

    return nbEvents;
}

int sensors_poll_context_t::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
    int index = handleToDriver(handle);
    if (index < 0) return index;
    return mSensors[index]->batch(handle, flags, period_ns, timeout);
}

int sensors_poll_context_t::flush(int handle)
{
    int index = handleToDriver(handle);
    if (index < 0) return index;
    return mSensors[index]->flush(handle);
}

static int device__close(struct hw_device_t *dev)
{
    FUNC_LOG;

    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    if (ctx)
        delete ctx;

    return 0;
}

static int device__activate(struct sensors_poll_device_t *dev,
                          int handle, int enabled)
{
    FUNC_LOG;

    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->activate(handle, enabled);
}

static int device__setDelay(struct sensors_poll_device_t *dev,
                          int handle, int64_t ns)
{
    FUNC_LOG;

    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->setDelay(handle, ns);
}

static int device__poll(struct sensors_poll_device_t *dev,
                      sensors_event_t* data, int count)
{
    FUNC_LOG;

    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->pollEvents(data, count);
}

static int device__batch(struct sensors_poll_device_1 *dev, int handle,
                         int flags, int64_t period_ns, int64_t timeout)
{
    FUNC_LOG;
    sensors_poll_context_t* ctx = (sensors_poll_context_t*) dev;
    return ctx->batch(handle, flags, period_ns, timeout);
}

static int device__flush(struct sensors_poll_device_1 *dev, int handle)
{
    FUNC_LOG;
    sensors_poll_context_t* ctx = (sensors_poll_context_t*) dev;
    return ctx->flush(handle);
}

static int open_sensors(const struct hw_module_t* module,
                        const char* id __unused,
                        struct hw_device_t** device)
{
    FUNC_LOG;

    int status = -EINVAL;
    sensors_poll_context_t *dev = new sensors_poll_context_t();

    memset(&dev->device, 0, sizeof(sensors_poll_device_1));

    dev->device.common.tag      = HARDWARE_DEVICE_TAG;
    dev->device.common.version  = SENSORS_DEVICE_API_VERSION_1_3;
    dev->device.common.module   = const_cast<hw_module_t*>(module);
    dev->device.common.close    = device__close;
    dev->device.activate        = device__activate;
    dev->device.setDelay        = device__setDelay;
    dev->device.poll            = device__poll;

    /* Batch processing */
    dev->device.batch           = device__batch;
    dev->device.flush           = device__flush;

    *device = &dev->device.common;
    status = 0;

    return status;
}
