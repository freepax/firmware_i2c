#ifndef FIRMWARE_I2C_H
#define FIRMWARE_I2C_H

#include <linux/i2c-dev.h>


namespace FirmwareI2CDeviceses {
static const char *i2c_0 = "/dev/i2c-0";
static const char *i2c_1 = "/dev/i2c-1";
static const char *i2c_2 = "/dev/i2c-2";
static const char *i2c_3 = "/dev/i2c-3";
static const char *i2c_4 = "/dev/i2c-4";
static const char *i2c_5 = "/dev/i2c-5";
static const char *i2c_6 = "/dev/i2c-6";
static const char *i2c_7 = "/dev/i2c-7";
}

static const int BufferSize = 2048;

class Firmware_I2C
{
public:
    /// ctor
    Firmware_I2C(char *device = (char*)FirmwareI2CDeviceses::i2c_0, unsigned char address = 0b00000000, bool debug = false);

    /// dtor
    ~Firmware_I2C();

    int openDevice();
    int closeDevice();

    int writeData(unsigned char *data, int size);
    int readData(unsigned char *data, int size);

    int setDevice(char *device);
    char *device(void) { return mDevice; }

    unsigned char address() { return mAddress; }
    const int fd() const { return mFd; }

    void setDebug(bool debug) { mDebug = debug; }
    const bool debug(void) const { return mDebug; }


protected:
    /// the i2c device to use (eg. "/dev/i2c-0")
    char *mDevice;

    /// file descriptor (opening mDevice)
    int mFd;

    /// i2c device address
    unsigned char mAddress;

    /// buffer for data traffic
    unsigned char mBuffer[BufferSize];

    /// debugging
    bool mDebug;
};

#endif /// FIRMWARE_I2C_H

