#include <iostream>

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "firmware_i2c.h"


/**
 * @brief Firmware_I2C::Firmware_I2C
 *
 * ctor
 *
 * @param device                    which i2c device (eg. /dev/i2c-x) to use
 * @param address                   address on i2c buss above
 * @param debug                     turn debug on or off
 */
Firmware_I2C::Firmware_I2C(char *device, unsigned char address, bool debug) : mFd(-1), mDevice(device), mAddress(address), mDebug(debug) { }


/**
 * @brief Firmware_I2C::~Firmware_I2C
 *
 * dtor - clone device - that is close descriptor to i2c driver
 *
 */
Firmware_I2C::~Firmware_I2C()
{
    if (mDebug)
        std::cout << "Firmware_I2C::" << __func__ << ":" << __LINE__ << " closing device" << std::endl;

    closeDevice();
}


/**
 * @brief Firmware_I2C::openDevice
 *
 * open descriptor to i2c driver and set i2c device address
 *
 * @return                          zero on success, negative error value on failure
 */
int Firmware_I2C::openDevice()
{
    /// close if open
    if (mFd > 0)
        closeDevice();

    if (mDebug)
        std::cout << "Firmware_I2C::" << __func__ << ":" << __LINE__ << " opening device " << mDevice << std::endl;

    /// Open I2C device
    mFd = open(mDevice, O_RDWR);
    if (mFd < 0) {
        std::cerr << __func__ << ":" << __LINE__ << " openI2C failed with error " << mFd << std::endl;
        return -1;
    }

    if (mDebug) {
        printf("Address 0x%02x\n", mAddress);
        std::cerr << std::endl;
    }

    /// Set i2c device address in i2c driver for this session
    int status = ioctl(mFd, I2C_SLAVE, mAddress);
    if (status < 0) {
        std::cerr << __func__ << ":" << __LINE__ << " ioctl failed with error " << status << std::endl;
        return -2;
    }

    return 0;
}


/**
 * @brief Firmware_I2C::writeData
 *
 * write data to device
 *
 * @param data                      data to write
 * @param size                      bytes to be written
 *
 * @return                          zero on success, negative error value on failure
 */
int Firmware_I2C::writeData(unsigned char *data, int size)
{
    /// write data to device
    int status = write(mFd, data, size);
    if (status < 0) {
        std::cerr << "Firmware_I2C::" << __func__ << ":" << __LINE__ << " write failed " << status << std::endl;
        return -1;
    }

    return 0;
}


/**
 * @brief Firmware_I2C::readData
 *
 * read data from device
 *
 * @param data                      buffer where data from device will be written
 * @param size                      number of bytes to read
 *
 * @return                          zero on success, negative error value on failure
 */
int Firmware_I2C::readData(unsigned char *data, int size)
{
    /// read data from device
    int status = read(mFd, data, size);
    if (status != size) {
        std::cerr << "Firmware_I2C::" << __func__ << ":" << __LINE__ << " read failed " << status << std::endl;
        return -2;
    }

    return 0;
}


/**
 * @brief Firmware_I2C::closeDevice
 *
 * close descriptor to i2c driver
 *
 * @return                          zero on success, negative error value on failure
 */
int Firmware_I2C::closeDevice()
{
    if (mFd != -1) {
        int status = close(mFd);
        mFd = -1;
        return status;
    }

    return 0;
}


/**
 * @brief Firmware_I2C::setDevice
 *
 * copy device to internal buffer (eg. /dev/i2c-x)
 *
 * @param device                    device address to set
 *
 * @return                          zero on success, negative error value on failure
 */
int Firmware_I2C::setDevice(char *device)
{
    /// check that device is valid
    if (strcmp(device, FirmwareI2CDeviceses::i2c_0)     != 0 && strcmp(device, FirmwareI2CDeviceses::i2c_1) != 0 &&
            strcmp(device, FirmwareI2CDeviceses::i2c_2) != 0 && strcmp(device, FirmwareI2CDeviceses::i2c_3) != 0 &&
            strcmp(device, FirmwareI2CDeviceses::i2c_4) != 0 && strcmp(device, FirmwareI2CDeviceses::i2c_5) != 0 &&
            strcmp(device, FirmwareI2CDeviceses::i2c_6) != 0 && strcmp(device, FirmwareI2CDeviceses::i2c_7) != 0) {
        std::cerr << "Firmware_I2C::" << __func__ << ":" << __LINE__ << " device " << device << " not a valid device" << std::endl;
        return -1;
    }

    /// set device
    mDevice = device;

    return 0;
}
