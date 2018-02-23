// I2Cdev library collection - MPU9250 I2C device class
// Based on InvenSense MPU-9250 register map document rev. 2.0, 5/19/2011
// (RM-MPU-6000A-00)
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "mpu9250/mpu9250.hpp"
#include "mpu9250/scale_conversions.hpp"

namespace addr = mpu9250::regs::addr;

namespace mpu9250
{

mpu9250::mpu9250()
    : m_params(), m_mag_sensitivity_adj{1.f, 1.f, 1.f}, m_mag_bias{0, 0, 0},
      m_mag_scale{1.f, 1.f, 1.f}
{
}

mpu9250::mpu9250(const parameters& params)
    : m_params(params), m_mag_sensitivity_adj{1.f, 1.f, 1.f}, m_mag_bias{0, 0, 0},
      m_mag_scale{1.f, 1.f, 1.f}
{
}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void mpu9250::initialize()
{
    // Wake up device
    I2Cdev::writeByte(m_params.dev_addr, addr::PWR_MGMT_1,
                      0x00); // Clear sleep mode bit (6), enable all sensors
    delay(100);

    // get stable time source
    I2Cdev::writeByte(m_params.dev_addr, addr::PWR_MGMT_1,
                      0x01); // Auto select clock source to
                             // be PLL gyroscope reference
                             // if ready else
    delay(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update
    // rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or
    // 1 kHz
    I2Cdev::writeByte(m_params.dev_addr, addr::CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    I2Cdev::writeByte(
            m_params.dev_addr, addr::SMPLRT_DIV,
            0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
                   // determined inset in CONFIG above

    setFullScaleGyroRange(m_params.gscale);
    setFullScaleAccelRange(m_params.ascale);

    I2Cdev::writeByte(m_params.dev_addr, addr::INT_PIN_CFG,
                      0x02); // set i2c bypass enable pin to true to access magnetometer
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see addr::PWR_MGMT_1
 * @see regs::PWR1_CLKSEL_BIT
 * @see regs::PWR1_CLKSEL_LENGTH
 */
void mpu9250::setClockSource(uint8_t source)
{
    I2Cdev::writeBits(m_params.dev_addr, addr::PWR_MGMT_1, regs::PWR1_CLKSEL_BIT,
                      regs::PWR1_CLKSEL_LENGTH, source);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool mpu9250::testConnection() { return getDeviceID() == 0x71; }
// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see regs::GYRO_FS_250
 * @see addr::GYRO_CONFIG
 * @see regs::GCONFIG_FS_SEL_BIT
 * @see regs::GCONFIG_FS_SEL_LENGTH
 */
uint8_t mpu9250::getFullScaleGyroRange()
{
    I2Cdev::readBits(m_params.dev_addr, addr::GYRO_CONFIG, regs::GCONFIG_FS_SEL_BIT,
                     regs::GCONFIG_FS_SEL_LENGTH, buffer);
    return buffer[0];
}
/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see regs::GYRO_FS_250
 * @see addr::GYRO_CONFIG
 * @see regs::GCONFIG_FS_SEL_BIT
 * @see regs::GCONFIG_FS_SEL_LENGTH
 */
void mpu9250::setFullScaleGyroRange(const gyro_scale& range)
{
    I2Cdev::writeBits(m_params.dev_addr, addr::GYRO_CONFIG, regs::GCONFIG_FS_SEL_BIT,
                      regs::GCONFIG_FS_SEL_LENGTH, (uint8_t)range);
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see regs::ACCEL_FS_2
 * @see addr::ACCEL_CONFIG
 * @see regs::ACONFIG_AFS_SEL_BIT
 * @see regs::ACONFIG_AFS_SEL_LENGTH
 */
uint8_t mpu9250::getFullScaleAccelRange()
{
    I2Cdev::readBits(m_params.dev_addr, addr::ACCEL_CONFIG, regs::ACONFIG_AFS_SEL_BIT,
                     regs::ACONFIG_AFS_SEL_LENGTH, buffer);
    return buffer[0];
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void mpu9250::setFullScaleAccelRange(const accel_scale& range)
{
    I2Cdev::writeBits(m_params.dev_addr, addr::ACCEL_CONFIG, regs::ACONFIG_AFS_SEL_BIT,
                      regs::ACONFIG_AFS_SEL_LENGTH, (uint8_t)range);
}

/** Get gyroscope output rate divider.
 * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
 * Motion detection, and Free Fall detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-9250 Product Specification document.
 *
 * @return Current sample rate
 * @see addr::SMPLRT_DIV
 */
uint8_t mpu9250::getRate()
{
    I2Cdev::readByte(m_params.dev_addr, addr::SMPLRT_DIV, buffer);
    return buffer[0];
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see addr::SMPLRT_DIV
 */
void mpu9250::setRate(uint8_t rate)
{
    I2Cdev::writeByte(m_params.dev_addr, addr::SMPLRT_DIV, rate);
}

// ACCEL_*OUT_* registers

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @param mx 16-bit signed integer container for magnetometer X-axis value
 * @param my 16-bit signed integer container for magnetometer Y-axis value
 * @param mz 16-bit signed integer container for magnetometer Z-axis value
 * @see getMotion6()
 * @see getAcceleration()
 * @see getRotation()
 * @see addr::ACCEL_XOUT_H
 */
void mpu9250::getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy,
                         int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz)
{
    // get accel and gyro
    getMotion6(ax, ay, az, gx, gy, gz);

    // read mag
    //I2Cdev::writeByte(m_params.dev_addr, addr::INT_PIN_CFG,
    //                  0x02); // set i2c bypass enable pin to true to access magnetometer
    int16_t mag[3];
    if (readMagData(mag))
    {
        *mx = mag[0];
        *my = mag[1];
        *mz = mag[2];
    }
}
/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see addr::ACCEL_XOUT_H
 */
void mpu9250::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy,
                         int16_t* gz)
{
    I2Cdev::readBytes(m_params.dev_addr, addr::ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}
/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see addr::GYRO_XOUT_H
 */
void mpu9250::getAcceleration(int16_t* x, int16_t* y, int16_t* z)
{
    I2Cdev::readBytes(m_params.dev_addr, addr::ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see addr::ACCEL_XOUT_H
 */
int16_t mpu9250::getAccelerationX()
{
    I2Cdev::readBytes(m_params.dev_addr, addr::ACCEL_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see addr::ACCEL_YOUT_H
 */
int16_t mpu9250::getAccelerationY()
{
    I2Cdev::readBytes(m_params.dev_addr, addr::ACCEL_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see addr::ACCEL_ZOUT_H
 */
int16_t mpu9250::getAccelerationZ()
{
    I2Cdev::readBytes(m_params.dev_addr, addr::ACCEL_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// TEMP_OUT_* registers

/** Get current internal temperature.
 * @return Temperature reading in 16-bit 2's complement format
 * @see addr::TEMP_OUT_H
 */
int16_t mpu9250::getTemperature()
{
    I2Cdev::readBytes(m_params.dev_addr, addr::TEMP_OUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

void mpu9250::getRotation(int16_t* x, int16_t* y, int16_t* z)
{
    I2Cdev::readBytes(m_params.dev_addr, addr::GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

int16_t mpu9250::getRotationX()
{
    I2Cdev::readBytes(m_params.dev_addr, addr::GYRO_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t mpu9250::getRotationY()
{
    I2Cdev::readBytes(m_params.dev_addr, addr::GYRO_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t mpu9250::getRotationZ()
{
    I2Cdev::readBytes(m_params.dev_addr, addr::GYRO_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

uint8_t mpu9250::getDeviceID()
{
    I2Cdev::readBits(m_params.dev_addr, addr::WHO_AM_I, regs::WHO_AM_I_BIT,
                     regs::WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}

void mpu9250::setDeviceID(uint8_t id)
{
    I2Cdev::writeBits(m_params.dev_addr, addr::WHO_AM_I, regs::WHO_AM_I_BIT,
                      regs::WHO_AM_I_LENGTH, id);
}

bool mpu9250::getDMPEnabled()
{
    I2Cdev::readBit(m_params.dev_addr, addr::USER_CTRL, regs::USERCTRL_DMP_EN_BIT,
                    buffer);
    return buffer[0];
}
void mpu9250::setDMPEnabled(bool enabled)
{
    I2Cdev::writeBit(m_params.dev_addr, addr::USER_CTRL, regs::USERCTRL_DMP_EN_BIT,
                     enabled);
}
void mpu9250::resetDMP()
{
    I2Cdev::writeBit(m_params.dev_addr, addr::USER_CTRL, regs::USERCTRL_DMP_RESET_BIT,
                     true);
}

uint8_t mpu9250::getDMPConfig1()
{
    I2Cdev::readByte(m_params.dev_addr, addr::DMP_CFG_1, buffer);
    return buffer[0];
}
void mpu9250::setDMPConfig1(uint8_t config)
{
    I2Cdev::writeByte(m_params.dev_addr, addr::DMP_CFG_1, config);
}

uint8_t mpu9250::getDMPConfig2()
{
    I2Cdev::readByte(m_params.dev_addr, addr::DMP_CFG_2, buffer);
    return buffer[0];
}

void mpu9250::setDMPConfig2(uint8_t config)
{
    I2Cdev::writeByte(m_params.dev_addr, addr::DMP_CFG_2, config);
}

bool mpu9250::getSleepEnabled()
{
    I2Cdev::readBit(m_params.dev_addr, addr::PWR_MGMT_1, regs::PWR1_SLEEP_BIT, buffer);
    return buffer[0];
}

void mpu9250::setSleepEnabled(bool enabled)
{
    I2Cdev::writeBit(m_params.dev_addr, addr::PWR_MGMT_1, regs::PWR1_SLEEP_BIT, enabled);
}

void mpu9250::calibrateAccelGyro(float* gyroBias, float* accelBias)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0};
    int32_t accel_bias[3] = {0, 0, 0};

    // reset device
    I2Cdev::writeByte(m_params.dev_addr, addr::PWR_MGMT_1,
                      0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if
    // ready
    // else use the internal oscillator, bits 2:0 = 001
    I2Cdev::writeByte(m_params.dev_addr, addr::PWR_MGMT_1, 0x01);
    I2Cdev::writeByte(m_params.dev_addr, addr::PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    I2Cdev::writeByte(m_params.dev_addr, addr::INT_ENABLE,
                      0x00);                                   // Disable all interrupts
    I2Cdev::writeByte(m_params.dev_addr, addr::FIFO_EN, 0x00); // Disable FIFO
    I2Cdev::writeByte(m_params.dev_addr, addr::PWR_MGMT_1,
                      0x00); // Turn on internal clock source
    I2Cdev::writeByte(m_params.dev_addr, addr::I2C_MST_CTRL, 0x00); // Disable I2C master
    I2Cdev::writeByte(m_params.dev_addr, addr::USER_CTRL,
                      0x00); // Disable FIFO and I2C master modes
    I2Cdev::writeByte(m_params.dev_addr, addr::USER_CTRL, 0x0C); // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    I2Cdev::writeByte(m_params.dev_addr, addr::CONFIG,
                      0x01); // Set low-pass filter to 188 Hz
    I2Cdev::writeByte(m_params.dev_addr, addr::SMPLRT_DIV,
                      0x00); // Set sample rate to 1 kHz
    I2Cdev::writeByte(
            m_params.dev_addr, addr::GYRO_CONFIG,
            0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    I2Cdev::writeByte(m_params.dev_addr, addr::ACCEL_CONFIG,
                      0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    I2Cdev::writeByte(m_params.dev_addr, addr::USER_CTRL, 0x40); // Enable FIFO
    I2Cdev::writeByte(m_params.dev_addr, addr::FIFO_EN,
                      0x78); // Enable gyro and accelerometer
    // sensors for FIFO  (max size 512
    // bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    I2Cdev::writeByte(m_params.dev_addr, addr::FIFO_EN,
                      0x00); // Disable gyro and accelerometer sensors for FIFO
    I2Cdev::readBytes(m_params.dev_addr, addr::FIFO_COUNTH, 2,
                      &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count /
                   12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        I2Cdev::readBytes(m_params.dev_addr, addr::FIFO_R_W, 12,
                          &data[0]); // read data for averaging
        accel_temp[0] =
                (int16_t)(((int16_t)data[0] << 8) |
                          data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases
                                                 // to get accumulated signed 32-bit
                                                 // biases
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }

    accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t)accelsensitivity;
    } // Remove gravity from the z-axis accelerometer bias calculation
    else
    {
        accel_bias[2] += (int32_t)accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are
    // reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s
                                               // to conform to expected bias input
                                               // format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on
                                               // calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    I2Cdev::writeByte(m_params.dev_addr, addr::XG_OFFS_USRH, data[0]);
    I2Cdev::writeByte(m_params.dev_addr, addr::XG_OFFS_USRL, data[1]);
    I2Cdev::writeByte(m_params.dev_addr, addr::YG_OFFS_USRH, data[2]);
    I2Cdev::writeByte(m_params.dev_addr, addr::YG_OFFS_USRL, data[3]);
    I2Cdev::writeByte(m_params.dev_addr, addr::ZG_OFFS_USRH, data[4]);
    I2Cdev::writeByte(m_params.dev_addr, addr::ZG_OFFS_USRL, data[5]);

    gyroBias[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
    gyroBias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    gyroBias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

    accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
    accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
    accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

void mpu9250::initMagnetometer()
{
    // Power down
    I2Cdev::writeByte(addr::mag::ADDRESS, addr::mag::CNTL, (uint8_t)mag_mode::POWER_DOWN);
    delay(50);

    // Enter Fuse ROM access mode
    I2Cdev::writeByte(addr::mag::ADDRESS, addr::mag::CNTL,
                      (uint8_t)mag_mode::FUSE_ROM_ACCESS);
    delay(50);

    // See register map for MPU9250 (doc. # RM-MPU-9250A-00) for the formula
    auto adjust = [](uint8_t raw) { return (float)(raw - 128) / 256. + 1.f; };

    I2Cdev::writeByte(m_params.dev_addr, addr::INT_PIN_CFG,
                      0x02); // set i2c bypass enable pin to true to access magnetometer
    uint8_t raw_data[3];
    I2Cdev::readBytes(addr::mag::ADDRESS, addr::mag::ASAX, 3, raw_data);
    m_mag_sensitivity_adj[0] = adjust(raw_data[0]);
    m_mag_sensitivity_adj[1] = adjust(raw_data[1]);
    m_mag_sensitivity_adj[2] = adjust(raw_data[2]);

    // Power down magnetometer
    I2Cdev::writeByte(addr::mag::ADDRESS, addr::mag::CNTL, (uint8_t)mag_mode::POWER_DOWN);
    delay(50);

    // Set the requested resolution and Mode
    I2Cdev::writeByte(addr::mag::ADDRESS, addr::mag::CNTL,
                      (uint8_t)m_params.mscale << 4 | (uint8_t)m_params.mmode);
    delay(50);
}

bool mpu9250::readMagData(int16_t* destination)
{
    uint8_t rawData[7];
    bool newMagData = (I2Cdev::readByte(addr::mag::ADDRESS, addr::mag::ST1) & 0x01);
    if (newMagData)
    {
        // Read the six raw data and ST2 registers sequentially into data
        // array
        I2Cdev::readBytes(addr::mag::ADDRESS, addr::mag::XOUT_L, 7, &rawData[0]);
        uint8_t c = rawData[6]; // End data read by reading ST2 register

        // Check if magnetic sensor overflow set, if not then report data
        if (!(c & 0x08))
        {
            destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
            destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
            destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
        }
    }
}

void mpu9250::calibrateMag()
{
    mag_mode mode = m_params.mmode;

    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767},
            mag_temp[3] = {0, 0, 0};

    Serial.println("# Mag Calibration: Wave device in a figure eight until done!");
    delay(4000);

    // shoot for ~fifteen seconds of mag data
    if (mode == mag_mode::CONT_MEASUREMENT_1)
    {
        sample_count = 256; // at 8 Hz ODR, new mag data is available every 125 ms
    }
    else if (mode == mag_mode::CONT_MEASUREMENT_2)
    {
        sample_count = 3000; // at 100 Hz ODR, new mag data is available every 10 ms
    }
    else
    {
        Serial.println("# Mag calibration failed: only CONT_MEASUREMENT_1/2 supported.");
        return;
    }

    for (ii = 0; ii < sample_count; ii++)
    {
        readMagData(mag_temp); // Read the mag data
        for (int jj = 0; jj < 3; jj++)
        {
            if (mag_temp[jj] > mag_max[jj])
                mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj])
                mag_min[jj] = mag_temp[jj];
        }
        if (mode == mag_mode::CONT_MEASUREMENT_1)
        {
            delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
        }
        else
        {
            delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
        }
    }

    Serial.println("mag x min/max:");
    Serial.println(mag_min[0]);
    Serial.println(mag_max[0]);
    Serial.println("mag y min/max:");
    Serial.println(mag_min[1]);
    Serial.println(mag_max[1]);
    Serial.println("mag z min/max:");
    Serial.println(mag_min[2]);
    Serial.println(mag_max[2]);

    // Get hard iron correction
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

    // save mag biases in G for main program
    float mag_resolution = ::mpu9250::get_mag_resolution(m_params.mscale);
    m_mag_bias[0] = (float)mag_bias[0] * mag_resolution * m_mag_sensitivity_adj[0];
    m_mag_bias[1] = (float)mag_bias[1] * mag_resolution * m_mag_sensitivity_adj[1];
    m_mag_bias[2] = (float)mag_bias[2] * mag_resolution * m_mag_sensitivity_adj[2];

    Serial.println("mag bias x:");
    Serial.println(m_mag_bias[0]);
    Serial.println("mag bias y:");
    Serial.println(m_mag_bias[1]);
    Serial.println("mag bias z:");
    Serial.println(m_mag_bias[2]);

    // Get soft iron correction estimate
    mag_scale[0] = (mag_max[0] - mag_min[0]) /
                   2; // get average x axis max chord length in counts
    mag_scale[1] = (mag_max[1] - mag_min[1]) /
                   2; // get average y axis max chord length in counts
    mag_scale[2] = (mag_max[2] - mag_min[2]) /
                   2; // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    m_mag_scale[0] = avg_rad / ((float)mag_scale[0]);
    m_mag_scale[1] = avg_rad / ((float)mag_scale[1]);
    m_mag_scale[2] = avg_rad / ((float)mag_scale[2]);

    Serial.println("mag x scale:");
    Serial.println(m_mag_scale[0]);
    Serial.println("mag y scale:");
    Serial.println(m_mag_scale[1]);
    Serial.println("mag z scale:");
    Serial.println(m_mag_scale[2]);

    Serial.println("# Mag Calibration done!");
}

} // namespace mpu9250
