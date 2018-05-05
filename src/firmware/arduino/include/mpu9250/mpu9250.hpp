// I2Cdev library collection - MPU9250 I2C device class
// Based on InvenSense MPU-9250 register map document rev. 2.0, 5/19/2011
// (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
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

The code in this file was modified by Lukas Solanka.

*/

#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stdint.h>

#include "I2Cdev/I2Cdev.h"

#include "registers.hpp"
#include "types.hpp"

namespace mpu9250
{
class mpu9250
{
  public:
    struct parameters
    {
        uint8_t dev_addr;
        accel_scale ascale;
        gyro_scale gscale;
        mag_scale mscale;
        mag_mode mmode;

        parameters()
            : dev_addr(regs::addr::DEFAULT_ADDRESS),
              ascale(accel_scale::AFS_2G),
              gscale(gyro_scale::GFS_250DPS),
              mscale(mag_scale::MFS_16BITS),
              mmode(mag_mode::CONT_MEASUREMENT_2)
        {
        }

        parameters(uint8_t address)
        {
            (*this) = get_default();
            dev_addr = address;
        }

        static parameters get_default() { return parameters{}; }
    };

    /** Default constructor, uses default I2C address, 2g accelerometer, 250dps gyro
     * resolution.
     * @see MPU9250_DEFAULT_ADDRESS
     */
    mpu9250();

    mpu9250(const parameters& params);

    void initialize();

    void initMagnetometer();

    bool testConnection();
    void setClockSource(uint8_t source);

    const parameters& getParameters() const { return m_params; }

    // SMPLRT_DIV register
    uint8_t getRate();
    void setRate(uint8_t rate);

    // GYRO_CONFIG register
    uint8_t getFullScaleGyroRange();
    void setFullScaleGyroRange(const gyro_scale& range);

    uint8_t getFullScaleAccelRange();
    void setFullScaleAccelRange(const accel_scale& scale);

    // ACCEL_*OUT_* registers
    void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy,
                    int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
    void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy,
                    int16_t* gz);
    void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
    int16_t getAccelerationX();
    int16_t getAccelerationY();
    int16_t getAccelerationZ();

    // TEMP_OUT_* registers
    int16_t getTemperature();

    /** Get 3-axis gyroscope readings.
     * These gyroscope measurement registers, along with the accelerometer
     * measurement registers, temperature measurement registers, and external sensor
     * data registers, are composed of two sets of registers: an internal register
     * set and a user-facing read register set.
     * The data within the gyroscope sensors' internal register set is always
     * updated at the Sample Rate. Meanwhile, the user-facing read register set
     * duplicates the internal register set's data values whenever the serial
     * interface is idle. This guarantees that a burst read of sensor registers will
     * read measurements from the same sampling instant. Note that if burst reads
     * are not used, the user is responsible for ensuring a set of single byte reads
     * correspond to a single sampling instant by checking the Data Ready interrupt.
     *
     * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
     * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
     * LSB in GYRO_xOUT is shown in the table below:
     *
     * <pre>
     * FS_SEL | Full Scale Range   | LSB Sensitivity
     * -------+--------------------+----------------
     * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
     * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
     * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
     * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
     * </pre>
     *
     * @param x 16-bit signed integer container for X-axis rotation
     * @param y 16-bit signed integer container for Y-axis rotation
     * @param z 16-bit signed integer container for Z-axis rotation
     * @see getMotion6()
     * @see MPU9250_RA_GYRO_XOUT_H
     */
    void getRotation(int16_t* x, int16_t* y, int16_t* z);

    /** Get X-axis gyroscope reading.
     * @return X-axis rotation measurement in 16-bit 2's complement format
     * @see getMotion6()
     * @see MPU9250_RA_GYRO_XOUT_H
     */
    int16_t getRotationX();

    /** Get Y-axis gyroscope reading.
     * @return Y-axis rotation measurement in 16-bit 2's complement format
     * @see getMotion6()
     * @see MPU9250_RA_GYRO_YOUT_H
     */
    int16_t getRotationY();

    /** Get Z-axis gyroscope reading.
     * @return Z-axis rotation measurement in 16-bit 2's complement format
     * @see getMotion6()
     * @see MPU9250_RA_GYRO_ZOUT_H
     */
    int16_t getRotationZ();

    /** Get Device ID.
     * This register is used to verify the identity of the device (0b110100, 0x34).
     * @return Device ID (6 bits only! should be 0x34)
     * @see MPU9250_RA_WHO_AM_I
     * @see MPU9250_WHO_AM_I_BIT
     * @see MPU9250_WHO_AM_I_LENGTH
     */
    uint8_t getDeviceID();

    /** Set Device ID.
     * Write a new ID into the WHO_AM_I register (no idea why this should ever be
     * necessary though).
     * @param id New device ID to set.
     * @see getDeviceID()
     * @see MPU9250_RA_WHO_AM_I
     * @see MPU9250_WHO_AM_I_BIT
     * @see MPU9250_WHO_AM_I_LENGTH
     */
    void setDeviceID(uint8_t id);

    // USER_CTRL register (DMP functions)
    bool getDMPEnabled();
    void setDMPEnabled(bool enabled);
    void resetDMP();

    // DMP_CFG_1 register
    uint8_t getDMPConfig1();
    void setDMPConfig1(uint8_t config);

    // DMP_CFG_2 register
    uint8_t getDMPConfig2();
    void setDMPConfig2(uint8_t config);

    /** Get sleep mode status.
     *
     * Setting the SLEEP bit in the register puts the device into very low power
     * sleep mode. In this mode, only the serial interface and internal registers
     * remain active, allowing for a very low standby current. Clearing this bit
     * puts the device back into normal mode. To save power, the individual standby
     * selections for each of the gyros should be used if any gyro axis is not used
     * by the application.
     * @return Current sleep mode enabled status
     * @see MPU9250_RA_PWR_MGMT_1
     * @see MPU9250_PWR1_SLEEP_BIT
     */
    bool getSleepEnabled();

    /** Set sleep mode status.
     * @param enabled New sleep mode enabled status
     * @see getSleepEnabled()
     * @see MPU9250_RA_PWR_MGMT_1
     * @see MPU9250_PWR1_SLEEP_BIT
     */
    void setSleepEnabled(bool enabled);

    /* Accumulates gyro and accelerometer data after device initialization.
     * It calculates the average of the at-rest readings and then loads the resulting
     * offsets into accelerometer and gyro bias registers.
     */
    void calibrateAccelGyro(float* gyroBias, float* accelBias);

    void calibrateMag();

    const float* getMagSensitivityAdjustment() const { return m_mag_sensitivity_adj; }
    const float* getMagBias() const { return m_mag_bias; }
    const float* getMagScale() const { return m_mag_scale; }

  private:
    uint8_t buffer[14];

    parameters m_params;
    float m_mag_sensitivity_adj[3];
    float m_mag_bias[3];
    float m_mag_scale[3];

    bool readMagData(int16_t* destination);
};

}  // namespace mpu9250

#endif /* _MPU9250_H_ */
