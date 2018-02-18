#include "ArduinoJson.hpp"
#include "I2Cdev/I2Cdev.h"
#include "MPU9250.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Magnetometer Registers
#define AK8963_ADDRESS 0x0C
#define AK8963_WHO_AM_I 0x00 // should return 0x48
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02    // data ready status bit 0
#define AK8963_XOUT_L 0x03 // data
#define AK8963_XOUT_H 0x04
#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_ST2 0x09 // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL                                                                      \
    0x0A // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM
         // (1111) modes on bits 3:0
#define AK8963_ASTC 0x0C   // Self test control
#define AK8963_I2CDIS 0x0F // I2C disable
#define AK8963_ASAX 0x10   // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY 0x11   // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ 0x12   // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A 0x10

#define XG_OFFSET_H 0x13 // User-defined trim values for gyroscope
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define LP_ACCEL_ODR 0x1E
#define WOM_THR 0x1F

#define MOT_DUR                                                                          \
    0x20 // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB =
         // 1 ms
#define ZMOT_THR 0x21 // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR                                                                        \
    0x22 // Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
         // LSB = 64 ms

#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A
#define I2C_SLV2_ADDR 0x2B
#define I2C_SLV2_REG 0x2C
#define I2C_SLV2_CTRL 0x2D
#define I2C_SLV3_ADDR 0x2E
#define I2C_SLV3_REG 0x2F
#define I2C_SLV3_CTRL 0x30
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_DO 0x33
#define I2C_SLV4_CTRL 0x34
#define I2C_SLV4_DI 0x35
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define DMP_INT_STATUS 0x39 // Check DMP interrupt
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO 0x63
#define I2C_SLV1_DO 0x64
#define I2C_SLV2_DO 0x65
#define I2C_SLV3_DO 0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL 0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1 0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2 0x6C
#define DMP_BANK 0x6D // Activates a specific bank in the DMP
#define DMP_RW_PNT                                                                       \
    0x6E // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG 0x6F // Register in DMP from which to read or to which to write
#define DMP_REG_1 0x70
#define DMP_REG_2 0x71
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

// Using the MPU9250Teensy 3.1 Add-On shield, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69 // Device address when ADO = 1
#define AK8963_ADDRESS 0x0C  //  Address of magnetometer
#define MS5637_ADDRESS 0x76  // Address of altimeter
#else
#define MPU9250_ADDRESS 0x68 // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C  //  Address of magnetometer
#define MS5637_ADDRESS 0x76  // Address of altimeter
#endif

#define SRF_BASE_ADDRESS 0x70 // Address of the SRF08
#define NUM_SENSORS 2
#define CMD                                                                              \
    (uint8_t)0x00 // Command byte, values of 0 being sent with write have to be masked as
                  // a byte to stop them being misinterpreted as NULL this is a bug with
                  // arduino 1.0
#define LIGHTBYTE 0x01 // Byte to read light sensor
#define RANGEBYTE 0x02 // Byte for start of ranging data
#define GAINBYTE 0x01

using namespace ArduinoJson;

enum Ascale
{
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale
{
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

enum Mscale
{
    MFS_14BITS = 0, // 0.6 mG per LSB
    MFS_16BITS      // 0.15 mG per LSB
};

const uint8_t Gscale = GFS_250DPS;
const uint8_t Ascale = AFS_2G;

MPU9250 imu;
bool imu_initialized = false;

float gyroBias[3] = {0, 0, 0};
float accelBias[3] = {0, 0, 0};

// Function which accumulates gyro and accelerometer data after device initialization. It
// calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and
// gyro bias registers.
void calibrate_accel_gyro(float* gyroBias, float* accelBias)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    I2Cdev::writeByte(MPU9250_ADDRESS, PWR_MGMT_1,
                      0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if
    // ready
    // else use the internal oscillator, bits 2:0 = 001
    I2Cdev::writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    I2Cdev::writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    I2Cdev::writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00); // Disable all interrupts
    I2Cdev::writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);    // Disable FIFO
    I2Cdev::writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Turn on internal clock source
    I2Cdev::writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    I2Cdev::writeByte(MPU9250_ADDRESS, USER_CTRL,
                      0x00); // Disable FIFO and I2C master modes
    I2Cdev::writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C); // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    I2Cdev::writeByte(MPU9250_ADDRESS, CONFIG, 0x01);     // Set low-pass filter to 188 Hz
    I2Cdev::writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
    I2Cdev::writeByte(
            MPU9250_ADDRESS, GYRO_CONFIG,
            0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    I2Cdev::writeByte(MPU9250_ADDRESS, ACCEL_CONFIG,
                      0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    I2Cdev::writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40); // Enable FIFO
    I2Cdev::writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);   // Enable gyro and accelerometer
    // sensors for FIFO  (max size 512
    // bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    I2Cdev::writeByte(MPU9250_ADDRESS, FIFO_EN,
                      0x00); // Disable gyro and accelerometer sensors for FIFO
    I2Cdev::readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2,
                      &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count /
                   12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        I2Cdev::readBytes(MPU9250_ADDRESS, FIFO_R_W, 12,
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
    I2Cdev::writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    I2Cdev::writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    I2Cdev::writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    I2Cdev::writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    I2Cdev::writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    I2Cdev::writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    gyroBias[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
    gyroBias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    gyroBias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias
    // registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on
    // boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is
    // used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048
    // LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {
            0, 0, 0}; // A place to hold the factory accelerometer trim biases
    I2Cdev::readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2,
                      &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    I2Cdev::readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    I2Cdev::readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte
                         // of accelerometer bias registers
    uint8_t mask_bit[3] = {
            0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++)
    {
        if ((accel_bias_reg[ii] & mask))
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that
                                 // fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias
    // from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged
                                              // accelerometer bias scaled to 2048 LSB/g
                                              // (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when
                                     // writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when
                                     // writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when
                                     // writing back to accelerometer bias registers

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    /*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
      writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
      writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
      writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
      writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
      writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
    */
    // Output scaled accelerometer biases for display in the main program
    accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
    accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
    accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

void initMPU9250()
{
    // wake up device
    I2Cdev::writeByte(MPU9250_ADDRESS, PWR_MGMT_1,
                      0x00); // Clear sleep mode bit (6), enable all sensors
    delay(100);              // Wait for all registers to reset

    // get stable time source
    I2Cdev::writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Auto select clock source to
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
    I2Cdev::writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    I2Cdev::writeByte(
            MPU9250_ADDRESS, SMPLRT_DIV,
            0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
                   // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into
    // positions 4:3
    uint8_t c = I2Cdev::readByte(MPU9250_ADDRESS,
                                 GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x03;       // Clear Fchoice bits [1:0]
    c = c & ~0x18;       // Clear GFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
    // GYRO_CONFIG
    I2Cdev::writeByte(MPU9250_ADDRESS, GYRO_CONFIG,
                      c); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = I2Cdev::readByte(MPU9250_ADDRESS,
                         ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;       // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
    I2Cdev::writeByte(MPU9250_ADDRESS, ACCEL_CONFIG,
                      c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = I2Cdev::readByte(MPU9250_ADDRESS,
                         ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    I2Cdev::writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2,
                      c); // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the
    // SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until
    // interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    //   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    I2Cdev::writeByte(MPU9250_ADDRESS, INT_PIN_CFG,
                      0x12); // INT is 50 microsecond pulse and any read to clear
    I2Cdev::writeByte(MPU9250_ADDRESS, INT_ENABLE,
                      0x01); // Enable data ready (bit 0) interrupt
    delay(100);
}

void setup()
{
    Wire.begin();
    delay(100);

    // Reduce range
    for (int sensor_idx = 0; sensor_idx < NUM_SENSORS; ++sensor_idx)
    {
        Wire.beginTransmission(SRF_BASE_ADDRESS + sensor_idx);
        Wire.write(RANGEBYTE);
        Wire.write(0x46);
        Wire.endTransmission();

        Wire.beginTransmission(SRF_BASE_ADDRESS + sensor_idx);
        Wire.write(GAINBYTE);
        Wire.write(0x18);
        Wire.endTransmission();
    }

    Serial.begin(57600);

    // IMU init
    imu.initialize();
    if (imu.testConnection())
    {
        imu_initialized = true;
    }

    delay(5000);

    calibrate_accel_gyro(gyroBias, accelBias);
    Serial.print("# accel biases (mg): ");
    Serial.print(1000. * accelBias[0]);
    Serial.print(" ");
    Serial.print(1000. * accelBias[1]);
    Serial.print(" ");
    Serial.println(1000. * accelBias[2]);

    Serial.print("# gyro biases (dps)");
    Serial.print(" ");
    Serial.print(gyroBias[0]);
    Serial.print(" ");
    Serial.print(gyroBias[1]);
    Serial.print(" ");
    Serial.println(gyroBias[2]);

    initMPU9250();
    Serial.println("# MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = I2Cdev::readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
    Serial.print("# AK8963: I AM ");
    Serial.print(d, HEX);
    Serial.print("; I should be ");
    Serial.println(0x48, HEX);
}

float get_accel_resolution()
{
    switch (Ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
        // value:
        case AFS_2G:
            return 2.0 / 32768.0;
            break;
        case AFS_4G:
            return 4.0 / 32768.0;
            break;
        case AFS_8G:
            return 8.0 / 32768.0;
            break;
        case AFS_16G:
            return 16.0 / 32768.0;
            break;
    }
}

void triggerRanging(uint8_t address)
{
    Wire.beginTransmission(address);
    Wire.write(CMD);
    Wire.write(0x51);
    Wire.endTransmission();

    delay(30);
}

/** Get a range reading from a ranger at the specified address. **/
int read_range(uint8_t address)
{
    uint8_t highByte = 0x00;
    uint8_t lowByte = 0x00;

    int range = 0;

    triggerRanging(address);

    Wire.beginTransmission(address);
    Wire.write(RANGEBYTE);
    Wire.endTransmission();

    Wire.requestFrom((int)address, 2);
    while (Wire.available() < 2)
        ;
    highByte = Wire.read();
    lowByte = Wire.read();

    range = (highByte << 8) + lowByte;

    return (range);
}

/** Get light sensor reading from the sensor at the specified address **/
int read_light(uint8_t address)
{
    Wire.beginTransmission(address);
    Wire.write(LIGHTBYTE);
    Wire.endTransmission();

    Wire.requestFrom((int)address, 1);
    while (Wire.available() < 0)
        ;
    return Wire.read();
}

void loop()
{
    static const float accel_resolution = get_accel_resolution();

    // To generate an object of 3 srf08 values we need roughly 100 bytes
    // const int BUFFER_SIZE = 100;
    // StaticJsonBuffer<BUFFER_SIZE> buffer;
    // JsonObject& root = buffer.createObject();
    // JsonObject& srf08_json = root.createNestedObject("srf08");

    // for (int sensor_idx = 0; sensor_idx < NUM_SENSORS; ++sensor_idx)
    //{
    //    uint8_t sensor_address = SRF_BASE_ADDRESS + sensor_idx;
    //    int range = read_range(sensor_address);
    //    int light = read_light(sensor_address);

    //    srf08_json["a"] = sensor_address;
    //    srf08_json["r"] = range;
    //    srf08_json["l"] = light;

    //    root.printTo(Serial);
    //    Serial.println();
    //}

    {
        const int IMU_BUFFER_SIZE = 300;
        StaticJsonBuffer<IMU_BUFFER_SIZE> buffer;
        JsonObject& root = buffer.createObject();
        JsonObject& imu_json = root.createNestedObject("imu");

        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        imu_json["ax"] = ax * accel_resolution - accelBias[0];
        imu_json["ay"] = ay * accel_resolution - accelBias[1];
        imu_json["az"] = az * accel_resolution - accelBias[2];

        imu_json["gx"] = gx;
        imu_json["gy"] = gy;
        imu_json["gz"] = gz;

        imu_json["mx"] = mx;
        imu_json["my"] = my;
        imu_json["mz"] = mz;
        root.printTo(Serial);
        Serial.println();
    }
}

//// I2C address is double of what is the address on TWI
//// Range of addresses on TWI - 0x70 - 0x7f
// void setAddressTwi(uint8_t address_twi)
//{
//  if (address_twi >= 0x70 && address_twi <= 0x7f)
//  {
//    Wire.beginTransmission(SRF_ADDRESS);
//    Wire.write(CMD);
//    Wire.write((uint8_t)0xA0);
//    Wire.endTransmission();
//
//    Wire.beginTransmission(SRF_ADDRESS);
//    Wire.write(CMD);
//    Wire.write((uint8_t)0xAA);
//    Wire.endTransmission();
//
//    Wire.beginTransmission(SRF_ADDRESS);
//    Wire.write(CMD);
//    Wire.write((uint8_t)0xA5);
//    Wire.endTransmission();
//
//    Wire.beginTransmission(SRF_ADDRESS);
//    Wire.write(CMD);
//    Wire.write(address_twi * 2);
//    Wire.endTransmission();
//  }
//}
//
