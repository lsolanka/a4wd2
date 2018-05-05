#pragma once

#include <stdint.h>

namespace mpu9250
{
namespace regs
{
namespace addr
{
namespace mag
{
// Magnetometer Registers
static constexpr uint8_t ADDRESS = 0x0C;
static constexpr uint8_t WHO_AM_I = 0x00;  // should return 0x48
static constexpr uint8_t INFO = 0x01;
static constexpr uint8_t ST1 = 0x02;     // data ready status bit 0
static constexpr uint8_t XOUT_L = 0x03;  // data
static constexpr uint8_t XOUT_H = 0x04;
static constexpr uint8_t YOUT_L = 0x05;
static constexpr uint8_t YOUT_H = 0x06;
static constexpr uint8_t ZOUT_L = 0x07;
static constexpr uint8_t ZOUT_H = 0x08;
static constexpr uint8_t ST2 =
        0x09;  // Data overflow bit 3 and data read error status bit 2
static constexpr uint8_t CNTL = 0x0A;    // Power down (0000), single-measurement (0001),
                                         // self-test (1000) and Fuse ROM (1111) modes on
                                         // bits 3:0
static constexpr uint8_t ASTC = 0x0C;    // Self test control
static constexpr uint8_t I2CDIS = 0x0F;  // I2C disable
static constexpr uint8_t ASAX = 0x10;    // Fuse ROM x-axis sensitivity adjustment value
static constexpr uint8_t ASAY = 0x11;    // Fuse ROM y-axis sensitivity adjustment value
static constexpr uint8_t ASAZ = 0x12;    // Fuse ROM z-axis sensitivity adjustment value
}  // namespace mag

static constexpr uint8_t ADDRESS_AD0_LOW =
        0x68;  // address pin low (GND), default for InvenSense evaluation board
static constexpr uint8_t ADDRESS_AD0_HIGH = 0x69;  // address pin high (VCC)

static constexpr uint8_t DEFAULT_ADDRESS = ADDRESS_AD0_LOW;

static constexpr uint8_t XG_OFFS_TC =
        0x00;  //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
static constexpr uint8_t YG_OFFS_TC =
        0x01;  //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
static constexpr uint8_t ZG_OFFS_TC =
        0x02;  //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
static constexpr uint8_t X_FINE_GAIN = 0x03;  //[7:0] X_FINE_GAIN
static constexpr uint8_t Y_FINE_GAIN = 0x04;  //[7:0] Y_FINE_GAIN
static constexpr uint8_t Z_FINE_GAIN = 0x05;  //[7:0] Z_FINE_GAIN
static constexpr uint8_t XA_OFFS_H = 0x06;    //[15:0] XA_OFFS
static constexpr uint8_t XA_OFFS_L_TC = 0x07;
static constexpr uint8_t YA_OFFS_H = 0x08;  //[15:0] YA_OFFS
static constexpr uint8_t YA_OFFS_L_TC = 0x09;
static constexpr uint8_t ZA_OFFS_H = 0x0A;  //[15:0] ZA_OFFS
static constexpr uint8_t ZA_OFFS_L_TC = 0x0B;
static constexpr uint8_t XG_OFFS_USRH = 0x13;  //[15:0] XG_OFFS_USR
static constexpr uint8_t XG_OFFS_USRL = 0x14;
static constexpr uint8_t YG_OFFS_USRH = 0x15;  //[15:0] YG_OFFS_USR
static constexpr uint8_t YG_OFFS_USRL = 0x16;
static constexpr uint8_t ZG_OFFS_USRH = 0x17;  //[15:0] ZG_OFFS_USR
static constexpr uint8_t ZG_OFFS_USRL = 0x18;
static constexpr uint8_t SMPLRT_DIV = 0x19;
static constexpr uint8_t CONFIG = 0x1A;
static constexpr uint8_t GYRO_CONFIG = 0x1B;
static constexpr uint8_t ACCEL_CONFIG = 0x1C;
static constexpr uint8_t FF_THR = 0x1D;
static constexpr uint8_t FF_DUR = 0x1E;
static constexpr uint8_t MOT_THR = 0x1F;
static constexpr uint8_t MOT_DUR = 0x20;
static constexpr uint8_t ZRMOT_THR = 0x21;
static constexpr uint8_t ZRMOT_DUR = 0x22;
static constexpr uint8_t FIFO_EN = 0x23;
static constexpr uint8_t I2C_MST_CTRL = 0x24;
static constexpr uint8_t I2C_SLV0_ADDR = 0x25;
static constexpr uint8_t I2C_SLV0_REG = 0x26;
static constexpr uint8_t I2C_SLV0_CTRL = 0x27;
static constexpr uint8_t I2C_SLV1_ADDR = 0x28;
static constexpr uint8_t I2C_SLV1_REG = 0x29;
static constexpr uint8_t I2C_SLV1_CTRL = 0x2A;
static constexpr uint8_t I2C_SLV2_ADDR = 0x2B;
static constexpr uint8_t I2C_SLV2_REG = 0x2C;
static constexpr uint8_t I2C_SLV2_CTRL = 0x2D;
static constexpr uint8_t I2C_SLV3_ADDR = 0x2E;
static constexpr uint8_t I2C_SLV3_REG = 0x2F;
static constexpr uint8_t I2C_SLV3_CTRL = 0x30;
static constexpr uint8_t I2C_SLV4_ADDR = 0x31;
static constexpr uint8_t I2C_SLV4_REG = 0x32;
static constexpr uint8_t I2C_SLV4_DO = 0x33;
static constexpr uint8_t I2C_SLV4_CTRL = 0x34;
static constexpr uint8_t I2C_SLV4_DI = 0x35;
static constexpr uint8_t I2C_MST_STATUS = 0x36;
static constexpr uint8_t INT_PIN_CFG = 0x37;
static constexpr uint8_t INT_ENABLE = 0x38;
static constexpr uint8_t DMP_INT_STATUS = 0x39;
static constexpr uint8_t INT_STATUS = 0x3A;
static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t ACCEL_XOUT_L = 0x3C;
static constexpr uint8_t ACCEL_YOUT_H = 0x3D;
static constexpr uint8_t ACCEL_YOUT_L = 0x3E;
static constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
static constexpr uint8_t ACCEL_ZOUT_L = 0x40;
static constexpr uint8_t TEMP_OUT_H = 0x41;
static constexpr uint8_t TEMP_OUT_L = 0x42;
static constexpr uint8_t GYRO_XOUT_H = 0x43;
static constexpr uint8_t GYRO_XOUT_L = 0x44;
static constexpr uint8_t GYRO_YOUT_H = 0x45;
static constexpr uint8_t GYRO_YOUT_L = 0x46;
static constexpr uint8_t GYRO_ZOUT_H = 0x47;
static constexpr uint8_t GYRO_ZOUT_L = 0x48;
static constexpr uint8_t EXT_SENS_DATA_00 = 0x49;
static constexpr uint8_t EXT_SENS_DATA_01 = 0x4A;
static constexpr uint8_t EXT_SENS_DATA_02 = 0x4B;
static constexpr uint8_t EXT_SENS_DATA_03 = 0x4C;
static constexpr uint8_t EXT_SENS_DATA_04 = 0x4D;
static constexpr uint8_t EXT_SENS_DATA_05 = 0x4E;
static constexpr uint8_t EXT_SENS_DATA_06 = 0x4F;
static constexpr uint8_t EXT_SENS_DATA_07 = 0x50;
static constexpr uint8_t EXT_SENS_DATA_08 = 0x51;
static constexpr uint8_t EXT_SENS_DATA_09 = 0x52;
static constexpr uint8_t EXT_SENS_DATA_10 = 0x53;
static constexpr uint8_t EXT_SENS_DATA_11 = 0x54;
static constexpr uint8_t EXT_SENS_DATA_12 = 0x55;
static constexpr uint8_t EXT_SENS_DATA_13 = 0x56;
static constexpr uint8_t EXT_SENS_DATA_14 = 0x57;
static constexpr uint8_t EXT_SENS_DATA_15 = 0x58;
static constexpr uint8_t EXT_SENS_DATA_16 = 0x59;
static constexpr uint8_t EXT_SENS_DATA_17 = 0x5A;
static constexpr uint8_t EXT_SENS_DATA_18 = 0x5B;
static constexpr uint8_t EXT_SENS_DATA_19 = 0x5C;
static constexpr uint8_t EXT_SENS_DATA_20 = 0x5D;
static constexpr uint8_t EXT_SENS_DATA_21 = 0x5E;
static constexpr uint8_t EXT_SENS_DATA_22 = 0x5F;
static constexpr uint8_t EXT_SENS_DATA_23 = 0x60;
static constexpr uint8_t MOT_DETECT_STATUS = 0x61;
static constexpr uint8_t I2C_SLV0_DO = 0x63;
static constexpr uint8_t I2C_SLV1_DO = 0x64;
static constexpr uint8_t I2C_SLV2_DO = 0x65;
static constexpr uint8_t I2C_SLV3_DO = 0x66;
static constexpr uint8_t I2C_MST_DELAY_CTRL = 0x67;
static constexpr uint8_t SIGNAL_PATH_RESET = 0x68;
static constexpr uint8_t MOT_DETECT_CTRL = 0x69;
static constexpr uint8_t USER_CTRL = 0x6A;
static constexpr uint8_t PWR_MGMT_1 = 0x6B;
static constexpr uint8_t PWR_MGMT_2 = 0x6C;
static constexpr uint8_t BANK_SEL = 0x6D;
static constexpr uint8_t MEM_START_ADDR = 0x6E;
static constexpr uint8_t MEM_R_W = 0x6F;
static constexpr uint8_t DMP_CFG_1 = 0x70;
static constexpr uint8_t DMP_CFG_2 = 0x71;
static constexpr uint8_t FIFO_COUNTH = 0x72;
static constexpr uint8_t FIFO_COUNTL = 0x73;
static constexpr uint8_t FIFO_R_W = 0x74;
static constexpr uint8_t WHO_AM_I = 0x75;

}  // namespace addr

static constexpr uint8_t TC_PWR_MODE_BIT = 7;
static constexpr uint8_t TC_OFFSET_BIT = 6;
static constexpr uint8_t TC_OFFSET_LENGTH = 6;
static constexpr uint8_t TC_OTP_BNK_VLD_BIT = 0;

static constexpr uint8_t VDDIO_LEVEL_VLOGIC = 0;
static constexpr uint8_t VDDIO_LEVEL_VDD = 1;

static constexpr uint8_t CFG_EXT_SYNC_SET_BIT = 5;
static constexpr uint8_t CFG_EXT_SYNC_SET_LENGTH = 3;
static constexpr uint8_t CFG_DLPF_CFG_BIT = 2;
static constexpr uint8_t CFG_DLPF_CFG_LENGTH = 3;

static constexpr uint8_t EXT_SYNC_DISABLED = 0x0;
static constexpr uint8_t EXT_SYNC_TEMP_OUT_L = 0x1;
static constexpr uint8_t EXT_SYNC_GYRO_XOUT_L = 0x2;
static constexpr uint8_t EXT_SYNC_GYRO_YOUT_L = 0x3;
static constexpr uint8_t EXT_SYNC_GYRO_ZOUT_L = 0x4;
static constexpr uint8_t EXT_SYNC_ACCEL_XOUT_L = 0x5;
static constexpr uint8_t EXT_SYNC_ACCEL_YOUT_L = 0x6;
static constexpr uint8_t EXT_SYNC_ACCEL_ZOUT_L = 0x7;

static constexpr uint8_t DLPF_BW_256 = 0x00;
static constexpr uint8_t DLPF_BW_188 = 0x01;
static constexpr uint8_t DLPF_BW_98 = 0x02;
static constexpr uint8_t DLPF_BW_42 = 0x03;
static constexpr uint8_t DLPF_BW_20 = 0x04;
static constexpr uint8_t DLPF_BW_10 = 0x05;
static constexpr uint8_t DLPF_BW_5 = 0x06;

static constexpr uint8_t GCONFIG_FS_SEL_BIT = 4;
static constexpr uint8_t GCONFIG_FS_SEL_LENGTH = 2;

// static constexpr uint8_t GYRO_FS_250                   0x00
// static constexpr uint8_t GYRO_FS_500                   0x01
// static constexpr uint8_t GYRO_FS_1000                  0x02
// static constexpr uint8_t GYRO_FS_2000                  0x03

static constexpr uint8_t ACONFIG_XA_ST_BIT = 7;
static constexpr uint8_t ACONFIG_YA_ST_BIT = 6;
static constexpr uint8_t ACONFIG_ZA_ST_BIT = 5;
static constexpr uint8_t ACONFIG_AFS_SEL_BIT = 4;
static constexpr uint8_t ACONFIG_AFS_SEL_LENGTH = 2;
static constexpr uint8_t ACONFIG_ACCEL_HPF_BIT = 2;
static constexpr uint8_t ACONFIG_ACCEL_HPF_LENGTH = 3;

// static constexpr uint8_t ACCEL_FS_2                    0x00
// static constexpr uint8_t ACCEL_FS_4                    0x01
// static constexpr uint8_t ACCEL_FS_8                    0x02
// static constexpr uint8_t ACCEL_FS_16                   0x03

static constexpr uint8_t DHPF_RESET = 0x00;
static constexpr uint8_t DHPF_5 = 0x01;
static constexpr uint8_t DHPF_2P5 = 0x02;
static constexpr uint8_t DHPF_1P25 = 0x03;
static constexpr uint8_t DHPF_0P63 = 0x04;
static constexpr uint8_t DHPF_HOLD = 0x07;

static constexpr uint8_t TEMP_FIFO_EN_BIT = 7;
static constexpr uint8_t XG_FIFO_EN_BIT = 6;
static constexpr uint8_t YG_FIFO_EN_BIT = 5;
static constexpr uint8_t ZG_FIFO_EN_BIT = 4;
static constexpr uint8_t ACCEL_FIFO_EN_BIT = 3;
static constexpr uint8_t SLV2_FIFO_EN_BIT = 2;
static constexpr uint8_t SLV1_FIFO_EN_BIT = 1;
static constexpr uint8_t SLV0_FIFO_EN_BIT = 0;

static constexpr uint8_t MULT_MST_EN_BIT = 7;
static constexpr uint8_t WAIT_FOR_ES_BIT = 6;
static constexpr uint8_t SLV_3_FIFO_EN_BIT = 5;
static constexpr uint8_t I2C_MST_P_NSR_BIT = 4;
static constexpr uint8_t I2C_MST_CLK_BIT = 3;
static constexpr uint8_t I2C_MST_CLK_LENGTH = 4;

static constexpr uint8_t CLOCK_DIV_348 = 0x0;
static constexpr uint8_t CLOCK_DIV_333 = 0x1;
static constexpr uint8_t CLOCK_DIV_320 = 0x2;
static constexpr uint8_t CLOCK_DIV_308 = 0x3;
static constexpr uint8_t CLOCK_DIV_296 = 0x4;
static constexpr uint8_t CLOCK_DIV_286 = 0x5;
static constexpr uint8_t CLOCK_DIV_276 = 0x6;
static constexpr uint8_t CLOCK_DIV_267 = 0x7;
static constexpr uint8_t CLOCK_DIV_258 = 0x8;
static constexpr uint8_t CLOCK_DIV_500 = 0x9;
static constexpr uint8_t CLOCK_DIV_471 = 0xA;
static constexpr uint8_t CLOCK_DIV_444 = 0xB;
static constexpr uint8_t CLOCK_DIV_421 = 0xC;
static constexpr uint8_t CLOCK_DIV_400 = 0xD;
static constexpr uint8_t CLOCK_DIV_381 = 0xE;
static constexpr uint8_t CLOCK_DIV_364 = 0xF;

static constexpr uint8_t I2C_SLV_RW_BIT = 7;
static constexpr uint8_t I2C_SLV_ADDR_BIT = 6;
static constexpr uint8_t I2C_SLV_ADDR_LENGTH = 7;
static constexpr uint8_t I2C_SLV_EN_BIT = 7;
static constexpr uint8_t I2C_SLV_BYTE_SW_BIT = 6;
static constexpr uint8_t I2C_SLV_REG_DIS_BIT = 5;
static constexpr uint8_t I2C_SLV_GRP_BIT = 4;
static constexpr uint8_t I2C_SLV_LEN_BIT = 3;
static constexpr uint8_t I2C_SLV_LEN_LENGTH = 4;

static constexpr uint8_t I2C_SLV4_RW_BIT = 7;
static constexpr uint8_t I2C_SLV4_ADDR_BIT = 6;
static constexpr uint8_t I2C_SLV4_ADDR_LENGTH = 7;
static constexpr uint8_t I2C_SLV4_EN_BIT = 7;
static constexpr uint8_t I2C_SLV4_INT_EN_BIT = 6;
static constexpr uint8_t I2C_SLV4_REG_DIS_BIT = 5;
static constexpr uint8_t I2C_SLV4_MST_DLY_BIT = 4;
static constexpr uint8_t I2C_SLV4_MST_DLY_LENGTH = 5;

static constexpr uint8_t MST_PASS_THROUGH_BIT = 7;
static constexpr uint8_t MST_I2C_SLV4_DONE_BIT = 6;
static constexpr uint8_t MST_I2C_LOST_ARB_BIT = 5;
static constexpr uint8_t MST_I2C_SLV4_NACK_BIT = 4;
static constexpr uint8_t MST_I2C_SLV3_NACK_BIT = 3;
static constexpr uint8_t MST_I2C_SLV2_NACK_BIT = 2;
static constexpr uint8_t MST_I2C_SLV1_NACK_BIT = 1;
static constexpr uint8_t MST_I2C_SLV0_NACK_BIT = 0;

static constexpr uint8_t INTCFG_INT_LEVEL_BIT = 7;
static constexpr uint8_t INTCFG_INT_OPEN_BIT = 6;
static constexpr uint8_t INTCFG_LATCH_INT_EN_BIT = 5;
static constexpr uint8_t INTCFG_INT_RD_CLEAR_BIT = 4;
static constexpr uint8_t INTCFG_FSYNC_INT_LEVEL_BIT = 3;
static constexpr uint8_t INTCFG_FSYNC_INT_EN_BIT = 2;
static constexpr uint8_t INTCFG_I2C_BYPASS_EN_BIT = 1;
static constexpr uint8_t INTCFG_CLKOUT_EN_BIT = 0;

static constexpr uint8_t INTMODE_ACTIVEHIGH = 0x00;
static constexpr uint8_t INTMODE_ACTIVELOW = 0x01;

static constexpr uint8_t INTDRV_PUSHPULL = 0x00;
static constexpr uint8_t INTDRV_OPENDRAIN = 0x01;

static constexpr uint8_t INTLATCH_50USPULSE = 0x00;
static constexpr uint8_t INTLATCH_WAITCLEAR = 0x01;

static constexpr uint8_t INTCLEAR_STATUSREAD = 0x00;
static constexpr uint8_t INTCLEAR_ANYREAD = 0x01;

static constexpr uint8_t INTERRUPT_FF_BIT = 7;
static constexpr uint8_t INTERRUPT_MOT_BIT = 6;
static constexpr uint8_t INTERRUPT_ZMOT_BIT = 5;
static constexpr uint8_t INTERRUPT_FIFO_OFLOW_BIT = 4;
static constexpr uint8_t INTERRUPT_I2C_MST_INT_BIT = 3;
static constexpr uint8_t INTERRUPT_PLL_RDY_INT_BIT = 2;
static constexpr uint8_t INTERRUPT_DMP_INT_BIT = 1;
static constexpr uint8_t INTERRUPT_DATA_RDY_BIT = 0;

// TODO: figure out what these actually do
// UMPL source code is not very obivous
static constexpr uint8_t DMPINT_5_BIT = 5;
static constexpr uint8_t DMPINT_4_BIT = 4;
static constexpr uint8_t DMPINT_3_BIT = 3;
static constexpr uint8_t DMPINT_2_BIT = 2;
static constexpr uint8_t DMPINT_1_BIT = 1;
static constexpr uint8_t DMPINT_0_BIT = 0;

static constexpr uint8_t MOTION_MOT_XNEG_BIT = 7;
static constexpr uint8_t MOTION_MOT_XPOS_BIT = 6;
static constexpr uint8_t MOTION_MOT_YNEG_BIT = 5;
static constexpr uint8_t MOTION_MOT_YPOS_BIT = 4;
static constexpr uint8_t MOTION_MOT_ZNEG_BIT = 3;
static constexpr uint8_t MOTION_MOT_ZPOS_BIT = 2;
static constexpr uint8_t MOTION_MOT_ZRMOT_BIT = 0;

static constexpr uint8_t DELAYCTRL_DELAY_ES_SHADOW_BIT = 7;
static constexpr uint8_t DELAYCTRL_I2C_SLV4_DLY_EN_BIT = 4;
static constexpr uint8_t DELAYCTRL_I2C_SLV3_DLY_EN_BIT = 3;
static constexpr uint8_t DELAYCTRL_I2C_SLV2_DLY_EN_BIT = 2;
static constexpr uint8_t DELAYCTRL_I2C_SLV1_DLY_EN_BIT = 1;
static constexpr uint8_t DELAYCTRL_I2C_SLV0_DLY_EN_BIT = 0;

static constexpr uint8_t PATHRESET_GYRO_RESET_BIT = 2;
static constexpr uint8_t PATHRESET_ACCEL_RESET_BIT = 1;
static constexpr uint8_t PATHRESET_TEMP_RESET_BIT = 0;

static constexpr uint8_t DETECT_ACCEL_ON_DELAY_BIT = 5;
static constexpr uint8_t DETECT_ACCEL_ON_DELAY_LENGTH = 2;
static constexpr uint8_t DETECT_FF_COUNT_BIT = 3;
static constexpr uint8_t DETECT_FF_COUNT_LENGTH = 2;
static constexpr uint8_t DETECT_MOT_COUNT_BIT = 1;
static constexpr uint8_t DETECT_MOT_COUNT_LENGTH = 2;

static constexpr uint8_t DETECT_DECREMENT_RESET = 0x0;
static constexpr uint8_t DETECT_DECREMENT_1 = 0x1;
static constexpr uint8_t DETECT_DECREMENT_2 = 0x2;
static constexpr uint8_t DETECT_DECREMENT_4 = 0x3;

static constexpr uint8_t USERCTRL_DMP_EN_BIT = 7;
static constexpr uint8_t USERCTRL_FIFO_EN_BIT = 6;
static constexpr uint8_t USERCTRL_I2C_MST_EN_BIT = 5;
static constexpr uint8_t USERCTRL_I2C_IF_DIS_BIT = 4;
static constexpr uint8_t USERCTRL_DMP_RESET_BIT = 3;
static constexpr uint8_t USERCTRL_FIFO_RESET_BIT = 2;
static constexpr uint8_t USERCTRL_I2C_MST_RESET_BIT = 1;
static constexpr uint8_t USERCTRL_SIG_COND_RESET_BIT = 0;

static constexpr uint8_t PWR1_DEVICE_RESET_BIT = 7;
static constexpr uint8_t PWR1_SLEEP_BIT = 6;
static constexpr uint8_t PWR1_CYCLE_BIT = 5;
static constexpr uint8_t PWR1_TEMP_DIS_BIT = 3;
static constexpr uint8_t PWR1_CLKSEL_BIT = 2;
static constexpr uint8_t PWR1_CLKSEL_LENGTH = 3;

static constexpr uint8_t CLOCK_INTERNAL = 0x00;
static constexpr uint8_t CLOCK_PLL_XGYRO = 0x01;
static constexpr uint8_t CLOCK_PLL_YGYRO = 0x02;
static constexpr uint8_t CLOCK_PLL_ZGYRO = 0x03;
static constexpr uint8_t CLOCK_PLL_EXT32K = 0x04;
static constexpr uint8_t CLOCK_PLL_EXT19M = 0x05;
static constexpr uint8_t CLOCK_KEEP_RESET = 0x07;

static constexpr uint8_t PWR2_LP_WAKE_CTRL_BIT = 7;
static constexpr uint8_t PWR2_LP_WAKE_CTRL_LENGTH = 2;
static constexpr uint8_t PWR2_STBY_XA_BIT = 5;
static constexpr uint8_t PWR2_STBY_YA_BIT = 4;
static constexpr uint8_t PWR2_STBY_ZA_BIT = 3;
static constexpr uint8_t PWR2_STBY_XG_BIT = 2;
static constexpr uint8_t PWR2_STBY_YG_BIT = 1;
static constexpr uint8_t PWR2_STBY_ZG_BIT = 0;

static constexpr uint8_t WAKE_FREQ_1P25 = 0x0;
static constexpr uint8_t WAKE_FREQ_2P5 = 0x1;
static constexpr uint8_t WAKE_FREQ_5 = 0x2;
static constexpr uint8_t WAKE_FREQ_10 = 0x3;

static constexpr uint8_t BANKSEL_PRFTCH_EN_BIT = 6;
static constexpr uint8_t BANKSEL_CFG_USER_BANK_BIT = 5;
static constexpr uint8_t BANKSEL_MEM_SEL_BIT = 4;
static constexpr uint8_t BANKSEL_MEM_SEL_LENGTH = 5;

static constexpr uint8_t WHO_AM_I_BIT = 6;
static constexpr uint8_t WHO_AM_I_LENGTH = 8;

static constexpr uint8_t DMP_MEMORY_BANKS = 8;
static constexpr uint16_t DMP_MEMORY_BANK_SIZE = 256;
static constexpr uint8_t DMP_MEMORY_CHUNK_SIZE = 16;

}  // namespace regs
}  // namespace mpu9250
