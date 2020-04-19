#pragma once

#include <stdint.h>


//#define MPU9250_I2C_ADDRESS 0xD0 //(pin AD0/SD0 is logic low) (0x68)
//#define MPU9250_I2C_ADDRESS 0xD2 // (pin AD0/SD0 is logic high) (0x69)
//#define MPU9250_I2C_ADDRESS 0x68 //(pin AD0/SD0 is logic low) (0x68)
//#define MPU9250_I2C_ADDRESS 0x69  // (pin AD0/SD0 is logic high) (0x69)

/*******************************************************************************
 *** MPU 9250 registers
 ******************************************************************************/

/// **************************
/// *** Accel/gyro MPU6500 ***
/// **************************
#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F
#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG_2 0x1D
#define LP_ACCEL_ODR 0x1E
#define WOM_THR 0x1F
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
#define I2C_SLV0_DO 0x63
#define I2C_SLV1_DO 0x64
#define I2C_SLV2_DO 0x65
#define I2C_SLV3_DO 0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

/*******************************************************************************
 *** MPU9250 configuration bits
 ******************************************************************************/
#define GYRO_FS_250         0x00
#define GYRO_FS_500         0x01
#define GYRO_FS_1000        0x02
#define GYRO_FS_2000        0x03

#define ACCEL_FS_2          0x00
#define ACCEL_FS_4          0x01
#define ACCEL_FS_8          0x02
#define ACCEL_FS_16         0x03

#define DEG_PER_LSB_250     (0.00762939453125f) //(float)((2 * 250.f) / 65536.0)
#define DEG_PER_LSB_500     (0.0152587890625f) //(float)((2 * 500.f) / 65536.0)
#define DEG_PER_LSB_1000    (0.030517578125f) //(float)((2 * 1000.f) / 65536.0)
#define DEG_PER_LSB_2000    (0.06103515625f) //(float)((2 * 2000.f) / 65536.0)

#define G_PER_LSB_2         (0.00006103515625f) //(float)((2 * 2) / 65536.0)
#define G_PER_LSB_4         (0.0001220703125f) //(float)((2 * 4) / 65536.0)
#define G_PER_LSB_8         (0.000244140625f) //(float)((2 * 8) / 65536.0)
#define G_PER_LSB_16        (0.00048828125f) //(float)((2 * 16) / 65536.0)

#define WHO_AM_I_ID         0x71 // See register map pp.44
#define WHO_AM_I_ID2        0x68 // See register map pp.44

// Temperature
#define ROOM_TEMP_OFFSET    (0.0f) // at 21 degree Celsius
#define TEMP_SENSITIVITY    (333.87f) // LSB/degree Celsius (untrimmed)

/// *************************************
/// *** AK8963 magnetometer registers ***
/// *************************************
#define AK8963_I2C_ADDRESS 0x0C
#define AK8963_I2C_ADDRESS_1 0x0D
#define AK8963_I2C_ADDRESS_2 0x0E
#define AK8963_I2C_ADDRESS_3 0x0F

#define WIA 0x00
#define INFO 0x01
#define ST1 0x02
#define HXL 0x03
#define HXH 0x04
#define HYL 0x05
#define HYH 0x06
#define HZL 0x07
#define HZH 0x08
#define ST2 0x09
#define CNTL1 0x0A
#define CNTL2 0x0B
#define ASTC 0x0C
#define TS1 0x0D
#define TS2 0x0E
#define I2CDIS 0x0F
#define ASAX 0x10
#define ASAY 0x11
#define ASAZ 0x12

#define MAG_WIA 0x48

/// **********************************************
/// *** AK8963 magnetometer configuration bits ***
/// **********************************************
#define AK8963_ST1_DRDY_BIT       0

#define AK8963_ST2_HOFL_BIT       3
#define AK8963_ST2_DERR_BIT       2

#define AK8963_CNTL_MODE_BIT      3
#define AK8963_CNTL_MODE_LENGTH   4

#define AK8963_MODE_POWERDOWN     0x00
#define AK8963_MODE_SINGLE        0x01
#define AK8963_MODE_CONT1         0x02
#define AK8963_MODE_CONT2         0x06
#define AK8963_MODE_EXTTRIG       0x04
#define AK8963_MODE_SELFTEST      0x08
#define AK8963_MODE_FUSEROM       0x0F
#define AK8963_MODE_14BIT         0x00
#define AK8963_MODE_16BIT         0x10

#define AK8963_ASTC_SELF_BIT      6

#define AK8963_I2CDIS_BIT         0

#define AK8963_ST_X_MIN           (int16_t)(-200)
#define AK8963_ST_X_MAX           (int16_t)(200)
#define AK8963_ST_Y_MIN           (int16_t)(-200)
#define AK8963_ST_Y_MAX           (int16_t)(200)
#define AK8963_ST_Z_MIN           (int16_t)(-3200)
#define AK8963_ST_Z_MAX           (int16_t)(-800)

/// Mag
#define MAG_FS_RANGE        (4912.0f) // +/-4900 [uT]
#define MAG_SENS            (0.15f) // 0.15 uT/LSB (16-bit)

/// Other useful declarations
#ifndef N_AXES
#define N_AXES 3
#endif

#ifndef I2C_READ_WRITE_BIT
#define I2C_READ_WRITE_BIT 0x01
#endif

#ifndef SPI_READ_WRITE_BIT
#define SPI_READ_WRITE_BIT 0x80
#endif


#define GRAVITY                 (9.80665f) // [m/s2]
//#define DEG_TO_RAD              (M_PI/180.0f)
#define UTESLA_TO_TESLA         (1e-6) // micro Tesla to Tesla

#define REG_DATA_SIZE       22 // Number of registers to burst read from the IMU

#define IMU_UPDATE_FREQ     (500) // [Hz]
#define IMU_UPDATE_DT       ((float)(1./IMU_UPDATE_FREQ)) // [s]


typedef struct Mpu9250_s {
    
    uint8_t register_data[REG_DATA_SIZE]; /*!< Raw register data */

    /// Raw register values
    int16_t a_rawi[N_AXES]; /*!< Accelerometer raw register data */ 
    int16_t g_rawi[N_AXES]; /*!< Gyroscope raw register data */ 
    int16_t m_rawi[N_AXES]; /*!< Magnetometer raw register data */ 

    /// Raw values
    float a_raw[N_AXES]; // accelerometer
    float g_raw[N_AXES]; // gyroscope
    float m_raw[N_AXES]; // magnetometer

    /// Corrected values (bias and scaling)
    float a[N_AXES]; // acc.
    float g[N_AXES]; // gyro.
    float m[N_AXES]; // mag.

    /// Filtered values
    float a_filt[N_AXES]; // acc.
    float g_filt[N_AXES]; // gyro.
    float m_filt[N_AXES]; // mag.

    /// Filter cutoff frequencies
    float a_cutoff_freq;
    float g_cutoff_freq;
    float m_cutoff_freq;

    /// Filter RC values
    float a_rc;
    float g_rc;
    float m_rc;

    float temperature;

    /// Magnetometer azimuth angle
    float mag_azimuth;

    float a_gain;
    float g_gain;
    float m_gain[N_AXES]; // Magnetometer sensitivity values

    /// Biases
    float ba[N_AXES];
    float bg[N_AXES];
    float bm[N_AXES];

    /// Scaling matrices
    float Ka[N_AXES][N_AXES];
    float Kg[N_AXES][N_AXES];
    float Km[N_AXES][N_AXES];

    /// Variance
    float va[N_AXES];
    float vg[N_AXES];
    float vm[N_AXES];

    /// Standard deviation
    float sa[N_AXES];
    float sg[N_AXES];
    float sm[N_AXES];

    /// delta time in seconds for the last raw sample measurement
    float dt;

    uint8_t isImuInit;
    uint8_t isMagInit;
    uint8_t isCalibrated;
    uint8_t isFiltered;
    uint8_t isCorrected;

    uint16_t sample_frequency; // [Hz]
    uint16_t sample_period; // [ms]

    uint32_t ag_error_count;
    uint32_t m_error_count;

    uint32_t calib_time; // [s]

} Mpu9250_t;

#define MAX_ERRORS              1000

/*******************************************************************************
 *** IMU MPU9250 functions
 ******************************************************************************/

uint8_t mpu9250_init(Mpu9250_t* mpu9250);

uint8_t mpu9250_init_mag(Mpu9250_t* mpu9250);

uint8_t mpu9250_write_byte_mag(uint8_t _reg, uint8_t _data);

uint8_t mpu9250_read_byte_mag(uint8_t _reg, uint8_t* _data);

uint8_t mpu9250_get_acc_gyro_mag_temp(Mpu9250_t* mpu9250);

uint8_t mpu9250_read_data_register(Mpu9250_t* mpu9250);

void mpu9250_extract_data_register(Mpu9250_t* mpu9250);

uint8_t mpu9250_close(Mpu9250_t* mpu9250);

uint8_t mpu9250_static_calibration(Mpu9250_t* mpu9250);

void mpu9250_get_azimuth(Mpu9250_t* mpu9250);

void mpu9250_low_pass_filter(Mpu9250_t* mpu9250);

void mpu9250_correct(Mpu9250_t* mpu9250);

uint8_t mpu9250_set_cut_off_freq(Mpu9250_t* mpu9250, float* cut_off_freqs);

void mpu9250_set_filtering(Mpu9250_t* mpu9250, uint8_t value);

void mpu9250_set_corrected(Mpu9250_t* mpu9250, uint8_t value);

void mpu9250_set_calib_time(Mpu9250_t* mpu9250, uint32_t time); // time in s

uint8_t mpu9250_getIntStatus(void);

uint8_t mpu9250_check_devId(Mpu9250_t* mpu9250);

uint8_t mpu9250_check_mag_devId(Mpu9250_t* mpu9250);

void mpu9250_set_smplrtDiv(Mpu9250_t* mpu9250, uint8_t _div);

void mpu9250_set_smplrtFreq(Mpu9250_t* mpu9250, uint16_t _freq);

void mpu9250_set_acc_g_per_lsb(Mpu9250_t* mpu9250, uint8_t range);

void mpu9250_set_gyro_deg_per_lsb(Mpu9250_t* mpu9250, uint8_t range);

void mpu9250_reset(Mpu9250_t* mpu9250);

uint8_t write_and_check_register(uint8_t reg, uint8_t data);