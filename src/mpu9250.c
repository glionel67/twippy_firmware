#include "mpu9250.h"
#include "spi.h"
#include "main.h"

#include <math.h>

#ifdef M_PI
#define PI ((float)M_PI)
#else
#define PI (3.141592653589793f)
#endif

#define N_TRIES 5
//#define DEG_TO_RAD (PI/180.0f)

//static const float deg_to_rad = PI / 180.f;


uint8_t mpu9250_init(Mpu9250_t* mpu9250) {
    uint8_t ret = 0;
    uint8_t i = 0, j = 0;

    if (mpu9250->isImuInit) {
        return 1; 
    }

    HAL_Delay(50); // For the IMU to init. after power up

    for (i=0;i<N_TRIES;i++) {
        write_byte_spi1(USER_CTRL, 0x30);
        HAL_Delay(1); // 1 ms
        
        // Software reset
        write_byte_spi1(PWR_MGMT_1, 0x80);
        HAL_Delay(100); // 100 ms
        
        write_byte_spi1(USER_CTRL, 0x30);
        write_byte_spi1(PWR_MGMT_1, 0x01); // Wake-up chip
        HAL_Delay(1); // 1 ms
        
        // Check if device has woken up
        ret = mpu9250_check_devId(mpu9250);
        if (ret) {
            break;
        }
    }
    if (i >= N_TRIES) {
        mpu9250->isImuInit = 0;
        return 0;
    }

    write_byte_spi1(PWR_MGMT_2, 0x00); // Enable Gyro XYZ and Accelero XYZ

    mpu9250_set_smplrtFreq(mpu9250, IMU_UPDATE_FREQ);

    write_byte_spi1(CONFIG, 0x01); // DLPF_CFG = 1 (Fs=1KHz)
    write_byte_spi1(GYRO_CONFIG, 0x10); // +/-1000dps,  FCHOICE_B = 0b00 (== FCHOICE=0b11) (Fs=1KHz)
    write_byte_spi1(ACCEL_CONFIG, 0x00); // ACCEL_FS_SEL = 00 (+/- 2g)
    write_byte_spi1(ACCEL_CONFIG_2, 0x01); // accel_fchoice = 1 (== accel_fchoice_b = 0), A_DLPF_CFG = 0b001
    write_byte_spi1(FIFO_EN, 0x00); // Disable all FIFO
    write_byte_spi1(INT_PIN_CFG, 0x10); // Bypass mode disabled + INT_ANYRD_2CLEAR + INT active high + push-pull + in pulse width of 50 us
    write_byte_spi1(INT_ENABLE, 0x01); // RAW_RDY_EN

    mpu9250->isImuInit = 1;

    // Magnetometer
    ret = mpu9250_init_mag(mpu9250);
    if (ret)
        mpu9250->isMagInit = 1;
    else
        mpu9250->isMagInit = 0;

    // Init. mpu9250 structure
    mpu9250->a_gain = G_PER_LSB_2;
    mpu9250->g_gain = DEG_PER_LSB_1000;
    mpu9250->m_gain[0] = MAG_SENS;
    mpu9250->m_gain[1] = MAG_SENS;
    mpu9250->m_gain[2] = MAG_SENS;
    mpu9250->ag_error_count = 0;
    mpu9250->m_error_count = 0;
    mpu9250->a_cutoff_freq = 0.f;
    mpu9250->g_cutoff_freq = 0.f;
    mpu9250->m_cutoff_freq = 0.f;
    mpu9250->a_rc = 0.f;
    mpu9250->g_rc = 0.f;
    mpu9250->m_rc = 0.f;
    mpu9250->calib_time = 10; // [s]
    mpu9250->isCalibrated = 0;
    mpu9250->isFiltered = 0;
    mpu9250->isCorrected = 0;
    for (i=0;i<N_AXES;i++) {
        mpu9250->a_rawi[i] = 0;
        mpu9250->g_rawi[i] = 0;
        mpu9250->m_rawi[i] = 0;

        mpu9250->a_raw[i] = 0.f;
        mpu9250->g_raw[i] = 0.f;
        mpu9250->m_raw[i] = 0.f;

        mpu9250->a[i] = 0.f;
        mpu9250->g[i] = 0.f;
        mpu9250->m[i] = 0.f;

        mpu9250->a_filt[i] = 0.f;
        mpu9250->g_filt[i] = 0.f;
        mpu9250->m_filt[i] = 0.f;

        mpu9250->ba[i] = 0.f;
        mpu9250->bg[i] = 0.f;
        mpu9250->bm[i] = 0.f;

        mpu9250->Ka[i][i] = 1.f;
        mpu9250->Kg[i][i] = 1.f;
        mpu9250->Km[i][i] = 1.f;

        for (j=0;j<N_AXES;j++) {
            mpu9250->Ka[i][j] = 0.f;
            mpu9250->Kg[i][j] = 0.f;
            mpu9250->Km[i][j] = 0.f;
        }
    }

    /*************************/
    /*** Test data reading ***/
    /*************************/
    for (i=0;i<N_TRIES;i++) {
        ret = mpu9250_get_acc_gyro_mag_temp(mpu9250);
        if (ret)
            break;
    }
    if (i >= N_TRIES) {
        mpu9250->isImuInit = 0;
        return 0;
    }

    return 1;
}

uint8_t mpu9250_write_byte_mag(uint8_t _reg, uint8_t _data) {
    uint8_t status = 0, n = 0;

    write_byte_spi1(USER_CTRL, 0x30);
    write_byte_spi1(I2C_MST_CTRL, 0x0D); // Set I2C_MST_CLK at 400 kHz = 13 = 0x0D
    //write_byte_spi1(I2C_MST_DELAY_CTRL, 0x10); // Trigger slave 4 actions at each sample
    write_byte_spi1(I2C_SLV4_ADDR, AK8963_I2C_ADDRESS);
    write_byte_spi1(I2C_SLV4_REG, _reg);
    write_byte_spi1(I2C_SLV4_DO, _data); // Write dummy
    write_byte_spi1(I2C_SLV4_CTRL, 0x80); // I2C_SLV4_EN

    read_byte_spi1(I2C_MST_STATUS, &status);
    while (!(status & 0x40) && n < N_TRIES) {
        HAL_Delay(1);
        read_byte_spi1(I2C_MST_STATUS, &status);
        n++;
    }

    if (status & 0x40)
        return 1;
    else
        return 0;
}

uint8_t mpu9250_read_byte_mag(uint8_t _reg, uint8_t* _data) {
    uint8_t status = 0, n = 0;

    write_byte_spi1(USER_CTRL, 0x30);
    write_byte_spi1(I2C_MST_CTRL, 0x0D); // Set I2C_MST_CLK at 400 kHz = 13 = 0x0D
    //write_byte_spi1(I2C_MST_DELAY_CTRL, 0x10); // Trigger slave 4 actions at each sample
    write_byte_spi1(I2C_SLV4_ADDR, AK8963_I2C_ADDRESS | 0x80);
    write_byte_spi1(I2C_SLV4_REG, _reg);
    write_byte_spi1(I2C_SLV4_DO, 0x00); // Write dummy
    write_byte_spi1(I2C_SLV4_CTRL, 0x80); // I2C_SLV4_EN

    read_byte_spi1(I2C_MST_STATUS, &status);
    while (!(status & 0x40) && n < N_TRIES) {
        HAL_Delay(1);
        read_byte_spi1(I2C_MST_STATUS, &status);
        n++;
    }

    if (status & 0x40) {
        read_byte_spi1(I2C_SLV4_DI, _data);
        return 1;
    }
    else
        return 0;
}

uint8_t mpu9250_init_mag(Mpu9250_t* mpu9250) {
    uint8_t ret = 0;
    uint8_t i = 0;
    uint8_t asa = 0;

    // Check mag Who am i via external sensor registers
    for (i=0;i<N_TRIES;i++) {
        ret = mpu9250_check_mag_devId(mpu9250);
        if (ret)
            break;
    }
    if (i >= N_TRIES) {
        return 0;
    }

    // Get magnetometer sensitivity
    ret = mpu9250_write_byte_mag(CNTL1, 0x1F); // Fuse ROM access mode + 16-bit output
    if (!ret)
        return 0;

    for (i=0;i<N_AXES;i++) {
        ret = mpu9250_read_byte_mag(ASAX+i, &asa);
        if (!ret)
            return 0;
        // cf. mag datasheet p. 32
        mpu9250->m_gain[i] = (float)((((asa-128.0f)/256.0f)+1.0f)*MAG_SENS);
    }

    ret = mpu9250_write_byte_mag(CNTL1, 0x00); // Power down mode
    if (!ret)
        return 0;

    // Configure mag sample rate: AKM measurement mode
    ret = mpu9250_write_byte_mag(CNTL1, 0x16); // 16 bits mode + 100 Hz (Continuous mode 2)
    if (!ret)
        return 0;

    // Configure read measurements
    // Slave 0 reads from AKM data registers
    write_byte_spi1(I2C_SLV0_ADDR, AK8963_I2C_ADDRESS | 0x80);
    // Compass reads start at this register
    write_byte_spi1(I2C_SLV0_REG, ST1);
    // set I2C_MST_DLY: slaves will only be enabled every (1+I2C_MST_DLY) samples
    write_byte_spi1(I2C_SLV4_CTRL, 0x04);
    // Enable slave 0, 8-byte reads
    write_byte_spi1(I2C_SLV0_CTRL, 0x88); // I2C_SLV0_EN, 8-byte read
    // Trigger slave 0 and slave 1 actions at each sample
    write_byte_spi1(I2C_MST_DELAY_CTRL, 0x01); // Trigger slave 0 actions at each sample

    return 1;
}

uint8_t mpu9250_get_acc_gyro_mag_temp(Mpu9250_t* mpu9250) {
    uint8_t ret = 0;

    if (!mpu9250->isImuInit)
        return 0;

    ret = read_bytes_spi1(ACCEL_XOUT_H, mpu9250->register_data, REG_DATA_SIZE);
    if (ret) {
        // Get accelerometer
        mpu9250->a_rawi[0] = (((int16_t)mpu9250->register_data[0]) << 8) | mpu9250->register_data[1];
        mpu9250->a_rawi[1] = (((int16_t)mpu9250->register_data[2]) << 8) | mpu9250->register_data[3];
        mpu9250->a_rawi[2] = (((int16_t)mpu9250->register_data[4]) << 8) | mpu9250->register_data[5];

        mpu9250->a_raw[0] = (float)mpu9250->a_rawi[0] * mpu9250->a_gain;
        mpu9250->a_raw[1] = (float)mpu9250->a_rawi[1] * mpu9250->a_gain;
        mpu9250->a_raw[2] = (float)mpu9250->a_rawi[2] * mpu9250->a_gain;

        // Get temperature
        // (TEMP_degC = ((TEMP_OUT - RoomTempOffset)/Temp_Sensitivity)+21degC)
        //mpu9250->temperature = (float)((((int16_t)(((int16_t)data[6] << 8) |
        //      data[7]))-ROOM_TEMP_OFFSET)/TEMP_SENSITIVITY+21.0f);

        // Get gyroscope
        mpu9250->g_rawi[0] = (((int16_t)mpu9250->register_data[8])  << 8) | mpu9250->register_data[9];
        mpu9250->g_rawi[1] = (((int16_t)mpu9250->register_data[10]) << 8) | mpu9250->register_data[11];
        mpu9250->g_rawi[2] = (((int16_t)mpu9250->register_data[12]) << 8) | mpu9250->register_data[13];

        mpu9250->g_raw[0] = (float)mpu9250->g_rawi[0] * mpu9250->g_gain;
        mpu9250->g_raw[1] = (float)mpu9250->g_rawi[1] * mpu9250->g_gain;
        mpu9250->g_raw[2] = (float)mpu9250->g_rawi[2] * mpu9250->g_gain;

        // Convert from degree/s to rad/s
        //mpu9250->g_raw[0] *= deg_to_rad;
        //mpu9250->g_raw[1] *= deg_to_rad;
        //mpu9250->g_raw[2] *= deg_to_rad;

        mpu9250->ag_error_count = 0;

        // Get magnetometer
        // Check if data ready and if mag data not overflow
        if (!(mpu9250->register_data[14] & 0x01) || (mpu9250->register_data[21] & 0x08)) {
            mpu9250->m_error_count++;
            if (mpu9250->m_error_count > MAX_ERRORS) {
                mpu9250->m_error_count = 0;
                mpu9250->isMagInit = 0;
            }
        }
        else {
            mpu9250->m_rawi[0] = (((int16_t)mpu9250->register_data[16]) << 8) | mpu9250->register_data[15];
            mpu9250->m_rawi[1] = (((int16_t)mpu9250->register_data[18]) << 8) | mpu9250->register_data[17];
            mpu9250->m_rawi[2] = (((int16_t)mpu9250->register_data[20]) << 8) | mpu9250->register_data[19];

            // my = ax, mx = ay, mz = -az
            mpu9250->m_raw[1] =  (float)mpu9250->m_rawi[0] * mpu9250->m_gain[0];
            mpu9250->m_raw[0] =  (float)mpu9250->m_rawi[1] * mpu9250->m_gain[1];
            mpu9250->m_raw[2] = -(float)mpu9250->m_rawi[2] * mpu9250->m_gain[2];

            mpu9250->m_error_count = 0;
        }
    }
    else {
        mpu9250->ag_error_count++;
        mpu9250->m_error_count++;
        if (mpu9250->ag_error_count > MAX_ERRORS || mpu9250->m_error_count > MAX_ERRORS) {
            mpu9250->ag_error_count = 0;
            mpu9250->m_error_count = 0;
            mpu9250->isImuInit = 0;
            mpu9250->isMagInit = 0;
        }
        ret = 0;
    }

    return 1;
}

uint8_t mpu9250_close(Mpu9250_t* mpu9250) {
    mpu9250->isImuInit = 0;
    mpu9250->isMagInit = 0;
    return 1;
}

// From https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
uint8_t mpu9250_static_calibration(Mpu9250_t* mpu9250) {
    uint8_t ret = 0;
    float delta = 0.f;
    float delta2 = 0.f;
    float count = 0.f;
    uint32_t i = 0;
    uint8_t j = 0;
    uint32_t nSamples = (uint32_t)mpu9250->calib_time * (uint32_t)mpu9250->sample_frequency;

    mpu9250->isCalibrated = 0;

    if (!mpu9250->isImuInit && !mpu9250->isMagInit)
        return 0;

    // Re-init biases and variances
    for (j=0;j<N_AXES;j++) {
        mpu9250->va[j] = mpu9250->vg[j] = mpu9250->vm[j] = 0.f;
        mpu9250->ba[j] = mpu9250->bg[j] = mpu9250->bm[j] = 0.f;
    }

    for (i=0;i<nSamples;i++) {
        // Get values
        ret = mpu9250_get_acc_gyro_mag_temp(mpu9250);
        if (ret) {
            count += 1.f;
            // 1g suppression on the z axis of the accelerometer
            mpu9250->a_raw[2] -= 1.0f;
            // Compute mean and variance online
            for (j=0;j<N_AXES;j++) {
                delta = mpu9250->a_raw[j] - mpu9250->ba[j]; // d = x - m
                mpu9250->ba[j] += delta / count; // m += d/n
                delta2 = mpu9250->a_raw[j] - mpu9250->ba[j]; // d = x - m
                mpu9250->va[j] += delta * delta2; // M += d*(x - m)

                delta = mpu9250->g_raw[j] - mpu9250->bg[j];
                mpu9250->bg[j] += delta / count;
                delta2 = mpu9250->g_raw[j] - mpu9250->bg[j];
                mpu9250->vg[j] +=  delta * delta2;

                delta = mpu9250->m_raw[j] - mpu9250->bm[j];
                mpu9250->bm[j] += delta / count;
                delta2 = mpu9250->m_raw[j] - mpu9250->bm[j];
                mpu9250->vm[j] += delta * delta2;
            }
            // Delay of sample_period
            HAL_Delay(mpu9250->sample_period);
        }
    }
    count -= 1.f;

    if (count > 0.f) {
        for (j=0;j<N_AXES;j++) {
            mpu9250->va[j] /= count;
            mpu9250->vg[j] /= count;
            mpu9250->vm[j] /= count;
        }
        mpu9250->isCalibrated = 1;
        return 1;
    }
    else {
        return 0;
    }
}

void mpu9250_get_azimuth(Mpu9250_t* mpu9250) {
    if (!mpu9250->isMagInit) {
        mpu9250->mag_azimuth = -10000.0f;
        return;
    }
    if (mpu9250->m[0] == 0) {
        if (mpu9250->m[1] < 0)
            mpu9250->mag_azimuth = .5f * PI;
        if (mpu9250->m[1] > 0)
            mpu9250->mag_azimuth = 3.f * PI *.5f;
    }
    else if (mpu9250->m[0] < 0) {
        mpu9250->mag_azimuth = PI - (float)atan2(mpu9250->m[1], mpu9250->m[0]);
    }
    else {
        if (mpu9250->m[1] < 0) {
            mpu9250->mag_azimuth = -(float)atan2(mpu9250->m[1], mpu9250->m[0]);
        }
        if (mpu9250->m[1] > 0) {
            mpu9250->mag_azimuth = (2.f*PI) - (float)atan2(mpu9250->m[1], mpu9250->m[0]);
        }
    }
}

void mpu9250_low_pass_filter(Mpu9250_t* mpu9250) {
    float alpha = 0.f;
    uint8_t i = 0;

    // Acc. filtering
    if (mpu9250->isImuInit && mpu9250->a_rc > 0.0f && mpu9250->dt > 0.0f) {
        alpha = (mpu9250->dt / (mpu9250->dt + mpu9250->a_rc));
        alpha = (alpha < 0.0f) ? 0.0f:alpha;
        alpha = (alpha > 1.0f) ? 1.0f:alpha;
        for (i=0;i<N_AXES;i++)
            mpu9250->a[i] += (mpu9250->a_raw[i] - mpu9250->a[i]) * alpha;
    }

    // Gyro. filtering
    if (mpu9250->isImuInit && mpu9250->g_rc > 0.0f && mpu9250->dt > 0.0f) {
        alpha = (mpu9250->dt / (mpu9250->dt + mpu9250->g_rc));
        alpha = (alpha < 0.0f) ? 0.0f:alpha;
        alpha = (alpha > 1.0f) ? 1.0f:alpha;
        for (i=0;i<N_AXES;i++)
            mpu9250->g[i] += (mpu9250->g_raw[i] - mpu9250->g[i]) * alpha;
    }

    // Mag. filtering
    if (mpu9250->isMagInit && mpu9250->m_rc > 0.0f && mpu9250->dt > 0.0f) {
        alpha = (mpu9250->dt / (mpu9250->dt + mpu9250->m_rc));
        alpha = (alpha < 0.0f) ? 0.0f:alpha;
        alpha = (alpha > 1.0f) ? 1.0f:alpha;
        for (i=0;i<N_AXES;i++)
            mpu9250->m[i] += (mpu9250->m_raw[i] - mpu9250->m[i]) * alpha;
    }
}

void mpu9250_correct(Mpu9250_t* mpu9250) {
    // Correct only for bias
    mpu9250->a[0] = mpu9250->a_raw[0] - mpu9250->ba[0];
    mpu9250->a[1] = mpu9250->a_raw[1] - mpu9250->ba[1];
    mpu9250->a[2] = mpu9250->a_raw[2] - mpu9250->ba[2];

    mpu9250->g[0] = mpu9250->g_raw[0] - mpu9250->bg[0];
    mpu9250->g[1] = mpu9250->g_raw[1] - mpu9250->bg[1];
    mpu9250->g[2] = mpu9250->g_raw[2] - mpu9250->bg[2];

    mpu9250->m[0] = mpu9250->m_raw[0] - mpu9250->bm[0];
    mpu9250->m[1] = mpu9250->m_raw[1] - mpu9250->bm[1];
    mpu9250->m[2] = mpu9250->m_raw[2] - mpu9250->bm[2];

    // Correct for bias and scaling
    /*
    float a_tmp[N_AXES] = {mpu9250->a_raw[0] - mpu9250->ba[0],
                            mpu9250->a_raw[1] - mpu9250->ba[1],
                            mpu9250->a_raw[2] - mpu9250->ba[2]};
    float g_tmp[N_AXES] = {mpu9250->g_raw[0] - mpu9250->bg[0],
                            mpu9250->g_raw[1] - mpu9250->bg[1],
                            mpu9250->g_raw[2] - mpu9250->bg[2]};
    float g_tmp[N_AXES] = {mpu9250->m_raw[0] - mpu9250->bm[0],
                            mpu9250->m_raw[1] - mpu9250->bm[1],
                            mpu9250->m_raw[2] - mpu9250->bm[2]};

    mpu9250->a[0] = mpu9250->Ka[0][0]*mpu9250->a_tmp[0] +
                    mpu9250->Ka[0][1]*mpu9250->a_tmp[1] +
                    mpu9250->Ka[0][2]*mpu9250->a_tmp[2];
    mpu9250->a[1] = mpu9250->Ka[1][0]*mpu9250->a_tmp[0] +
                    mpu9250->Ka[1][1]*mpu9250->a_tmp[1] +
                    mpu9250->Ka[1][2]*mpu9250->a_tmp[2];
    mpu9250->a[2] = mpu9250->Ka[2][0]*mpu9250->a_tmp[0] +
                    mpu9250->Ka[2][1]*mpu9250->a_tmp[1] +
                    mpu9250->Ka[2][2]*mpu9250->a_tmp[2];

    mpu9250->g[0] = mpu9250->Kg[0][0]*mpu9250->g_tmp[0] +
                    mpu9250->Kg[0][1]*mpu9250->g_tmp[1] +
                    mpu9250->Kg[0][2]*mpu9250->g_tmp[2];
    mpu9250->g[1] = mpu9250->Kg[1][0]*mpu9250->g_tmp[0] +
                    mpu9250->Kg[1][1]*mpu9250->g_tmp[1] +
                    mpu9250->Kg[1][2]*mpu9250->g_tmp[2];
    mpu9250->g[2] = mpu9250->Kg[2][0]*mpu9250->g_tmp[0] +
                    mpu9250->Kg[2][1]*mpu9250->g_tmp[1] +
                    mpu9250->Kg[2][2]*mpu9250->g_tmp[2];

    mpu9250->m[0] = mpu9250->Km[0][0]*mpu9250->m_tmp[0] +
                    mpu9250->Km[0][1]*mpu9250->m_tmp[1] +
                    mpu9250->Km[0][2]*mpu9250->m_tmp[2];
    mpu9250->m[1] = mpu9250->Km[1][0]*mpu9250->m_tmp[0] +
                    mpu9250->Km[1][1]*mpu9250->m_tmp[1] +
                    mpu9250->Km[1][2]*mpu9250->m_tmp[2];
    mpu9250->m[2] = mpu9250->Km[2][0]*mpu9250->m_tmp[0] +
                    mpu9250->Km[2][1]*mpu9250->m_tmp[1] +
                    mpu9250->Km[2][2]*mpu9250->m_tmp[2];
    */
}

uint8_t mpu9250_set_cut_off_freq(Mpu9250_t* mpu9250, float* cut_off_freqs) {
    if (cut_off_freqs == NULL)
        return 0;

    mpu9250->a_cutoff_freq = cut_off_freqs[0];
    mpu9250->g_cutoff_freq = cut_off_freqs[1];
    mpu9250->m_cutoff_freq = cut_off_freqs[2];

    mpu9250->a_rc = (1.f/(2.f*PI*mpu9250->a_cutoff_freq));
    mpu9250->g_rc = (1.f/(2.f*PI*mpu9250->g_cutoff_freq));
    mpu9250->m_rc = (1.f/(2.f*PI*mpu9250->m_cutoff_freq));

    return 1;
}

void mpu9250_set_filtering(Mpu9250_t* mpu9250, uint8_t value) {
    mpu9250->isFiltered = value;
}

void mpu9250_set_corrected(Mpu9250_t* mpu9250, uint8_t value) {
    mpu9250->isCorrected = value;
}

void mpu9250_set_calib_time(Mpu9250_t* mpu9250, uint32_t time) {
    mpu9250->calib_time = time;
}

uint8_t mpu9250_getIntStatus(void) {
    uint8_t intStatus = 0;
    read_byte_spi1(INT_STATUS, &intStatus);
    return intStatus;
}

uint8_t mpu9250_check_devId(Mpu9250_t* mpu9250) {
    uint8_t whoAmI = 0;

    if (mpu9250 == 0)
        return 0;

    // Look for device ID
    read_byte_spi1(WHO_AM_I, &whoAmI);
    
    // Verify with the default value
    if (whoAmI == WHO_AM_I_ID || whoAmI == WHO_AM_I_ID2) {
        return 1;
    }
    else {
        return 0;
    }
}

uint8_t mpu9250_check_mag_devId(Mpu9250_t* mpu9250) {
    uint8_t whoAmI = 0;

    if (mpu9250 == 0)
        return 0;

    // Look for device ID
    mpu9250_read_byte_mag(WIA, &whoAmI);

    // Verify with the default value
    if (whoAmI > 0x40 && whoAmI < 0x80) {
        return 1;
    }
    else {
        return 0;
    }
}

void mpu9250_set_smplrtDiv(Mpu9250_t* mpu9250, uint8_t _div) {
    float freq = 0.;
    
    write_byte_spi1(SMPLRT_DIV, _div);
    
    freq = roundf(1000.f / (1.f + _div));
    mpu9250->sample_frequency = (uint16_t)round(freq); // [Hz]
    mpu9250->sample_period = (uint16_t)round(1000.f/freq); // [ms]
    mpu9250->dt = 1.f/freq; // [s]
}

void mpu9250_set_smplrtFreq(Mpu9250_t* mpu9250, uint16_t _freq) {
    uint8_t div = 0;
    float freq = 0;

    if (_freq > 1000) {
        return;
    }

    div = (uint8_t)(1000/_freq - 1);
    write_byte_spi1(SMPLRT_DIV, div);

    freq = 1000.f / (1.f + div);
    mpu9250->sample_frequency = (uint16_t)round(freq); // [Hz]
    mpu9250->sample_period = (uint16_t)round(1000.f/freq); // [ms]
    mpu9250->dt = 1.f/freq; // [s]
}

void mpu9250_set_acc_g_per_lsb(Mpu9250_t* mpu9250, uint8_t range) {
    uint8_t data = 0;

    read_byte_spi1(ACCEL_CONFIG, &data);
    if (range == ACCEL_FS_2) {
        data &= ~(1 << 3);
        data &= ~(1 << 4);
        mpu9250->a_gain = G_PER_LSB_2;
    }
    else if (range == ACCEL_FS_4) {
        data |= (1 << 3);
        data &= ~(1 << 4);
        mpu9250->a_gain = G_PER_LSB_4;
    }
    else if (range == ACCEL_FS_8) {
        data &= ~(1 << 3);
        data |= (1 << 4);
        mpu9250->a_gain = G_PER_LSB_8;
    }
    else if (range == ACCEL_FS_16) {
        data |= (1 << 3);
        data |= (1 << 4);
        mpu9250->a_gain = G_PER_LSB_16;
    }
    else {
        //fprintf(stderr, "mpu9250_set_acc_g_per_lsb: error\n");
    }
    write_byte_spi1(ACCEL_CONFIG, data);
}

void mpu9250_set_gyro_deg_per_lsb(Mpu9250_t* mpu9250, uint8_t range) {
    uint8_t data = 0;
    read_byte_spi1(GYRO_CONFIG, &data);
    if (range == GYRO_FS_250) {
        data &= ~(1 << 3);
        data &= ~(1 << 4);
        mpu9250->g_gain = DEG_PER_LSB_250;
    }
    else if (range == GYRO_FS_500) {
        data |= (1 << 3);
        data &= ~(1 << 4);
        mpu9250->g_gain = DEG_PER_LSB_500;
    }
    else if (range == GYRO_FS_1000) {
        data &= ~(1 << 3);
        data |= (1 << 4);
        mpu9250->g_gain = DEG_PER_LSB_1000;
    }
    else if (range == GYRO_FS_2000) {
        data |= (1 << 3);
        data |= (1 << 4);
        mpu9250->g_gain = DEG_PER_LSB_2000;
    }
    else {
        //fprintf(stderr, "mpu9250_set_gyro_deg_per_lsb: error\n");
    }
    write_byte_spi1(GYRO_CONFIG, data);
}

void mpu9250_reset(Mpu9250_t* mpu9250) {
    if (mpu9250 == 0)
        return;
    write_byte_spi1(PWR_MGMT_1, 0x80);
}