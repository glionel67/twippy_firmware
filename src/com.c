/**
 * \file com.c
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief Communication functions to exchange data with other device
 */

#include "comm.h"
#include "uart.h"
#include "odometry.h"

Inputs_t inputs;

int decode_msg(uint8_t* buf, uint8_t len)
{
    uint8_t data[DEBUGSIZE] = { 0, };

    if (buf[0] == MSG_HEADER1 && buf[1] == MSG_HEADER2)
    {
        switch (buf[2])
        {
        case 0x00: {
            uint8_t str[] = "Unknown msg";
            uart2_write(str, sizeof(str));
            break;
        }
        case EMERGENCY_STOP: {
            inputs.emergencyStop = 1;
            sprintf((char*) data, (const char*) "cmd stop\r\n");
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_MOTORS_REF_RPM: {
            int16_t rpm1 = ((int16_t)(((int16_t)buf[3] << 8) | buf[4]));
            int16_t rpm2 = ((int16_t)(((int16_t)buf[5] << 8) | buf[6]));
            inputs.rpm[0] = rpm1;
            inputs.rpm[1] = rpm2;
            inputs.newRpm[0] = inputs.newRpm[1] = 1;
            sprintf((char*) data, (const char*) "cmd M1=%d,M2=%d [RPM]\r\n", rpm1, rpm2);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_MOTOR_REF_RPM: {
            uint8_t i = buf[3];
            int16_t rpmi = ((int16_t)(((int16_t)buf[4] << 8) | buf[5]));
            inputs.rpm[i] = rpmi;
            inputs.newRpm[i] = 1;
            sprintf((char*) data, (const char*) "cmd M%d=%d [RPM]\r\n", i, rpmi);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_MOTORS_PWM: {
            uint16_t pwm1 = ((uint16_t)(((uint16_t)buf[3] << 8) | buf[4]));
            uint16_t pwm2 = ((uint16_t)(((uint16_t)buf[5] << 8) | buf[6]));
            inputs.pwm[0] = pwm1;
            inputs.pwm[1] = pwm2;
            inputs.newPwm[0] = inputs.newPwm[1] = 1;
            sprintf((char*) data, (const char*) "cmd M1=%d,M2=%d [PWM]\r\n", pwm1, pwm2);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_MOTOR_PWM: {
            uint8_t i = buf[3];
            int16_t pwmi = ((int16_t)(((int16_t)buf[4] << 8) | buf[5]));
            inputs.pwm[i] = pwmi;
            inputs.newPwm[i] = 1;
            sprintf((char*) data, (const char*) "cmd M%d=%d [PWM]\r\n", i, pwmi);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_V_ROBOT: {
            int16_t vi = ((uint16_t)(((uint16_t)buf[3] << 8) | buf[4]));
            float v = ((float)vi)/1000.; // Convert mm/s to m/s
            sprintf((char*) data, (const char*) "cmd v=%f [m/s]\r\n", v);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_W_ROBOT: {
            int16_t wi = ((uint16_t)(((uint16_t)buf[3] << 8) | buf[4]));
            float w = ((float)wi)/1000.; // Convert mrad/s to rad/s
            sprintf((char*) data, (const char*) "cmd w=%f [rad/s]\r\n", w);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_REF_V_ROBOT: {
            int16_t vi = ((uint16_t)(((uint16_t)buf[3] << 8) | buf[4]));
            float v = ((float)vi)/1000.; // Convert mm/s to m/s
            robotSpeedsToWheelSpeeds(v, 0.f, &inputs.rpm[0], &inputs.rpm[1]);
            inputs.newRpm[0] = inputs.newRpm[1] = 1;
            sprintf((char*) data, (const char*) "cmd v_ref=%f [m/s]\r\n", v);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_REF_W_ROBOT: {
            int16_t wi = ((uint16_t)(((uint16_t)buf[3] << 8) | buf[4]));
            float w = ((float)wi)/1000.; // Convert mrad/s to rad/s
            robotSpeedsToWheelSpeeds(0.f, w, &inputs.rpm[0], &inputs.rpm[1]);
            inputs.newRpm[0] = inputs.newRpm[1] = 1;
            sprintf((char*) data, (const char*) "cmd w_ref=%f [rad/s]\r\n", w);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_ROBOT_SPEEDS: {
            int16_t vi = ((int16_t)(((int16_t)buf[3] << 8) | buf[4]));
            int16_t wi = ((int16_t)(((int16_t)buf[5] << 8) | buf[6]));
            float v = ((float)vi)/1000.; // Convert mm/s to m/s
            float w = ((float)wi)/1000.; // Convert mrad/s to rad/s
            sprintf((char*) data, (const char*) "cmd v=%f [m/s],w=%f [rad/s]\r\n", v, w);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_REF_ROBOT_SPEEDS: {
            int16_t vi = ((int16_t)(((int16_t)buf[3] << 8) | buf[4]));
            int16_t wi = ((int16_t)(((int16_t)buf[5] << 8) | buf[6]));
            float v = ((float)vi)/1000.; // Convert mm/s to m/s
            float w = ((float)wi)/1000.; // Convert mrad/s to rad/s
            robotSpeedsToWheelSpeeds(v, w, &inputs.rpm[0], &inputs.rpm[1]);
            inputs.newRpm[0] = inputs.newRpm[1] = 1;
            sprintf((char*) data, (const char*) "cmd v_ref=%f [m/s],w_ref=%f [rad/s]\r\n", v, w);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_ROBOT_POSE: {
            int16_t xi = ((int16_t)(((int16_t)buf[3] << 8) | buf[4]));
            int16_t yi = ((int16_t)(((int16_t)buf[5] << 8) | buf[6]));
            int16_t ti = ((int16_t)(((int16_t)buf[7] << 8) | buf[8]));
            float x = ((float)xi)/100.f; // Convert cm to m
            float y = ((float)yi)/100.f; // Convert cm to m
            float t = ((float)ti)/1000.f; // Convert mrad to rad
            sprintf((char*) data, (const char*) "cmd x=%f m,y=%f m,t=%f rad\r\n", x, y, t);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_REF_ROBOT_POSE: {
            int16_t xi = ((int16_t)(((int16_t)buf[3] << 8) | buf[4]));
            int16_t yi = ((int16_t)(((int16_t)buf[5] << 8) | buf[6]));
            int16_t ti = ((int16_t)(((int16_t)buf[7] << 8) | buf[8]));
            float x = ((float)xi)/100.f; // Convert cm to m
            float y = ((float)yi)/100.f; // Convert cm to m
            float t = ((float)ti)/1000.f; // Convert mrad to rad
            inputs.pose[0] = x;
            inputs.pose[1] = y;
            inputs.pose[2] = t;
            inputs.newPose = 1;
            sprintf((char*) data, (const char*) "cmd x_ref=%f m,y_ref=%f m,t_ref=%f rad\r\n", x, y, t);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_SERVO_ANGLE: {
            uint8_t i = buf[3];
            int16_t ai = ((int16_t)(((int16_t)buf[4] << 8) | buf[5]));
            inputs.servo[i] = ai;
            inputs.newServo[i] = 1;
            sprintf((char*) data, (const char*) "cmd A%d=%d [degrees]\r\n", i, ai);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_SERVOS_ANGLE: {
            int16_t a1 = ((int16_t)(((int16_t)buf[3] << 8) | buf[4]));
            int16_t a2 = ((int16_t)(((int16_t)buf[5] << 8) | buf[6]));
            int16_t a3 = ((int16_t)(((int16_t)buf[7] << 8) | buf[8]));
            inputs.servo[0] = a1;
            inputs.servo[1] = a2;
            inputs.servo[2] = a3;
            inputs.newServo[0] = inputs.newServo[1] = inputs.newServo[2] = 1;
            sprintf((char*) data, (const char*) "cmd A1=%d,A2=%d,A2=%d [degrees]\r\n", a1, a2, a3);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_MOTORS_PID_P: {
            uint32_t tmp = ((buf[3] << 24) | (buf[4] << 16) | (buf[5] <<  8) | buf[6]);
            float p = *((float *) tmp); //float p = *(float *)&tmp;
            sprintf((char*) data, (const char*) "cmd set P=%f\r\n", p);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_MOTORS_PID_I: {
            uint32_t tmp = ((buf[3] << 24) | (buf[4] << 16) | (buf[5] <<  8) | buf[6]);
            float i = *((float *) tmp); //float i = *(float *)&tmp;
            sprintf((char*) data, (const char*) "cmd set I=%f\r\n", i);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_MOTORS_PID_D: {
            uint32_t tmp = ((buf[3] << 24) | (buf[4] << 16) | (buf[5] <<  8) | buf[6]);
            float d = *((float *) tmp); //float d = *(float *)&tmp;
            sprintf((char*) data, (const char*) "cmd set D=%f\r\n", d);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_WHEEL_RADIUS: {
            uint16_t ri = ((int16_t)(((int16_t)buf[3] << 8) | buf[4]));
            float r = ((float)ri)/1000.f; // Convert mm to m
            set_wheel_radius(r);
            sprintf((char*) data, (const char*) "cmd set r=%f\r\n", r);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_WHEEL_BASE: {
            uint16_t bi = ((int16_t)(((int16_t)buf[3] << 8) | buf[4]));
            float b = ((float)bi)/1000.f; // Convert mm to m
            set_wheel_base(b);
            sprintf((char*) data, (const char*) "cmd set b=%f\r\n", b);
            uart2_write(data, sizeof(data));
            break;
        }
        case SET_PWM_RPM_MODE: {
            inputs.pwmRpmMode = (uint8_t)buf[3];
            sprintf((char*) data, (const char*) "cmd set pwmRpmMode=%d\r\n", inputs.pwmRpmMode);
            uart2_write(data, sizeof(data));
            break;
        }
        default:
            return -1;
        }
    }
    else
    {
        return -1;
    }
    return 0;
}

int send_info(DataPacket_t* dataPacket, uint8_t output)
{
    uint8_t debug[DEBUGSIZE] = { 0, };
    int ret = 0;

    debug[0] = '#';

    debug[1] = dataPacket->imot[0] & 0x00FF;
    debug[2] = (dataPacket->imot[0] >> 8) & 0x00FF;
    debug[3] = dataPacket->imot[1] & 0x00FF;
    debug[4] = (dataPacket->imot[1] >> 8) & 0x00FF;

    debug[5] = dataPacket->ticks[0] & 0x00FF;
    debug[6] = (dataPacket->ticks[0] >> 8) & 0x00FF;
    debug[7] = dataPacket->ticks[1] & 0x00FF;
    debug[8] = (dataPacket->ticks[1] >> 8) & 0x00FF;

    debug[9] = dataPacket->wref[0] & 0x00FF;
    debug[10] = (dataPacket->wref[0] >> 8) & 0x00FF;
    debug[11] = dataPacket->wref[1] & 0x00FF;
    debug[12] = (dataPacket->wref[1] >> 8) & 0x00FF;

    debug[13] = dataPacket->wmes[0] & 0x00FF;
    debug[14] = (dataPacket->wmes[0] >> 8) & 0x00FF;
    debug[15] = dataPacket->wmes[1] & 0x00FF;
    debug[16] = (dataPacket->wmes[1] >> 8) & 0x00FF;

    debug[17] = dataPacket->vbat & 0x00FF;
    debug[18] = (dataPacket->vbat >> 8) & 0x00FF;

    debug[19] = dataPacket->ibat & 0x00FF;
    debug[20] = (dataPacket->ibat >> 8) & 0x00FF;

    debug[21] = dataPacket->acc[0] & 0x00FF;
    debug[22] = (dataPacket->acc[0] >> 8) & 0x00FF;
    debug[23] = dataPacket->acc[1] & 0x00FF;
    debug[24] = (dataPacket->acc[1] >> 8) & 0x00FF;
    debug[25] = dataPacket->acc[2] & 0x00FF;
    debug[26] = (dataPacket->acc[2] >> 8) & 0x00FF;

    debug[27] = dataPacket->gyro[0] & 0x00FF;
    debug[28] = (dataPacket->gyro[0] >> 8) & 0x00FF;
    debug[29] = dataPacket->gyro[1] & 0x00FF;
    debug[30] = (dataPacket->gyro[1] >> 8) & 0x00FF;
    debug[31] = dataPacket->gyro[2] & 0x00FF;
    debug[32] = (dataPacket->gyro[2] >> 8) & 0x00FF;

    debug[33] = dataPacket->mag[0] & 0x00FF;
    debug[34] = (dataPacket->mag[0] >> 8) & 0x00FF;
    debug[35] = dataPacket->mag[1] & 0x00FF;
    debug[36] = (dataPacket->mag[1] >> 8) & 0x00FF;
    debug[37] = dataPacket->mag[2] & 0x00FF;
    debug[38] = (dataPacket->mag[2] >> 8) & 0x00FF;

    debug[39] = dataPacket->ir[0] & 0x00FF;
    debug[40] = (dataPacket->ir[0] >> 8) & 0x00FF;
    debug[41] = dataPacket->ir[1] & 0x00FF;
    debug[42] = (dataPacket->ir[1] >> 8) & 0x00FF;
    debug[43] = dataPacket->ir[2] & 0x00FF;
    debug[44] = (dataPacket->ir[2] >> 8) & 0x00FF;
    debug[45] = dataPacket->ir[3] & 0x00FF;
    debug[46] = (dataPacket->ir[3] >> 8) & 0x00FF;

    debug[47] = dataPacket->us[0] & 0x00FF;
    debug[48] = (dataPacket->us[0] >> 8) & 0x00FF;
    debug[49] = dataPacket->us[1] & 0x00FF;
    debug[50] = (dataPacket->us[1] >> 8) & 0x00FF;
    debug[51] = dataPacket->us[2] & 0x00FF;
    debug[52] = (dataPacket->us[2] >> 8) & 0x00FF;
    debug[53] = dataPacket->us[3] & 0x00FF;
    debug[54] = (dataPacket->us[3] >> 8) & 0x00FF;

    debug[55] = dataPacket->servo[0] & 0x00FF;
    debug[56] = (dataPacket->servo[0] >> 8) & 0x00FF;
    debug[57] = dataPacket->servo[1] & 0x00FF;
    debug[58] = (dataPacket->servo[1] >> 8) & 0x00FF;
    debug[59] = dataPacket->servo[2] & 0x00FF;
    debug[60] = (dataPacket->servo[2] >> 8) & 0x00FF;

    debug[61] = '\r';
    debug[62] = '\n';

    // Send debug over UART1
    if (output & 0x01)
        ret = uart1_write_it(debug, DEBUGSIZE);
    // Send debug over UART2
    if (output & 0x02)
        ret = uart2_write_it(debug, DEBUGSIZE);

    return ret;
} // send_info

int send_packet(DataPacket_u* packet, uint8_t output)
{
    int ret = 0;
    // Send debug over UART1
    if (output & 0x01)
        ret = uart1_write_it(packet->bytes, PACKET_SIZE);
    // Send debug over UART2
    if (output & 0x02)
        ret = uart2_write_it(packet->bytes, PACKET_SIZE);

    return ret;
} // send_packet