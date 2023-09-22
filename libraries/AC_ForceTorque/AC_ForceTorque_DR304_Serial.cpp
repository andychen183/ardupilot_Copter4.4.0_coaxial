#include "AC_ForceTorque_DR304_Serial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define DR304_FRAME_HEADER1 0x01    // Header1 Byte from DR304_Serial
#define DR304_FRAME_HEADER2 0x03    // Header2 Byte from DR304_Serial
#define DR304_FRAME_LENGTH 29
#define DR304_DATA_LENGTH 0x18     // length of Data for Byte of DR304_Serial

#define FORCETORQUE_FORCE_MAX_N 50
#define FORCETORQUE_TORQUE_MAX_NM 2 

// format of serial packets received from forceTorque sensor D.R304
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x01
// byte 1               Frame header    0x03
// byte 2               DATA_LENGTH     force and torque totle length of byte, default is 0x18
// byte 3               Fx_d3           force in x axis raw data 3 high 8 bits
// byte 4               Fx_d2           force in x axis raw data 2 high 8 bits
// byte 5               Fx_d1           force in x axis raw data 1 low 8 bits
// bute 6               Fx_d0           force in x axis raw data 0 low 8 bits
// byte 7               Fy_d3           force in y axis raw data 3 high 8 bits
// bute 8               Fy_d2           force in y axis raw data 2 high 8 bits
// byte 9               Fy_d1           force in y axis raw data 1 low 8 bits
// byte 10              Fy_d0           force in y axis raw data 0 low 8 bits
// byte 11              Fz_d3           force in z axis raw data 3 high 8 bits
// byte 12              Fz_d2           force in z axis raw data 2 high 8 bits
// byte 13              Fz_d1           force in z axis raw data 1 low 8 bits
// byte 14              Fz_d0           force in z axis raw data 0 low 8 bits
// byte 15              Tx_d3           torque in x axis raw data 3 high 8 bits
// byte 16              Tx_d2           torque in x axis raw data 2 high 8 bits
// byte 17              Tx_d1           torque in x axis raw data 1 low 8 bits
// bute 18              Tx_d0           torque in x axis raw data 0 low 8 bits
// byte 19              Ty_d3           torque in y axis raw data 3 high 8 bits
// bute 20              Ty_d2           torque in y axis raw data 2 high 8 bits
// byte 21              Ty_d1           torque in y axis raw data 1 low 8 bits
// byte 22              Ty_d0           torque in y axis raw data 0 low 8 bits
// byte 23              Tz_d3           torque in z axis raw data 3 high 8 bits
// byte 24              Tz_d2           torque in z axis raw data 2 high 8 bits
// byte 25              Tz_d1           torque in z axis raw data 1 low 8 bits
// byte 26              Tz_d0           torque in z axis raw data 0 low 8 bits
// byte 27              Checksum       high 8 bits of Checksum byte, sum of bytes 0 to bytes 26
// byte 28              Checksum       low  8 bits of Checksum byte, sum of bytes 0 to bytes 26


// read - return last value measured by sensor
bool AC_ForceTorque_DR304_Serial::get_reading(Vector3f &reading_force_N, Vector3f &reading_torque_Nm)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_fx_N = 0;
    float sum_fy_N = 0;
    float sum_fz_N = 0;
    float sum_Tx_Nm = 0;
    float sum_Ty_Nm = 0;
    float sum_Tz_Nm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_positive_range = 0;
    uint16_t count_out_of_negtive_range = 0;

    // read any available lines from the inclination
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }

        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0x01, add to buffer
        if (linebuf_len == 0) {
            if (c == DR304_FRAME_HEADER1) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x03, add it to buffer
            // if not clear the buffer
            if (c == DR304_FRAME_HEADER2) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
             // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 18 items try to decode it
            if (linebuf_len == DR304_FRAME_LENGTH) {
                // calculate checksum
                uint16_t crc = (linebuf[DR304_FRAME_LENGTH-1]<<8) | linebuf[DR304_FRAME_LENGTH-2];
                if (crc == calc_crc_modbus(linebuf, DR304_FRAME_LENGTH-2)) {
                    // calculate Fx raw data
                    int32_t fx_raw = ((uint32_t)linebuf[3] << 24) | ((uint32_t)linebuf[4] << 16) | ((uint16_t)linebuf[5] << 8) | linebuf[6];
                    // calculate Fy raw data
                    int32_t fy_raw = ((uint32_t)linebuf[7] << 24) | ((uint32_t)linebuf[8] << 16) | ((uint16_t)linebuf[9] << 8) | linebuf[10];
                    // calculate Fz raw data
                    int32_t fz_raw = ((uint32_t)linebuf[11] << 24) | ((uint32_t)linebuf[12] << 16) | ((uint16_t)linebuf[13] << 8) | linebuf[14];
                    // calculate Tx raw data
                    int32_t Tx_raw = ((uint32_t)linebuf[15] << 24) | ((uint32_t)linebuf[16] << 16) | ((uint16_t)linebuf[17] << 8) | linebuf[18];
                    // calculate Ty raw data
                    int32_t Ty_raw = ((uint32_t)linebuf[19] << 24) | ((uint32_t)linebuf[20] << 16) | ((uint16_t)linebuf[21] << 8) | linebuf[22];
                    // calculate Tz raw data
                    int32_t Tz_raw = ((uint32_t)linebuf[23] << 24) | ((uint32_t)linebuf[24] << 16) | ((uint16_t)linebuf[25] << 8) | linebuf[26];
                    float fx = (float)(fx_raw*0.001);
                    float fy = (float)(fy_raw*0.001);
                    float fz = (float)(fz_raw*0.001);
                    float Tx = (float)(Tx_raw*0.0001);
                    float Ty = (float)(Ty_raw*0.0001);
                    float Tz = (float)(Tz_raw*0.0001);
                    
                    if (fx > FORCETORQUE_FORCE_MAX_N || fy > FORCETORQUE_FORCE_MAX_N || fz > FORCETORQUE_FORCE_MAX_N || Tx > FORCETORQUE_TORQUE_MAX_NM || Ty > FORCETORQUE_TORQUE_MAX_NM || Tz > FORCETORQUE_TORQUE_MAX_NM) {
                        // this reading is out of positive range
                        count_out_of_positive_range++;
                    } else if((fx < - FORCETORQUE_FORCE_MAX_N) || (fy < - FORCETORQUE_FORCE_MAX_N) || (fz < - FORCETORQUE_FORCE_MAX_N) || (Tx < - FORCETORQUE_TORQUE_MAX_NM) || (Ty < - FORCETORQUE_TORQUE_MAX_NM) || (Tz < - FORCETORQUE_TORQUE_MAX_NM)){
                        // this reading is out of negtive range
                        count_out_of_negtive_range++;
                    } else {
                        // add degree to sum
                        //hal.console->printf("555inclination tilt sensor uart: %f\t, %lu\t,  %lu\r\n", roll, roll_raw, (roll_raw - ROLL_YAW_OFFSET));
                        sum_fx_N += fx;
                        sum_fy_N += fy;
                        sum_fz_N += fz;
                        sum_Tx_Nm += Tx;
                        sum_Ty_Nm += Ty;
                        sum_Tz_Nm += Tz;
                        count++;
                    }
                }                

                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        // return average distance of readings
        reading_force_N.x = sum_fx_N / count;
        reading_force_N.y = sum_fy_N / count;
        reading_force_N.z = sum_fz_N / count;
        reading_torque_Nm.x = sum_Tx_Nm / count;
        reading_torque_Nm.y = sum_Ty_Nm / count;
        reading_torque_Nm.z = sum_Tz_Nm / count;
        return true;
    }

    if (count_out_of_positive_range > 0) {
        // if out of range readings return maximum range for the positive angle
        reading_force_N.x = FORCETORQUE_FORCE_MAX_N;
        reading_force_N.y = FORCETORQUE_FORCE_MAX_N;
        reading_force_N.z = FORCETORQUE_FORCE_MAX_N;
        reading_torque_Nm.x = FORCETORQUE_TORQUE_MAX_NM;
        reading_torque_Nm.y = FORCETORQUE_TORQUE_MAX_NM;
        reading_torque_Nm.z = FORCETORQUE_TORQUE_MAX_NM;
        return true;
    }

    if (count_out_of_negtive_range > 0) {
        // if out of range readings return maximum range for the negtive angle
        reading_force_N.x = -FORCETORQUE_FORCE_MAX_N;
        reading_force_N.y = -FORCETORQUE_FORCE_MAX_N;
        reading_force_N.z = -FORCETORQUE_FORCE_MAX_N;
        reading_torque_Nm.x = -FORCETORQUE_TORQUE_MAX_NM;
        reading_torque_Nm.y = -FORCETORQUE_TORQUE_MAX_NM;
        reading_torque_Nm.z = -FORCETORQUE_TORQUE_MAX_NM;
        return true;
    }

    // no readings so return false
    return false;
}