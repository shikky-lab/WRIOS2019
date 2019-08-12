/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MPU9250Operator.h
 * Author: mfukunaga
 *
 * Created on 2019/07/28, 0:15
 */

#ifndef MPU9250OPERATOR_H
#define MPU9250OPERATOR_H

#include "MPU9250.h"
#include <stdint.h>

class MPU9250Operator {
public:
    MPU9250Operator(const int _piId,const int _spi_handle);
    int writeBytes(uint8_t firstAddr,int cnt,uint8_t *rawVals);
    int readBytes(uint8_t firstAddr,int cnt,uint8_t *rawVals);
    int readGyroData(int16_t *destination);
    int readAccelData(int16_t *destination);
    int writeByte(uint8_t Addr,uint8_t Val);
    uint8_t readByte(uint8_t addr);
    void initMPU9250();
    void calibrateMPU9250(float * gyroBias, float * accelBias);
    void calcGres();
    void calcAres();
    float aRes,gRes;
    
        // Set initial input parameters
    enum Ascale
    {
      AFS_2G = 0,
      AFS_4G,
      AFS_8G,
      AFS_16G
    };

    enum Gscale {
      GFS_250DPS = 0,
      GFS_500DPS,
      GFS_1000DPS,
      GFS_2000DPS
    };

    enum Mscale {
      MFS_14BITS = 0, // 0.6 mG per LSB
      MFS_16BITS      // 0.15 mG per LSB
    };

    enum M_MODE {
      M_8HZ = 0x02,  // 8 Hz update
      M_100HZ = 0x06 // 100 Hz continuous magnetometer
    };

    
    // TODO: Add setter methods for this hard coded stuff
    // Specify sensor full scale
    uint8_t Gscale = GFS_250DPS;
    uint8_t Ascale = AFS_2G;
    // Choose either 14-bit or 16-bit magnetometer resolution
    uint8_t Mscale = MFS_16BITS;

    // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    uint8_t Mmode = M_8HZ;
private:
    const int piId;
    const int spi_handle;
    const uint8_t DUMMYADDR = 117;//who am i

};

#endif /* MPU9250OPERATOR_H */

