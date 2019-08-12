/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MPU9250Operator.cpp
 * Author: mfukunaga
 * 
 * Created on 2019/07/28, 0:15
 */

#include "MPU9250Operator.h"
#include <pigpiod_if2.h>
#include "MPU9250.h"
#include <stdio.h>
#define MILLI_SEC (1000000)

void wait_millisecond(int wait_msec)
{
    int loop;
    struct timespec req = {0, MILLI_SEC};

    for(loop=0; loop < wait_msec; loop++) {
        nanosleep(&req, NULL);
    }
}

MPU9250Operator::MPU9250Operator(const int _piId,const int _spi_handle):piId(_piId),spi_handle(_spi_handle) {
}

int MPU9250Operator::readGyroData(int16_t *destination){
    uint8_t rawData[6]={0};
    readBytes((uint8_t)GYRO_XOUT_H,6,rawData);
    
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
    return 0;
}

void MPU9250Operator::initMPU9250()
{
    // wake up device
    // Clear sleep mode bit (6), enable all sensors
    writeByte(PWR_MGMT_1, 0x00);
    wait_millisecond(100); // Wait for all registers to reset

    // Get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    writeByte(PWR_MGMT_1, 0x01);
    wait_millisecond(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
    // respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion
    // update rates cannot be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
    // 8 kHz, or 1 kHz
    writeByte( CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above.
    writeByte(SMPLRT_DIV, 0x04);

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
    // left-shifted into positions 4:3

    // get current GYRO_CONFIG register value
    uint8_t c = readByte(GYRO_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x02; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
    // GYRO_CONFIG
    // c =| 0x00;
    // Write new GYRO_CONFIG value to register
    writeByte( GYRO_CONFIG, c );

    // Set accelerometer full-scale range configuration
    // Get current ACCEL_CONFIG register value
    c = readByte(ACCEL_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
    // Write new ACCEL_CONFIG register value
    writeByte(ACCEL_CONFIG, c);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by
    // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
    // 1.13 kHz
    // Get current ACCEL_CONFIG2 register value
    c = readByte(ACCEL_CONFIG2);
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    // Write new ACCEL_CONFIG2 register value
    writeByte(ACCEL_CONFIG2, c);
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because
    // of the SMPLRT_DIV setting

    //以下割り込み設定？割り込むのか込まれるのかいまだに不詳だが，どちらにせよ使わないのでオフっとく
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of INT_STATUS, and enable
    // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
    // controlled by the Arduino as master.
//    writeByte(INT_PIN_CFG, 0x22);
    // Enable data ready (bit 0) interrupt
//    writeByte(INT_ENABLE, 0x01);
    wait_millisecond(200);

    this->calcAres();
    this->calcGres();
}

int MPU9250Operator::readAccelData(int16_t *destination){
    uint8_t rawData[6]={0};
    readBytes((uint8_t)ACCEL_XOUT_H,6,rawData);
    
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
    return 0;
}

uint8_t MPU9250Operator::readByte(uint8_t addr){
    uint8_t retVal;
    readBytes(addr,1,&retVal);
    return retVal;
 }

int MPU9250Operator::readBytes(uint8_t firstAddr,int cnt,uint8_t *rawVals){
    uint8_t tempAddrs[cnt+1]={0};
    uint8_t tempRawVals[cnt+1]={0};
    
    for(int i=0;i<cnt;i++){
        *(tempAddrs+i)=(firstAddr+i)|READ_FLAG;
    }
    *(tempAddrs+cnt)=DUMMYADDR | READ_FLAG;
    
    spi_xfer(piId,spi_handle,(char *)tempAddrs,(char *)tempRawVals,cnt+1);

   
    for(int i=0;i<cnt;i++){
        *(rawVals+i) = *(tempRawVals+i+1);
    }
        
    return 0;
}

//FEXME:実装おかしい．writeByte参照
int MPU9250Operator::writeBytes(uint8_t firstAddr,int cnt,uint8_t *rawVals){
    uint8_t tempAddrs[cnt]={0};
    
    for(int i=0;i<cnt;i++){
        *(tempAddrs+i)=(firstAddr+i)|READ_FLAG;
    }
    return spi_xfer(piId,spi_handle,(char *)tempAddrs,(char *)rawVals,cnt);
}

int MPU9250Operator::writeByte(uint8_t Addr,uint8_t Val){
    uint8_t sendAddrs[2]={Addr,Val};
    
    return spi_write(piId,spi_handle,(char *)sendAddrs,2);
}
// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250Operator::calibrateMPU9250(float * gyroBias, float * accelBias)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte( PWR_MGMT_1, READ_FLAG);
wait_millisecond(100);
  // get stable time source; Auto select clock source to be PLL gyroscope
  // reference if ready else use the internal oscillator, bits 2:0 = 001
  writeByte( PWR_MGMT_1, 0x01);
  writeByte( PWR_MGMT_2, 0x00);
wait_millisecond(200);

  // Configure device for bias calculation
  // Disable all interrupts
  writeByte( INT_ENABLE, 0x00);
  // Disable FIFO
  writeByte( FIFO_EN, 0x00);
  // Turn on internal clock source
  writeByte( PWR_MGMT_1, 0x00);
  // Disable I2C master
  writeByte( I2C_MST_CTRL, 0x00);
  // Disable FIFO and I2C master modes
  writeByte( USER_CTRL, 0x00);
  // Reset FIFO and DMP
  writeByte( USER_CTRL, 0x0C);
wait_millisecond(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  // Set low-pass filter to 188 Hz
  writeByte( CONFIG, 0x01);
  // Set sample rate to 1 kHz
  writeByte( SMPLRT_DIV, 0x00);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte( GYRO_CONFIG, 0x00);
  // Set accelerometer full-scale to 2 g, maximum sensitivity
  writeByte( ACCEL_CONFIG, 0x00);

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte( USER_CTRL, 0x40);  // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
  // MPU-9150)
  writeByte( FIFO_EN, 0x78);
wait_millisecond(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  // Disable gyro and accelerometer sensors for FIFO
  writeByte( FIFO_EN, 0x00);
  // Read FIFO sample count
  readBytes( FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    // Read data for averaging
    readBytes( FIFO_R_W, 12, &data[0]);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
    // biases.
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t) accelsensitivity;
  }
  else
  {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup.
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
  // format.
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
  // Biases are additive, so change sign on calculated average gyro biases
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
//  writeByte( XG_OFFSET_H, 0);
//  writeByte( XG_OFFSET_L, 0);
  writeByte( XG_OFFSET_H, data[0]);
  writeByte( XG_OFFSET_L, data[1]);
  writeByte( YG_OFFSET_H, data[2]);
  writeByte( YG_OFFSET_L, data[3]);
  writeByte( ZG_OFFSET_H, data[4]);
  writeByte( ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0];
//  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer
  // bias registers. These registers contain factory trim values which must be
  // added to the calculated accelerometer biases; on boot up these registers
  // will hold non-zero values. In addition, bit 0 of the lower byte must be
  // preserved since it is used for temperature compensation calculations.
  // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  // A place to hold the factory accelerometer trim biases
  int32_t accel_bias_reg[3] = {0, 0, 0};
  // Read factory accelerometer trim values
  readBytes( XA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes( YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes( ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  // Define mask for temperature compensation bit 0 of lower byte of
  // accelerometer bias registers
  uint32_t mask = 1uL;
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};

  for (ii = 0; ii < 3; ii++)
  {
    // If temperature compensation bit is set, record that fact in mask_bit
    if ((accel_bias_reg[ii] & mask))
    {
      mask_bit[ii] = 0x01;
    }
  }

  // Construct total accelerometer bias, including calculated average
  // accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
  // (16 g full scale)
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[5] = data[5] | mask_bit[2];

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte( XA_OFFSET_H, data[0]);
  writeByte( XA_OFFSET_L, data[1]);
  writeByte( YA_OFFSET_H, data[2]);
  writeByte( YA_OFFSET_L, data[3]);
  writeByte( ZA_OFFSET_H, data[4]);
  writeByte( ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
  accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
  accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
void MPU9250Operator::calcGres()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
      gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      gRes = 2000.0f / 32768.0f;
      break;
  }
}

void MPU9250Operator::calcAres()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
      aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      aRes = 16.0f / 32768.0f;
      break;
  }
}