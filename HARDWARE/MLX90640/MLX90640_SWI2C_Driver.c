/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
 /**
 * As the timings depend heavily on the MCU in use, it is recommended
 * to make sure that the proper timings are achieved. For that purpose
 * an oscilloscope might be needed to strobe the SCL and SDA signals.
 * The Wait(int) function could be modified in order to better 
 * trim the frequency. For coarse setting of the frequency or 
 * dynamic frequency change using the default function implementation, 
 * ‘freqCnt’ argument should be changed – lower value results in 
 * higher frequency.
 */
 
#include "MLX90640_I2C_Driver.h"
#include "math.h"
#include "stdio.h"
#include "myiic.h"


void MLX90640_I2CInit()
{   
	iic_init();
}


int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    uint8_t* bp = (uint8_t*) data;
    int cnt = 0;
    // Start I2C communication
    iic_start();
    
    // Send slave address with write bit (0)
    iic_send_byte((slaveAddr << 1) | 0); // Write mode
    if (iic_wait_ack()) {
        iic_stop();
        return 1; // Failed to get ACK
    }
    
    // Send the start address (16-bit)
    iic_send_byte((startAddress >> 8) & 0xFF); // Send high byte
    if (iic_wait_ack()) {
        iic_stop();
        return 2; // Failed to get ACK
    }
    
    iic_send_byte(startAddress & 0xFF); // Send low byte
    if (iic_wait_ack()) {
        iic_stop();
        return 3; // Failed to get ACK
    }
    
    // Restart I2C communication for reading
    iic_start();
    
    // Send slave address with read bit (1)
    iic_send_byte((slaveAddr << 1) | 1); // Read mode
    if (iic_wait_ack()) {
        iic_stop();
        return 4; // Failed to get ACK
    }
    
    // Read the data
    for (cnt = 0; cnt < nMemAddressRead * 2; cnt++) {
        bp[cnt] = iic_read_byte(cnt == (nMemAddressRead * 2 - 1) ? 0 : 1); // Send NACK for last byte
    }
    
    // Stop I2C communication
    iic_stop();
    
    // Swap bytes to match endianness if needed
    for (cnt = 0; cnt < nMemAddressRead * 2; cnt += 2) {
        uint8_t tmpbytelsb = bp[cnt + 1];
        bp[cnt + 1] = bp[cnt];
        bp[cnt] = tmpbytelsb;
    }
    
    return 0; // Success
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t cmd[2];
    uint16_t dataCheck;

    cmd[0] = data >> 8;     // 高字节
    cmd[1] = data & 0x00FF; // 低字节

    // Start I2C communication
    iic_start();
    
    // Send slave address with write bit (0)
    iic_send_byte((slaveAddr << 1) | 0); // Write mode
    if (iic_wait_ack()) {
        iic_stop();
        return -1; // Failed to get ACK
    }
    
    // Send the write address (16-bit)
    iic_send_byte((writeAddress >> 8) & 0xFF); // Send high byte
    if (iic_wait_ack()) {
        iic_stop();
        return -1; // Failed to get ACK
    }
    
    iic_send_byte(writeAddress & 0xFF); // Send low byte
    if (iic_wait_ack()) {
        iic_stop();
        return -1; // Failed to get ACK
    }
    
    // Send the data to write (2 bytes)
    iic_send_byte(cmd[0]); // Send high byte of data
    if (iic_wait_ack()) {
        iic_stop();
        return -1; // Failed to get ACK
    }

    iic_send_byte(cmd[1]); // Send low byte of data
    if (iic_wait_ack()) {
        iic_stop();
        return -1; // Failed to get ACK
    }

    // Stop I2C communication
    iic_stop();
    
    // Verify that data was written correctly by reading it back
    if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &dataCheck) != 0) {
        return -1; // Read failed
    }
    
    if (dataCheck != data) {
        return -2; // Data verification failed
    }    
    
    return 0; // Success
}

