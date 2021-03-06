//
//  devINA219.c
//  
//
//  Created by Harry Horsley on 17/11/2020.
//

#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

extern volatile WarpI2CDeviceState    deviceVEML7700State;
extern volatile uint32_t        gWarpI2cBaudRateKbps;
extern volatile uint32_t        gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t        gWarpSupplySettlingDelayMilliseconds;

void
initVEML7700(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
    deviceStatePointer->i2cAddress    = i2cAddress;
    deviceStatePointer->signalType    = (    kWarpTypeMaskALSOut);
    return;
}

WarpStatus
writeSensorRegisterVEML7700(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue)
{
    
    uint8_t        payloadByte[2], commandByte[1];
    i2c_status_t    status;
    
    switch (deviceRegister)
    {
        case 0x00: case 0x01: case 0x02: case 0x03:
        {
            /* OK */
            break;
        }
        
        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave =
    {
        .address = deviceVEML7700State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };
    
    commandByte[0] = deviceRegister;
    payloadByte[1] = (payload>>8) & 0xFF;
    payloadByte[0] = payload&(0xFF);
    
    status = I2C_DRV_MasterSendDataBlocking(
                            0 /* I2C instance */,
                            &slave,
                            commandByte,
                            1,
                            payloadByte,
                            2,
                            gWarpI2cTimeoutMilliseconds);
    
    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_WriteString(0, "\nDidnt write succesfully.");
        return kWarpStatusDeviceCommunicationFailed;
    }
    
    SEGGER_RTT_WriteString(0, "\nWritten successfully to VEML7700.");

    return kWarpStatusOK;
}

//WarpStatus
//configureSensorVEML7700(uint8_t payload_SETUP1, uint8_t payload_SETUP2, uint8_t payload_CALIB1, uint8_t payload_CALIB2, uint16_t menuI2cPullupValue)
//{
//    WarpStatus    i2cWriteStatus1, i2cWriteStatus2;
//
//    i2cWriteStatus1 = writeSensorRegisterVEML7700(0x00,
//                            payload_SETUP1, payload_SETUP2 /* payload: Disable FIFO */,
//                            menuI2cPullupValue);
//
//    i2cWriteStatus2 = writeSensorRegisterVEML7700(0x05,
//                            payload_CALIB1, payload_CALIB2 /* payload: Disable FIFO */,
//                            menuI2cPullupValue);
//
//    return (i2cWriteStatus1 | i2cWriteStatus2);
//}

WarpStatus
readSensorRegisterVEML7700(uint8_t deviceRegister, int numberOfBytes)
{
    uint8_t        cmdBuf[1] = {0xFF};
    i2c_status_t    status;


    USED(numberOfBytes);
    switch (deviceRegister)
    {
        case 0x00: case 0x01: case 0x02:
        case 0x04: case 0x05: case 0x06:
        {
            /* OK */
            break;
        }
        
        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }


    i2c_device_t slave =
    {
        .address = deviceVEML7700State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };


    cmdBuf[0] = deviceRegister;

    status = I2C_DRV_MasterReceiveDataBlocking(
                            0 /* I2C peripheral instance */,
                            &slave,
                            cmdBuf,
                            1,
                            (uint8_t *)deviceVEML7700State.i2cBuffer,
                            numberOfBytes,
                            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_WriteString(0, "\nDidnt READ successfully.");
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

uint16_t
printSensorDataVEML7700(void)
{
    uint16_t    readSensorRegisterValueLSB;
    uint16_t    readSensorRegisterValueMSB;
    int16_t        readSensorRegisterValueCombined;
    WarpStatus    i2cReadStatus;


    /* Read each of the measurement registers on the VEML7700 */
    
    i2cReadStatus = readSensorRegisterVEML7700(kWarpSensorOutputRegisterVEML7700ALSOut, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceVEML7700State.i2cBuffer[1];
    readSensorRegisterValueLSB = deviceVEML7700State.i2cBuffer[0];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);

    /*
     *    Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
     */
    uint16_t CurrentValue = readSensorRegisterValueCombined;
    
    if (i2cReadStatus != kWarpStatusOK)
            {
                return 0;
            }
    
    return CurrentValue;

//    SEGGER_RTT_WriteString(0, "\nALS reading (lux):");
//
//    if (i2cReadStatus != kWarpStatusOK)
//    {
//        SEGGER_RTT_WriteString(0, " ----,");
//    }
//    else
//    {
//        SEGGER_RTT_printf(0, " %d,", CurrentValue);
//
//    }
}
