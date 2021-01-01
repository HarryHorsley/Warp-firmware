//
//  devMultiplex.c
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
#include "devVEML7700.h"

extern volatile WarpI2CDeviceState    deviceMultiplexState;
extern volatile uint32_t        gWarpI2cBaudRateKbps;
extern volatile uint32_t        gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t        gWarpSupplySettlingDelayMilliseconds;

void
initMultiplex(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
    deviceStatePointer->i2cAddress    = i2cAddress;
    deviceStatePointer->signalType    = (kWarpTypeMaskShunt);
    return;
}

//WarpStatus
//writeSensorRegisterMultiplex(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue, uint8_t controlRegister)
//{
//
//    uint8_t        payloadByte[4], commandByte[1];
//    i2c_status_t    status;
//
//
//    i2c_device_t slave =
//    {
//        .address = deviceMultiplexState.i2cAddress,
//        .baudRate_kbps = gWarpI2cBaudRateKbps
//    };
//
//    commandByte[0] = controlRegister;
//    payloadByte[1] = 0x10;
//    payloadByte[0] = deviceRegister;
//    payloadByte[3] = (payload>>8) & 0xFF;
//    payloadByte[2] = payload&(0xFF);
//
//    status = I2C_DRV_MasterSendDataBlocking(
//                            0 /* I2C instance */,
//                            &slave,
//                            commandByte,
//                            1,
//                            payloadByte,
//                            4,
//                            gWarpI2cTimeoutMilliseconds);
//    if (status != kStatus_I2C_Success)
//    {
//        SEGGER_RTT_WriteString(0, "\nDidnt write succesfully to multiplex.");
//        return kWarpStatusDeviceCommunicationFailed;
//    }
//    return kWarpStatusOK;
//}

WarpStatus
writeSensorRegisterMultiplex(uint8_t deviceRegister)
{
    uint8_t *payload = 0x00;
    uint8_t        commandByte[1];
    i2c_status_t    status;

    
    i2c_device_t slave =
    {
        .address = deviceMultiplexState.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };
    
    commandByte[0] = deviceRegister;

    status = I2C_DRV_MasterSendDataBlocking(
                            0 /* I2C instance */,
                            &slave,
                            commandByte,
                            1,
                            payload,
                            0,
                            gWarpI2cTimeoutMilliseconds);
    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_WriteString(0, "\nDidnt write succesfully to multiplex.");
        return kWarpStatusDeviceCommunicationFailed;
    }
    return kWarpStatusOK;
}

uint16_t printSensorDataMultiplex(bool sensorChoice)
{
    uint16_t    value;
    uint8_t controlRegister;
    
    if (!sensorChoice){
        controlRegister = 0b00000001; // Channel 0
        writeSensorRegisterMultiplex(controlRegister);
    }
    
    else{
        controlRegister = 0b00000010; // Channel 1
        writeSensorRegisterMultiplex(controlRegister);
    }

    value = printSensorDataVEML7700();

    return value;
}
