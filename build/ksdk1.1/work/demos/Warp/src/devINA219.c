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

extern volatile WarpI2CDeviceState    deviceINA219State;
extern volatile uint32_t        gWarpI2cBaudRateKbps;
extern volatile uint32_t        gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t        gWarpSupplySettlingDelayMilliseconds;

void
initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
    deviceStatePointer->i2cAddress    = i2cAddress;
    return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payload1, uint8_t payload2, uint16_t menuI2cPullupValue)
{
    uint8_t        payloadBytes[2], commandByte[1];
    i2c_status_t    status;

    i2c_device_t slave =
    {
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    commandByte[0] = deviceRegister;
    payloadBytes[0] = payload2;
    payloadBytes[1] = payload1;
  
    
    
    status = I2C_DRV_MasterSendDataBlocking(
                            0 /* I2C instance */,
                            &slave,
                            commandByte,
                            1,
                            payloadBytes,
                            2,
                            gWarpI2cTimeoutMilliseconds);
    
    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "\nYou CANNOT write to this device!\n");
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(uint8_t payload_SETUP1, uint8_t payload_SETUP2, uint8_t payload_CALIB1, uint8_t payload_CALIB2, uint16_t menuI2cPullupValue)
{
    WarpStatus    i2cWriteStatus1, i2cWriteStatus2;

    i2cWriteStatus1 = writeSensorRegisterINA219(0x00,
                            payload_SETUP1, payload_SETUP2 /* payload: Disable FIFO */,
                            menuI2cPullupValue);
    
    i2cWriteStatus2 = writeSensorRegisterINA219(0x05,
                            payload_CALIB1, payload_CALIB2 /* payload: Disable FIFO */,
                            menuI2cPullupValue);

    return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
    uint8_t        cmdBuf[1];
    i2c_status_t    status;

    i2c_device_t slave =
    {
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    cmdBuf[0] = deviceRegister;

    status = I2C_DRV_MasterReceiveDataBlocking(
                            0 /* I2C peripheral instance */,
                            &slave,
                            cmdBuf,
                            1,
                            (uint8_t *)deviceINA219State.i2cBuffer,
                            2,
                            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "Device Communication FAILED son!");
        return kWarpStatusDeviceCommunicationFailed;
    }
    
    SEGGER_RTT_printf(0, "Device Communication Got to here son!");
    return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
    uint16_t    readSensorRegisterValueLSB;
    uint16_t    readSensorRegisterValueMSB;
    uint16_t     readSensorRegisterValueCombined;
    WarpStatus    i2cReadStatus;


    /*
     *    From the INA219 datasheet:
     *
     *        "A random read access to the LSB registers is not possible.
     *        Reading the MSB register and then the LSB register in sequence
     *        ensures that both bytes (LSB and MSB) belong to the same data
     *        sample, even if a new data sample arrives between reading the
     *        MSB and the LSB byte."
     *
     *    We therefore do 2-byte read transactions, for each of the registers.
     *    We could also improve things by doing a 6-byte read transaction.
     */
    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Current, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
    
    if (i2cReadStatus != kWarpStatusOK)
    {
        SEGGER_RTT_WriteString(0, " ----, doesn't work");
    }
    else
    {
        if (hexModeFlag)
        {
            SEGGER_RTT_printf(0, "Current: 0x%02x,", readSensorRegisterValueCombined);
        }
        else
        {
            SEGGER_RTT_printf(0, "Current is: %d,", readSensorRegisterValueCombined);
        }
    }
}
