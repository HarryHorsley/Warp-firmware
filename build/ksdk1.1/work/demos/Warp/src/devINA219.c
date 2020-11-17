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
    deviceStatePointer->signalType    = (kWarpTypeMaskCurrent);
    return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue)
{
    uint8_t        payloadByte[1], commandByte[1];
    i2c_status_t    status;

    switch (deviceRegister)
    {
        case 0x00: case 0x05:
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
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    commandByte[0] = deviceRegister;
    payloadByte[0] = payload;
    status = I2C_DRV_MasterSendDataBlocking(
                            0 /* I2C instance */,
                            &slave,
                            commandByte,
                            1,
                            payloadByte,
                            1,
                            gWarpI2cTimeoutMilliseconds);
    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(uint16_t payload_SETUP, uint16_t payload_CALIB, uint16_t menuI2cPullupValue)
{
    WarpStatus    i2cWriteStatus1, i2cWriteStatus2;

    i2cWriteStatus1 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219Config,
                            payload_SETUP /* payload: Disable FIFO */,
                            menuI2cPullupValue);
    
    i2cWriteStatus2 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219Calibration,
                            payload_CALIB /* payload: Disable FIFO */,
                            menuI2cPullupValue);

    return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
    uint8_t        cmdBuf[1] = {0xFF};
    i2c_status_t    status;


    USED(numberOfBytes);
    switch (deviceRegister)
    {
        case 0x00: case 0x01: case 0x02: case 0x03:
        case 0x04: case 0x05:
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
                            numberOfBytes,
                            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "Device Communication FAILED son!");
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
    uint16_t    readSensorRegisterValue;
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
    readSensorRegisterValue = deviceINA219State.i2cBuffer[0]; // Maybe get rid of the [0] here I don''t know

    if (i2cReadStatus != kWarpStatusOK)
    {
        SEGGER_RTT_WriteString(0, " ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            SEGGER_RTT_printf(0, "Current: 0x%02x,", readSensorRegisterValue);
        }
        else
        {
            SEGGER_RTT_printf(0, "Current: %d,", readSensorRegisterValue);
        }
    }
}
