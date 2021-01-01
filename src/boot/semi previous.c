 /*
    Authored 2016-2018. Phillip Stanley-Marbell.
    Additional contributions, 2018 onwards: Jan Heck, Chatura Samarakoon, Youchao Wang, Sam Willis.
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    *    Redistributions of source code must retain the above
        copyright notice, this list of conditions and the following
        disclaimer.
    *    Redistributions in binary form must reproduce the above
        copyright notice, this list of conditions and the following
        disclaimer in the documentation and/or other materials
        provided with the distribution.
    *    Neither the name of the author nor the names of its
        contributors may be used to endorse or promote products
        derived from this software without specific prior written
        permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTI ES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
#include "fsl_tpm_driver.h"
#include "fsl_tpm_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devINA219.h"
#include "devVEML7700.h"
#include "devMMA8451Q.h"
#include "devMultiplex.h"


#define WARP_FRDMKL03

#define WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF

#define                        kWarpConstantStringI2cFailure        "\rI2C failed, reg 0x%02x, code %d\n"
#define                        kWarpConstantStringErrorInvalidVoltage    "\rInvalid supply voltage [%d] mV!"
#define                        kWarpConstantStringErrorSanity        "\rSanity check failed!"

#ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
volatile WarpI2CDeviceState            deviceMMA8451QState;
#endif

#ifdef WARP_BUILD_ENABLE_DEVINA219
volatile WarpI2CDeviceState         deviceINA219State;
#endif

#ifdef WARP_BUILD_ENABLE_DEVVEML7700
volatile WarpI2CDeviceState         deviceVEML7700State;
#endif

volatile WarpI2CDeviceState         deviceMultiplexState;

/*
 *    TODO: move this and possibly others into a global structure
 */
volatile i2c_master_state_t            i2cMasterState;
volatile spi_master_state_t            spiMasterState;
volatile spi_master_user_config_t        spiUserConfig;
volatile lpuart_user_config_t             lpuartUserConfig;
volatile lpuart_state_t             lpuartState;

/*
 *    TODO: move magic default numbers into constant definitions.
 */
volatile uint32_t            gWarpI2cBaudRateKbps        = 200;
volatile uint32_t            gWarpUartBaudRateKbps        = 1;
volatile uint32_t            gWarpSpiBaudRateKbps        = 200;
volatile uint32_t            gWarpSleeptimeSeconds        = 0;
volatile WarpModeMask            gWarpMode            = kWarpModeDisableAdcOnSleep;
volatile uint32_t            gWarpI2cTimeoutMilliseconds    = 5;
volatile uint32_t            gWarpSpiTimeoutMicroseconds    = 5;
volatile uint32_t            gWarpMenuPrintDelayMilliseconds    = 10;
volatile uint32_t            gWarpSupplySettlingDelayMilliseconds = 1;

void                    sleepUntilReset(void);
void                    lowPowerPinStates(void);
void                    disableTPS82740A(void);
void                    disableTPS82740B(void);
void                    enableTPS82740A(uint16_t voltageMillivolts);
void                    enableTPS82740B(uint16_t voltageMillivolts);
void                    setTPS82740CommonControlLines(uint16_t voltageMillivolts);
void                    printPinDirections(void);
void                    dumpProcessorState(void);
void                    repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress,
                                uint8_t pullupValue, bool autoIncrement, int chunkReadsPerAddress, bool chatty,
                                int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
                                uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);
int                    char2int(int character);
void                    enableSssupply(uint16_t voltageMillivolts);
void                    disableSssupply(void);
void                    activateAllLowPowerSensorModes(bool verbose);
void                    powerupAllSensors(void);
uint8_t                    readHexByte(void);
int                    read4digits(void);
void                    printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, int i2cPullupValue);

// My Declarations

bool initSG90(void);
uint16_t readSensor(bool sensorChoice);
uint32_t trackLight(uint32_t percentage, tpm_pwm_param_t *param);
void pwmPulse(float degrees);
bool initSG902(void);
bool initSG903(void);

/*
 *    TODO: change the following to take byte arrays
 */
WarpStatus                writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus                writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);


void                    warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);



/*
 *    From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *    static clock callback table.
 */
clock_manager_callback_user_config_t        clockManagerCallbackUserlevelStructure =
                                    {
                                        .callback    = clockManagerCallbackRoutine,
                                        .callbackType    = kClockManagerCallbackBeforeAfter,
                                        .callbackData    = NULL
                                    };

static clock_manager_callback_user_config_t *    clockCallbackTable[] =
                                    {
                                        &clockManagerCallbackUserlevelStructure
                                    };

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
    clock_manager_error_code_t result = kClockManagerSuccess;

    switch (notify->notifyType)
    {
        case kClockManagerNotifyBefore:
            break;
        case kClockManagerNotifyRecover:
        case kClockManagerNotifyAfter:
            break;
        default:
            result = kClockManagerError;
        break;
    }

    return result;
}


/*
 *    Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
    if (RTC_DRV_IsAlarmPending(0))
    {
        RTC_DRV_SetAlarmIntCmd(0, false);
    }
}

/*
 *    Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
    gWarpSleeptimeSeconds++;
}

/*
 *    Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *  notify,
                    power_manager_callback_data_t *  dataPtr)
{
    WarpPowerManagerCallbackStructure *        callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
    power_manager_error_code_t            status = kPowerManagerError;

    switch (notify->notifyType)
    {
        case kPowerManagerNotifyBefore:
            status = kPowerManagerSuccess;
            break;
        case kPowerManagerNotifyAfter:
            status = kPowerManagerSuccess;
            break;
        default:
            callbackUserData->errorCount++;
            break;
    }

    return status;
}

/*
 *    From KSDK power_manager_demo.c <<END>>>
 */



void
sleepUntilReset(void)
{
    while (1)
    {
#ifdef WARP_BUILD_ENABLE_DEVSI4705
        GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
#endif
        warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);
#ifdef WARP_BUILD_ENABLE_DEVSI4705
        GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
#endif
        warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
    }
}


void
enableLPUARTpins(void)
{
    /*    Enable UART CLOCK */
    CLOCK_SYS_EnableLpuartClock(0);

    /*
    *    set UART pin association
    *    see page 99 in https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
    */

#ifdef WARP_BUILD_ENABLE_DEVPAN1326
    /*    Warp KL03_UART_HCI_TX    --> PTB3 (ALT3)    --> PAN1326 HCI_RX */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
    /*    Warp KL03_UART_HCI_RX    --> PTB4 (ALT3)    --> PAN1326 HCI_RX */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

    /* TODO: Partial Implementation */
    /*    Warp PTA6 --> PAN1326 HCI_RTS */
    /*    Warp PTA7 --> PAN1326 HCI_CTS */
#endif

    /*
     *    Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
     *
     */
    lpuartUserConfig.baudRate = 115;
    lpuartUserConfig.parityMode = kLpuartParityDisabled;
    lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
    lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;

    LPUART_DRV_Init(0,(lpuart_state_t *)&lpuartState,(lpuart_user_config_t *)&lpuartUserConfig);

}


void
disableLPUARTpins(void)
{
    /*
     *    LPUART deinit
     */
    LPUART_DRV_Deinit(0);

    /*    Warp KL03_UART_HCI_RX    --> PTB4 (GPIO)    */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);
    /*    Warp KL03_UART_HCI_TX    --> PTB3 (GPIO) */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

#ifdef WARP_BUILD_ENABLE_DEVPAN1326
    GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_HCI_CTS);
    GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_HCI_CTS);
#endif

    GPIO_DRV_ClearPinOutput(kWarpPinLPUART_HCI_TX);
    GPIO_DRV_ClearPinOutput(kWarpPinLPUART_HCI_RX);

    /* Disable LPUART CLOCK */
    CLOCK_SYS_DisableLpuartClock(0);

}

void
enableSPIpins(void)
{
    CLOCK_SYS_EnableSpiClock(0);

    /*    Warp KL03_SPI_MISO    --> PTA6    (ALT3)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

    /*    Warp KL03_SPI_MOSI    --> PTA7    (ALT3)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

    /*    Warp KL03_SPI_SCK    --> PTB0    (ALT3)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);


    /*
     *    Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
     *
     */
    uint32_t            calculatedBaudRate;
    spiUserConfig.polarity        = kSpiClockPolarity_ActiveHigh;
    spiUserConfig.phase        = kSpiClockPhase_FirstEdge;
    spiUserConfig.direction        = kSpiMsbFirst;
    spiUserConfig.bitsPerSec    = gWarpSpiBaudRateKbps * 1000;
    SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
    SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}



void
disableSPIpins(void)
{
    SPI_DRV_MasterDeinit(0);


    /*    Warp KL03_SPI_MISO    --> PTA6    (GPI)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

    /*    Warp KL03_SPI_MOSI    --> PTA7    (GPIO)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

    /*    Warp KL03_SPI_SCK    --> PTB0    (GPIO)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);


    CLOCK_SYS_DisableSpiClock(0);
}

void
configureI2Cpins(uint8_t pullupValue)
{
#ifdef WARP_BUILD_ENABLE_DEVISL23415
    /*
     *    Configure the two ISL23415 DCPs over SPI
     */
    uint8_t valuesDCP[2] = {pullupValue, pullupValue};
    writeDeviceRegisterISL23415(kWarpISL23415RegWR, valuesDCP, 4);
#endif
}

void
enableI2Cpins(uint8_t pullupValue)
{
    CLOCK_SYS_EnableI2cClock(0);

    /*    Warp KL03_I2C0_SCL    --> PTB3    (ALT2 == I2C)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

    /*    Warp KL03_I2C0_SDA    --> PTB4    (ALT2 == I2C)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);


    I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);

    configureI2Cpins(pullupValue);
}



void
disableI2Cpins(void)
{
    I2C_DRV_MasterDeinit(0 /* I2C instance */);


    /*    Warp KL03_I2C0_SCL    --> PTB3    (GPIO)            */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

    /*    Warp KL03_I2C0_SDA    --> PTB4    (GPIO)            */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);


    /*
     *    Reset DCP configuration
     */
    configureI2Cpins(0x80); /* Defaults DCP configuration ISL datasheet FN7780 Rev 2.00 - page 14 */

    /*
     *    Drive the I2C pins low
     */
    GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
    GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);


    CLOCK_SYS_DisableI2cClock(0);
}


// TODO: add pin states for pan1326 lp states
void
lowPowerPinStates(void)
{
    /*
     *    Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
     *    we configure all pins as output and set them to a known state. We choose
     *    to set them all to '0' since it happens that the devices we want to keep
     *    deactivated (SI4705, PAN1326) also need '0'.
     */

    /*
     *            PORT A
     */
    /*
     *    For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
     */
    /*
    PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAsGpio);
    */

    /*
     *    PTA3 and PTA4 are the EXTAL/XTAL
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

    PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);

    /*
     *    NOTE: The KL03 has no PTA10 or PTA11
     */

    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);



    /*
     *            PORT B
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

    /*
     *    PTB1 is connected to KL03_VDD. We have a choice of:
     *        (1) Keep 'disabled as analog'.
     *        (2) Set as output and drive high.
     *
     *    Pin state "disabled" means default functionality (ADC) is _active_
     */
    if (gWarpMode & kWarpModeDisableAdcOnSleep)
    {
        PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
    }
    else
    {
        PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
    }

    PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

    /*
     *    PTB3 and PTB3 (I2C pins) are true open-drain
     *    and we purposefully leave them disabled.
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);


    PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);

    /*
     *    NOTE: The KL03 has no PTB8 or PTB9
     */

    PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

    /*
     *    NOTE: The KL03 has no PTB12
     */

    PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAsGpio);



    /*
     *    Now, set all the pins (except kWarpPinKL03_VDD_ADC, the SWD pins, and the XTAL/EXTAL) to 0
     */



    /*
     *    If we are in mode where we disable the ADC, then drive the pin high since it is tied to KL03_VDD
     */
    if (gWarpMode & kWarpModeDisableAdcOnSleep)
    {
        GPIO_DRV_SetPinOutput(kWarpPinKL03_VDD_ADC);
    }
#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
#ifdef WARP_BUILD_ENABLE_DEVPAN1326
    GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);
#endif
#endif

    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
    GPIO_DRV_ClearPinOutput(kWarpPinCLKOUT32K);
#endif

    GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

    /*
     *    Drive these chip selects high since they are active low:
     */
#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
    GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);
#endif
#ifdef WARP_BUILD_ENABLE_DEVADXL362
    GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS);
#endif

    /*
     *    When the PAN1326 is installed, note that it has the
     *    following pull-up/down by default:
     *
     *        HCI_RX / kWarpPinI2C0_SCL    : pull up
     *        HCI_TX / kWarpPinI2C0_SDA    : pull up
     *        HCI_RTS / kWarpPinSPI_MISO    : pull up
     *        HCI_CTS / kWarpPinSPI_MOSI    : pull up
     *
     *    These I/Os are 8mA (see panasonic_PAN13xx.pdf, page 10),
     *    so we really don't want to be driving them low. We
     *    however also have to be careful of the I2C pullup and
     *    pull-up gating. However, driving them high leads to
     *    higher board power dissipation even when SSSUPPLY is off
     *    by ~80mW on board #003 (PAN1326 populated).
     *
     *    In revB board, with the ISL23415 DCP pullups, we also
     *    want I2C_SCL and I2C_SDA driven high since when we
     *    send a shutdown command to the DCP it will connect
     *    those lines to 25570_VOUT.
     *
     *    For now, we therefore leave the SPI pins low and the
     *    I2C pins (PTB3, PTB4, which are true open-drain) disabled.
     */

    GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
    GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

    /*
     *    HCI_RX / kWarpPinI2C0_SCL is an input. Set it low.
     */
    //GPIO_DRV_SetPinOutput(kWarpPinI2C0_SCL);

    /*
     *    HCI_TX / kWarpPinI2C0_SDA is an output. Set it high.
     */
    //GPIO_DRV_SetPinOutput(kWarpPinI2C0_SDA);

    /*
     *    HCI_RTS / kWarpPinSPI_MISO is an output. Set it high.
     */
    //GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO);

    /*
     *    From PAN1326 manual, page 10:
     *
     *        "When HCI_CTS is high, then CC256X is not allowed to send data to Host device"
     */
    //GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI);
}



void
disableTPS82740A(void)
{
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
}

void
disableTPS82740B(void)
{
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
}


void
enableTPS82740A(uint16_t voltageMillivolts)
{
    setTPS82740CommonControlLines(voltageMillivolts);
    GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_CTLEN);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);

    /*
     *    Select the TS5A3154 to use the output of the TPS82740
     *
     *        IN = high selects the output of the TPS82740B:
     *        IN = low selects the output of the TPS82740A:
     */
    GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
}


void
enableTPS82740B(uint16_t voltageMillivolts)
{
    setTPS82740CommonControlLines(voltageMillivolts);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
    GPIO_DRV_SetPinOutput(kWarpPinTPS82740B_CTLEN);

    /*
     *    Select the TS5A3154 to use the output of the TPS82740
     *
     *        IN = high selects the output of the TPS82740B:
     *        IN = low selects the output of the TPS82740A:
     */
    GPIO_DRV_SetPinOutput(kWarpPinTS5A3154_IN);
}


void
setTPS82740CommonControlLines(uint16_t voltageMillivolts)
{
    /*
     *     From Manual:
     *
     *        TPS82740A:    VSEL1 VSEL2 VSEL3:    000-->1.8V, 111-->2.5V
     *        TPS82740B:    VSEL1 VSEL2 VSEL3:    000-->2.6V, 111-->3.3V
     */

    switch(voltageMillivolts)
    {
        case 2600:
        case 1800:
        {
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

            break;
        }

        case 2700:
        case 1900:
        {
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

            break;
        }

        case 2800:
        case 2000:
        {
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

            break;
        }

        case 2900:
        case 2100:
        {
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

            break;
        }

        case 3000:
        case 2200:
        {
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

            break;
        }

        case 3100:
        case 2300:
        {
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

            break;
        }

        case 3200:
        case 2400:
        {
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

            break;
        }

        case 3300:
        case 2500:
        {
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

            break;
        }

        /*
         *    Should never happen, due to previous check in enableSssupply()
         */
        default:
        {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
            SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
#endif
        }
    }

    /*
     *    Vload ramp time of the TPS82740 is 800us max (datasheet, Section 8.5 / page 5)
     */
    OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
}

void
enableSssupply(uint16_t voltageMillivolts)
{
    if (voltageMillivolts >= 1800 && voltageMillivolts <= 2500)
    {
        enableTPS82740A(voltageMillivolts);
    }
    else if (voltageMillivolts >= 2600 && voltageMillivolts <= 3300)
    {
        enableTPS82740B(voltageMillivolts);
    }
    else
    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
#endif
    }
}



void
disableSssupply(void)
{
    disableTPS82740A();
    disableTPS82740B();

    /*
     *    Clear the pin. This sets the TS5A3154 to use the output of the TPS82740B,
     *    which shouldn't matter in any case. The main objective here is to clear
     *    the pin to reduce power drain.
     *
     *        IN = high selects the output of the TPS82740B:
     *        IN = low selects the output of the TPS82740A:
     */
    GPIO_DRV_SetPinOutput(kWarpPinTS5A3154_IN);

    /*
     *    Vload ramp time of the TPS82740 is 800us max (datasheet, Section 8.5 / page 5)
     */
    OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
}



void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
    /*
     *    Set all pins into low-power states. We don't just disable all pins,
     *    as the various devices hanging off will be left in higher power draw
     *    state. And manuals say set pins to output to reduce power.
     */
    if (forceAllPinsIntoLowPowerState)
    {
        lowPowerPinStates();
    }

    warpSetLowPowerMode(kWarpPowerModeVLPR, 0);
    warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
}



void
printPinDirections(void)
{
    /*
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
    SEGGER_RTT_printf(0, "KL03_VDD_ADC:%d\n", GPIO_DRV_GetPinDir(kWarpPinKL03_VDD_ADC));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "I2C0_SDA:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SDA));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "I2C0_SCL:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SCL));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "SPI_MOSI:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MOSI));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "SPI_MISO:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MISO));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "SPI_SCK_I2C_PULLUP_EN:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_SCK_I2C_PULLUP_EN));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740A_VSEL2:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740_VSEL2));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "ADXL362_CS:%d\n", GPIO_DRV_GetPinDir(kWarpPinADXL362_CS));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "kWarpPinPAN1326_nSHUTD:%d\n", GPIO_DRV_GetPinDir(kWarpPinPAN1326_nSHUTD));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740A_CTLEN:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740A_CTLEN));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740B_CTLEN:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740B_CTLEN));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740A_VSEL1:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740_VSEL1));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740A_VSEL3:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740_VSEL3));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "CLKOUT32K:%d\n", GPIO_DRV_GetPinDir(kWarpPinCLKOUT32K));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TS5A3154_IN:%d\n", GPIO_DRV_GetPinDir(kWarpPinTS5A3154_IN));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "SI4705_nRST:%d\n", GPIO_DRV_GetPinDir(kWarpPinSI4705_nRST));
    OSA_TimeDelay(100);
#endif
    */
}



void
dumpProcessorState(void)
{
/*
    uint32_t    cpuClockFrequency;
    CLOCK_SYS_GetFreq(kCoreClock, &cpuClockFrequency);
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
    SEGGER_RTT_printf(0, "\r\n\n\tCPU @ %u KHz\n", (cpuClockFrequency / 1000));
    SEGGER_RTT_printf(0, "\r\tCPU power mode: %u\n", POWER_SYS_GetCurrentMode());
    SEGGER_RTT_printf(0, "\r\tCPU clock manager configuration: %u\n", CLOCK_SYS_GetCurrentConfiguration());
    SEGGER_RTT_printf(0, "\r\tRTC clock: %d\n", CLOCK_SYS_GetRtcGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tSPI clock: %d\n", CLOCK_SYS_GetSpiGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tI2C clock: %d\n", CLOCK_SYS_GetI2cGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tLPUART clock: %d\n", CLOCK_SYS_GetLpuartGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tPORT A clock: %d\n", CLOCK_SYS_GetPortGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tPORT B clock: %d\n", CLOCK_SYS_GetPortGateCmd(1));
    SEGGER_RTT_printf(0, "\r\tFTF clock: %d\n", CLOCK_SYS_GetFtfGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tADC clock: %d\n", CLOCK_SYS_GetAdcGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tCMP clock: %d\n", CLOCK_SYS_GetCmpGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tVREF clock: %d\n", CLOCK_SYS_GetVrefGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tTPM clock: %d\n", CLOCK_SYS_GetTpmGateCmd(0));
#endif
*/
}

#ifdef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
void
addAndMultiplicationBusyLoop(long iterations)
{
    int value;
    for (volatile long i = 0; i < iterations; i++)
    {
        value = kWarpThermalChamberBusyLoopAdder + value * kWarpThermalChamberBusyLoopMutiplier;
    }
}

uint8_t
checkSum(uint8_t *  pointer, uint16_t length) /*    Adapted from https://stackoverflow.com/questions/31151032/writing-an-8-bit-checksum-in-c    */
{
    unsigned int sum;
    for ( sum = 0 ; length != 0 ; length-- )
    {
        sum += *(pointer++);
    }
    return (uint8_t)sum;
}
#endif

int
main(void)
{
    uint8_t                    key;
    WarpSensorDevice            menuTargetSensor = kWarpSensorVEML7700;
    volatile WarpI2CDeviceState *        menuI2cDevice = NULL;
    uint16_t                menuI2cPullupValue = 32768;
    uint8_t                    menuRegisterAddress = 0x00;
    uint16_t                menuSupplyVoltage = 0;

    

    rtc_datetime_t                warpBootDate;

    power_manager_user_config_t        warpPowerModeWaitConfig;
    power_manager_user_config_t        warpPowerModeStopConfig;
    power_manager_user_config_t        warpPowerModeVlpwConfig;
    power_manager_user_config_t        warpPowerModeVlpsConfig;
    power_manager_user_config_t        warpPowerModeVlls0Config;
    power_manager_user_config_t        warpPowerModeVlls1Config;
    power_manager_user_config_t        warpPowerModeVlls3Config;
    power_manager_user_config_t        warpPowerModeRunConfig;

    const power_manager_user_config_t    warpPowerModeVlprConfig = {
                            .mode            = kPowerManagerVlpr,
                            .sleepOnExitValue    = false,
                            .sleepOnExitOption    = false
                        };

    power_manager_user_config_t const *    powerConfigs[] = {
                            /*
                             *    NOTE: This order is depended on by POWER_SYS_SetMode()
                             *
                             *    See KSDK13APIRM.pdf Section 55.5.3
                             */
                            &warpPowerModeWaitConfig,
                            &warpPowerModeStopConfig,
                            &warpPowerModeVlprConfig,
                            &warpPowerModeVlpwConfig,
                            &warpPowerModeVlpsConfig,
                            &warpPowerModeVlls0Config,
                            &warpPowerModeVlls1Config,
                            &warpPowerModeVlls3Config,
                            &warpPowerModeRunConfig,
                        };

    WarpPowerManagerCallbackStructure            powerManagerCallbackStructure;

    /*
     *    Callback configuration structure for power manager
     */
    const power_manager_callback_user_config_t callbackCfg0 = {
                            callback0,
                            kPowerManagerCallbackBeforeAfter,
                            (power_manager_callback_data_t *) &powerManagerCallbackStructure};

    /*
     *    Pointers to power manager callbacks.
     */
    power_manager_callback_user_config_t const *    callbacks[] = {
                                &callbackCfg0
                        };



    /*
     *    Enable clock for I/O PORT A and PORT B
     */
    CLOCK_SYS_EnablePortClock(0);
    CLOCK_SYS_EnablePortClock(1);



    /*
     *    Setup board clock source.
     */
    g_xtal0ClkFreq = 32768U;



    /*
     *    Initialize KSDK Operating System Abstraction layer (OSA) layer.
     */
    OSA_Init();



    /*
     *    Setup SEGGER RTT to output as much as fits in buffers.
     *
     *    Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
     *    we might have SWD disabled at time of blockage.
     */
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);


    SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Warp, in 3... ");
    OSA_TimeDelay(200);
    SEGGER_RTT_WriteString(0, "2... ");
    OSA_TimeDelay(200);
    SEGGER_RTT_WriteString(0, "1...\n\r");
    OSA_TimeDelay(200);



    /*
     *    Configure Clock Manager to default, and set callback for Clock Manager mode transition.
     *
     *    See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
     */
    CLOCK_SYS_Init(    g_defaultClockConfigurations,
            CLOCK_CONFIG_NUM,
            &clockCallbackTable,
            ARRAY_SIZE(clockCallbackTable)
            );
    CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);



    /*
     *    Initialize RTC Driver
     */
    RTC_DRV_Init(0);



    /*
     *    Set initial date to 1st January 2016 00:00, and set date via RTC driver
     */
    warpBootDate.year    = 2016U;
    warpBootDate.month    = 1U;
    warpBootDate.day    = 1U;
    warpBootDate.hour    = 0U;
    warpBootDate.minute    = 0U;
    warpBootDate.second    = 0U;
    RTC_DRV_SetDatetime(0, &warpBootDate);



    /*
     *    Setup Power Manager Driver
     */
    memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));


    warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
    warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

    warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
    warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

    warpPowerModeWaitConfig = warpPowerModeVlprConfig;
    warpPowerModeWaitConfig.mode = kPowerManagerWait;

    warpPowerModeStopConfig = warpPowerModeVlprConfig;
    warpPowerModeStopConfig.mode = kPowerManagerStop;

    warpPowerModeVlls0Config = warpPowerModeVlprConfig;
    warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

    warpPowerModeVlls1Config = warpPowerModeVlprConfig;
    warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

    warpPowerModeVlls3Config = warpPowerModeVlprConfig;
    warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

    warpPowerModeRunConfig.mode = kPowerManagerRun;

    POWER_SYS_Init(    &powerConfigs,
            sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
            &callbacks,
            sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
            );



    /*
     *    Switch CPU to Very Low Power Run (VLPR) mode
     */
    warpSetLowPowerMode(kWarpPowerModeVLPR, 0);



    /*
     *    Initialize the GPIO pins with the appropriate pull-up, etc.,
     *    defined in the inputPins and outputPins arrays (gpio_pins.c).
     *
     *    See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
     */
    GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);

    /*
     *    Note that it is lowPowerPinStates() that sets the pin mux mode,
     *    so until we call it pins are in their default state.
     */
    lowPowerPinStates();



    /*
     *    Toggle LED3 (kWarpPinSI4705_nRST)
     */
    GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);



    /*
     *    Initialize all the sensors
     */
#ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
    initMMA8451Q(    0x1C    /* i2cAddress */,    &deviceMMA8451QState    );
#endif
    
#ifdef WARP_BUILD_ENABLE_DEVINA219
    initINA219( 0x40 /* i2cAddress */, &deviceINA219State);
#endif
    
#ifdef WARP_BUILD_ENABLE_DEVVEML7700
    initVEML7700(  0x10 /* i2cAddress */, &deviceVEML7700State);
#endif
    
    initMultiplex(0x70, &deviceMultiplexState);

    /*
     *    Make sure SCALED_SENSOR_SUPPLY is off.
     *
     *    (There's no point in calling activateAllLowPowerSensorModes())
     */
    disableSssupply();


    /*
     *    TODO: initialize the kWarpPinKL03_VDD_ADC, write routines to read the VDD and temperature
     */




#ifdef WARP_BUILD_BOOT_TO_CSVSTREAM
    /*
     *    Force to printAllSensors
     */
    gWarpI2cBaudRateKbps = 300;
    warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */);
    enableSssupply(3000);
    enableI2Cpins(menuI2cPullupValue);
    printAllSensors(false /* printHeadersAndCalibration */, true /* hexModeFlag */, 0 /* menuDelayBetweenEachRun */, menuI2cPullupValue);
    /*
     *    Notreached
     */
#endif

    // My new variables
    
    uint16_t                    GainSetting;
    uint16_t                   IntegrationTimeMilliseconds;
    uint16_t                  GainConfig = 0;
    uint16_t                  ITConfig = 0;
    
    while (1)
    {
        /*
         *    Do not, e.g., lowPowerPinStates() on each iteration, because we actually
         *    want to use menu to progressiveley change the machine state with various
         *    commands.
         */

        /*
         *    We break up the prints with small delays to allow us to use small RTT print
         *    buffers without overrunning them when at max CPU speed.
         */
        SEGGER_RTT_WriteString(0, "\r\n\n\n\n[ *\t\t\t\tW\ta\tr\tp\t(rev. b)\t\t\t* ]\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

        SEGGER_RTT_WriteString(0, "\rSelect:\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- 'a': set default sensor.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- 'g': Set supply voltage.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- 'n': read from INA219 sensor\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '0': Configure VEML7700 sensor with no Multiplexer\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '1': Configure VEML7700 sensors with Multiplexer\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '2': Read from VEML7700 sensor with specified readings\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '3': Read from VEML7700 sensor with specified readings and multiplexer\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '4': Initialise the SG90\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '5': PWM pulse\n");
        SEGGER_RTT_WriteString(0, "\rEnter selection> ");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        key = SEGGER_RTT_WaitKey();
        
        switch (key)
        {
            /*
             *        Select sensor
             */
            case 'a':
            {
                SEGGER_RTT_WriteString(0, "\r\tSelect:\n");

#ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
                SEGGER_RTT_WriteString(0, "\r\t- '5' MMA8451Q            (0x00--0x31): 1.95V -- 3.6V\n");
                #else
                SEGGER_RTT_WriteString(0, "\r\t- '5' MMA8451Q            (0x00--0x31): 1.95V -- 3.6V (compiled out) \n");
#endif
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

#ifdef WARP_BUILD_ENABLE_DEVINA219
                SEGGER_RTT_WriteString(0, "\r\t- 'n' INA219            (0x00--0x31): 1.95V -- 3.6V\n");
                #else
                SEGGER_RTT_WriteString(0, "\r\t- 'n' INA219            (0x00--0x31): 1.95V -- 3.6V (compiled out) \n");
#endif
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                
#ifdef WARP_BUILD_ENABLE_DEVVEML7700
                SEGGER_RTT_WriteString(0, "\r\t- 'v' VEML7700            (0x00--0x31): 1.95V -- 3.6V\n");
                #else
                SEGGER_RTT_WriteString(0, "\r\t- 'v' VEML7700            (0x00--0x31): 1.95V -- 3.6V (compiled out) \n");
#endif
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

                SEGGER_RTT_WriteString(0, "\r\tEnter selection> ");
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

                key = SEGGER_RTT_WaitKey();
                
                switch(key)
                {
#ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
                    case '5':
                    {
                        menuTargetSensor = kWarpSensorMMA8451Q;
                        menuI2cDevice = &deviceMMA8451QState;
                        break;
                    }
#endif
#ifdef WARP_BUILD_ENABLE_DEVINA219
                    case 'n':
                    {
                        menuTargetSensor = kWarpSensorINA219;
                        menuI2cDevice = &deviceINA219State;
                        break;
                    }
#endif
#ifdef WARP_BUILD_ENABLE_DEVVEML7700
                    case 'v':
                    {
                        menuTargetSensor = kWarpSensorVEML7700;
                        menuI2cDevice = &deviceVEML7700State;
                        break;
                    }
#endif
                    default:
                    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                        SEGGER_RTT_printf(0, "\r\tInvalid selection '%c' !\n", key);
#endif
                    }
                }

                break;
            }
                
            case 'g':
                        {
                            SEGGER_RTT_WriteString(0, "\r\n\tOverride SSSUPPLY in mV (e.g., '1800')> ");
                            menuSupplyVoltage = read4digits();
            #ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                            SEGGER_RTT_printf(0, "\r\n\tOverride SSSUPPLY set to %d mV", menuSupplyVoltage);
            #endif

                            break;
                        }
                
            case 'n':
            {

                SEGGER_RTT_WriteString(0, "\r\n\tEnabling I2C pins...\n");
                enableI2Cpins(menuI2cPullupValue);
                
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                
                //write to config register
                writeSensorRegisterINA219(0x00, 0x019F, menuI2cPullupValue);
                
                //write to calibration register
                writeSensorRegisterINA219(0x05, 0x2000, menuI2cPullupValue);
                
                SEGGER_RTT_WriteString(0, "\nCompleted configuration for current sensor.\n");
                
                SEGGER_RTT_WriteString(0, "INA219 Current sensor data below:\n");
            
             
                printSensorDataINA219(); // NO hex mode flag, taken out
                
                
                disableI2Cpins();
                break;

            }
            case '0':
            {
                // Set gain parameter
                bool GainSet = false;
                while(GainSet == false){
                SEGGER_RTT_WriteString(0, "\r\n\tWhat is the gain:\n1 - x2\n2 - x1\n3 - x1/4\n4 - x1/8\n");
                uint16_t GainChoice = SEGGER_RTT_WaitKey();
                    switch (GainChoice){
                        case '1':{
                            GainSetting = 2;
                            GainConfig = 0b01;
                            GainSet = true;
                            break;
                        }
                        case '2':{
                            GainSetting = 1;
                            GainConfig = 0b00;
                            GainSet = true;
                            break;
                        }
                        case '3':{
                            GainSetting = 1/4;
                            GainConfig = 0b11;
                            GainSet = true;
                            break;
                        }
                        case '4':{
                            GainSetting = 1/8;
                            GainConfig = 0b10;
                            GainSet = true;
                            break;
                        }
                        default:
                        {
                            SEGGER_RTT_WriteString(0, "\r\n\tInvalid input. Try again.");
                        }
                    }
                }
                
                // Set integration time
                bool ITSet = false;
                while(ITSet != true){
                SEGGER_RTT_WriteString(0, "\r\n\tWhat is the Integration Time:\n1 - 800ms\n2 - 400ms\n3 - 200ms\n4 - 100ms\n5 - 50ms\n6 - 25ms\n");
                    uint16_t ITChoice = SEGGER_RTT_WaitKey();
                        switch (ITChoice){
                            case '1':{
                                IntegrationTimeMilliseconds = 800;
                                ITConfig = 0b0011;
                                ITSet = true;
                                break;
                            }
                            case '2':{
                                IntegrationTimeMilliseconds = 400;
                                ITConfig = 0b0010;
                                ITSet = true;
                                break;
                            }
                            case '3':{
                                IntegrationTimeMilliseconds = 200;
                                ITConfig = 0b0001;
                                ITSet = true;
                                break;
                            }
                            case '4':{
                                IntegrationTimeMilliseconds = 100;
                                ITConfig = 0b0000;
                                ITSet = true;
                                break;
                            }
                            case '5': {
                                IntegrationTimeMilliseconds = 50;
                                ITConfig = 0b1000;
                                ITSet = true;
                                break;
                            }
                            case '6':{
                                IntegrationTimeMilliseconds = 25;
                                ITConfig = 0b1100;
                                ITSet = true;
                                break;
                            }
                            default:
                            {
                                SEGGER_RTT_WriteString(0, "\r\n\tInvalid input. Try again.");
                            }
                        }
                    }
                
                
                uint16_t ALSConfiguration = (GainConfig << 12) | (ITConfig << 6);
                
                SEGGER_RTT_printf(0, "\r\n\tALSConfiguration is %d", ALSConfiguration);
                
                // Write to config register
                
                enableI2Cpins(menuI2cPullupValue);
                
                writeSensorRegisterVEML7700(0x00, ALSConfiguration, menuI2cPullupValue);
                
                // Minimum time delay before reading any sensors after turning on the ALS
                OSA_TimeDelay(2.5);
                
                SEGGER_RTT_WriteString(0, "\nCompleted configuration for ALS.\n");
                
                disableI2Cpins();
                break;
            }
            case '1':
            {
                // Set gain parameter
                bool GainSet = false;
                while(GainSet == false){
                SEGGER_RTT_WriteString(0, "\r\n\tWhat is the gain:\n1 - x2\n2 - x1\n3 - x1/4\n4 - x1/8\n");
                uint16_t GainChoice = SEGGER_RTT_WaitKey();
                    switch (GainChoice){
                        case '1':{
                            GainSetting = 2;
                            GainConfig = 0b01;
                            GainSet = true;
                            break;
                        }
                        case '2':{
                            GainSetting = 1;
                            GainConfig = 0b00;
                            GainSet = true;
                            break;
                        }
                        case '3':{
                            GainSetting = 1/4;
                            GainConfig = 0b11;
                            GainSet = true;
                            break;
                        }
                        case '4':{
                            GainSetting = 1/8;
                            GainConfig = 0b10;
                            GainSet = true;
                            break;
                        }
                        default:
                        {
                            SEGGER_RTT_WriteString(0, "\r\n\tInvalid input. Try again.");
                        }
                    }
                }
                
                // Set integration time
                bool ITSet = false;
                while(ITSet != true){
                SEGGER_RTT_WriteString(0, "\r\n\tWhat is the Integration Time:\n1 - 800ms\n2 - 400ms\n3 - 200ms\n4 - 100ms\n5 - 50ms\n6 - 25ms\n");
                    uint16_t ITChoice = SEGGER_RTT_WaitKey();
                        switch (ITChoice){
                            case '1':{
                                IntegrationTimeMilliseconds = 800;
                                ITConfig = 0b0011;
                                ITSet = true;
                                break;
                            }
                            case '2':{
                                IntegrationTimeMilliseconds = 400;
                                ITConfig = 0b0010;
                                ITSet = true;
                                break;
                            }
                            case '3':{
                                IntegrationTimeMilliseconds = 200;
                                ITConfig = 0b0001;
                                ITSet = true;
                                break;
                            }
                            case '4':{
                                IntegrationTimeMilliseconds = 100;
                                ITConfig = 0b0000;
                                ITSet = true;
                                break;
                            }
                            case '5': {
                                IntegrationTimeMilliseconds = 50;
                                ITConfig = 0b1000;
                                ITSet = true;
                                break;
                            }
                            case '6':{
                                IntegrationTimeMilliseconds = 25;
                                ITConfig = 0b1100;
                                ITSet = true;
                                break;
                            }
                            default:
                            {
                                SEGGER_RTT_WriteString(0, "\r\n\tInvalid input. Try again.");
                            }
                        }
                    }
                
                enableI2Cpins(menuI2cPullupValue);
            
                uint16_t ALSConfiguration = (GainConfig << 12) | (ITConfig << 6);
                
                SEGGER_RTT_printf(0, "\r\n\tALSConfiguration is %d", ALSConfiguration);
            
                //THINK I NEED TO SEND ANOTHER BYTE FOR THE SLAVE ADDRESS
                
                // Write to config register
                writeSensorRegisterMultiplex(0b00000001); // Writes to first light sensor
                writeSensorRegisterVEML7700(0x00, ALSConfiguration, menuI2cPullupValue);
                
                SEGGER_RTT_WriteString(0,"\nFirst write successful");
                
                writeSensorRegisterMultiplex(0b00000010); // Writes to second
                
                writeSensorRegisterVEML7700(0x00, ALSConfiguration, menuI2cPullupValue);
                
                SEGGER_RTT_WriteString(0,"\nSecond write succesful");
                
                // Minimum time delay before reading any sensors after turning on the ALS
                OSA_TimeDelay(2.5);
                
                SEGGER_RTT_WriteString(0, "\nCompleted configuration for ALS.\n");
                
                disableI2Cpins();
                break;
            }
                
            case '2':
            {
                
                SEGGER_RTT_WriteString(0, "\r\n\tEnabling I2C pins...\n");
                enableI2Cpins(menuI2cPullupValue);
                
                SEGGER_RTT_printf(0, "\r\n\tSet the time delay between each run in milliseconds. You must do at least the IT time: %d ms ", IntegrationTimeMilliseconds);
                uint16_t    menuDelayBetweenEachRun = read4digits();
                SEGGER_RTT_printf(0, "\r\n\tDelay between read batches set to %d milliseconds.\n\n", menuDelayBetweenEachRun);
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
            
                SEGGER_RTT_WriteString(0, "\r\n\tSet the number of readings (e.g., '1234')> ");
                uint16_t    NumberOfReadings = read4digits();
                SEGGER_RTT_printf(0, "\r\n\tDelay between read batches set to %d milliseconds.\n\n", NumberOfReadings);
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                
                SEGGER_RTT_WriteString(0, "\nVEML7700 sensor data below:\n");
                
                uint16_t CurrentValue;
                
                for (int i; i < NumberOfReadings; i++){
                    
                    CurrentValue = printSensorDataVEML7700();
                    
                    if (CurrentValue == 0)
                    {
                        SEGGER_RTT_WriteString(0, "Read failed from the sensor. Exiting Loop.");
                        break;
                    }
                    
                    SEGGER_RTT_printf(0, " %d,", CurrentValue);
                    
                    OSA_TimeDelay(menuDelayBetweenEachRun);
            
                }
                disableI2Cpins();
                break;

            }
                
            case '3':
            {
                
                SEGGER_RTT_WriteString(0, "\r\n\tEnabling I2C pins...\n");
                enableI2Cpins(menuI2cPullupValue);
                
                SEGGER_RTT_printf(0, "\r\n\tSet the time delay between each run in milliseconds. You must do at least the IT time: %d ms ", IntegrationTimeMilliseconds);
                uint16_t    menuDelayBetweenEachRun = read4digits();
                SEGGER_RTT_printf(0, "\r\n\tDelay between read batches set to %d milliseconds.\n\n", menuDelayBetweenEachRun);
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
            
                SEGGER_RTT_WriteString(0, "\r\n\tSet the number of readings (e.g., '1234')> ");
                uint16_t    NumberOfReadings = read4digits();
                SEGGER_RTT_printf(0, "\r\n\tDelay between read batches set to %d milliseconds.\n\n", NumberOfReadings);
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                
                SEGGER_RTT_WriteString(0, "\nVEML7700 sensor data below:\n");
                
                uint16_t CurrentValue1;
                uint16_t CurrentValue2;
                
                for (int i; i < NumberOfReadings; i++){
                    
                    CurrentValue1 = printSensorDataMultiplex(false);
                    
                    CurrentValue2 = printSensorDataMultiplex(true);
                    
                    if (CurrentValue1 == 0)
                    {
                        SEGGER_RTT_WriteString(0, "Read failed from Sensor 1. Exiting Loop.");
                        break;
                    }
                    
                    if (CurrentValue2 == 0)
                    {
                        SEGGER_RTT_WriteString(0, "Read failed from Sensor 2. Exiting Loop.");
                        break;
                    }
                    
                    SEGGER_RTT_printf(0, "\n %d,", CurrentValue1);
                    SEGGER_RTT_printf(0, " %d,", CurrentValue2);
                    
                    OSA_TimeDelay(menuDelayBetweenEachRun);
            
                }
                disableI2Cpins();
                break;

            }
                
            case '4':
            {
                SEGGER_RTT_WriteString(0, "\r\n\tStarting motor..\n");
                
                enableI2Cpins(menuI2cPullupValue);
                
                bool init = initSG90();
            
                if (!init){
                    SEGGER_RTT_WriteString(0,"\nNot initiated.");
                    break;
                }
                
                uint32_t dutyPercentage = 7;
                
                tpm_pwm_param_t param = {
                        .mode              = kTpmEdgeAlignedPWM,
                        .edgeMode          = kTpmHighTrue,
                        .uFrequencyHZ      = 50u,
                        .uDutyCyclePercent = 7u
                };
                
                uint16_t sensorClockData;
                uint16_t sensorAntiData;
                uint16_t differenceClock;
                uint16_t sum;
                float percent;
                uint32_t percentage;
                
                while(1){
                    
                    
                    
//                    sensorClockData = printSensorDataMultiplex(false);
//                    sensorAntiData = printSensorDataMultiplex(true);
//
//                    differenceClock = abs(sensorClockData - sensorAntiData);
//                    SEGGER_RTT_printf(0,"\n Difference clockwise is: %d", differenceClock);
//                    sum = sensorClockData + sensorAntiData;
//                    SEGGER_RTT_printf(0,"\n Sum is: %d", sum);
//                    percent = ((float)differenceClock*100.0f)/(float)sensorClockData;
//                    SEGGER_RTT_printf(0,"\n percent is: %d", percent);
//
//                    //Change degrees to track light by a little bit
//
//                    if ( percent < 5.0f){}
//                    else if (sensorClockData > sensorAntiData){
//                        percentage +=1;
//                        if (percentage > 10){
//                            percentage = 10;
//                        }
//                    }
//                    else{
//                        percentage -= 1;
//                        if (percentage < 5){
//                            percentage = 5;
//                        }
//                    }
//
//                    SEGGER_RTT_printf(0,"\n percentage is: %d", percentage);
//
//                    param.uDutyCyclePercent = percentage;
//
//                    OSA_TimeDelay(1000);
                    
                }
                
                TPM_DRV_Deinit(0);
                break;
                
            }
                
            case '5':
            {
                
                //Initialise a
                
                tpm_general_config_t driverInfo;
                
                tpm_pwm_param_t param = {
                    .mode = kTpmEdgeAlignedPWM,
                    .edgeMode = kTpmHighTrue,
                    .uFrequencyHZ = 50,
                    .uDutyCyclePercent = 7
                };
                
                PORT_HAL_SetMuxMode(PORTB_BASE,11u,kPortMuxAlt2);
                
                CLOCK_SYS_SetConfiguration(&g_defaultClockConfigRun);
                
                memset(&driverInfo, 0, sizeof(driverInfo));
                
                TPM_DRV_Init(0,&driverInfo);
                
                TPM_DRV_SetClock(0, kTpmClockSourceModuleHighFreq, kTpmDividedBy2);
                while(1){
                    TPM_DRV_PwmStart(0,&param,0);
                    OSA_TimeDelay(1000);
                    param.uDutyCyclePercent += 1;
                    if (param.uDutyCyclePercent >= 100){
                        param.uDutyCyclePercent = 5;
                    }
                }
                break;
            }
        }
    }

    return 0;
}

static tpm_clock_mode_t s_tpmClockSource = kTpmClockSourceModuleHighFreq;

//bool initSG903(void){
//
//    tpm_general_config_t driverInfo;
//    tpm_pwm_param_t param = {
//            .mode              = kTpmEdgeAlignedPWM,
//            .edgeMode          = kTpmHighTrue,
//            .uFrequencyHZ      = 50u,
//            .uDutyCyclePercent = 7u
//    };
//
//    hardware_init();
//
//    memset(&driverInfo, 0, sizeof(driverInfo));
//
//    TPM_DRV_Init(BOARD_TPM_INSTANCE, &driverInfo);
//
//    TPM_DRV_SetClock(BOARD_TPM_INSTANCE, kTpmClockSourceModuleHighFreq, kTpmDividedBy2);
//
//    return true;
    
//
//}

bool initSG902(void){
    
    PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAlt2);
    
    tpm_clock_ps_t myclockPs = 1;
    bool status;

    tpm_general_config_t *info, config_struct;

    info = &config_struct;

    info->isDBGMode = false;
    info->isGlobalTimeBase = false; //Could try True here
    info->isTriggerMode = false;
    info->isStopCountOnOveflow = false;
    info->isCountReloadOnTrig = false;
    info->triggerSource = kTpmTpm0Trig;

    TPM_DRV_Init(0,info);

    TPM_DRV_SetClock(0,s_tpmClockSource,myclockPs);

    tpm_pwm_param_t *param, param_struct;

    param = &param_struct;

    param->mode = kTpmEdgeAlignedPWM;
    param->edgeMode = kTpmHighTrue;
    param->uFrequencyHZ = 50;
    param->uDutyCyclePercent = 7;

    status = TPM_DRV_PwmStart(0, param, 0);

    if (status == true){
        SEGGER_RTT_WriteString(0, "\n\t SG90 initialised.");
        return status;
    }

    else{
        SEGGER_RTT_WriteString(0, "\n\t SG90 initialisation failed.");
        return status;
    }
}

bool initTPM(void){

uint8_t instance = 0;

uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];

/*Enable TPM clock*/
CLOCK_SYS_EnableTpmClock(instance);

TPM_HAL_Reset(tpmBaseAddr, instance);

/*trigger mode*/
TPM_HAL_SetTriggerMode(tpmBaseAddr, false);
TPM_HAL_SetStopOnOverflowMode(tpmBaseAddr, false);
TPM_HAL_SetReloadOnTriggerMode(tpmBaseAddr, false);

/*trigger source*/
TPM_HAL_SetTriggerSrc(tpmBaseAddr, false);

/*global time base*/
TPM_HAL_EnableGlobalTimeBase(tpmBaseAddr, false);

/*Debug mode*/
TPM_HAL_SetDbgMode(tpmBaseAddr, false);

NVIC_ClearPendingIRQ(g_tpmIrqId[instance]);
INT_SYS_EnableIRQ(g_tpmIrqId[instance]);
    
    // Setting a clock
    
    TPM_HAL_SetClockDiv(tpmBaseAddr, 1);
    
    CLOCK_SYS_SetTpmSrc(instance, (clock_tpm_src_t)kTpmClockSourceModuleHighFreq);
    s_tpmClockSource = kTpmClockSourceModuleClk;


    if (s_tpmClockSource == kTpmClockSourceNoneClk)
    {
        SEGGER_RTT_WriteString(0, "\n No clock source.");
        return false;
    }
    
    return true;

}

bool startPWM(uint32_t uDutyCyclePercent){
    
    uint8_t instance = 0;
    uint8_t channel = 0;
    
    uint32_t freq;
    uint16_t uMod, uCnv;

    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];

    freq = TPM_DRV_GetClock(instance);

    /* When switching mode, disable channel first  */
    TPM_HAL_DisableChn(tpmBaseAddr, channel);

    /* Set the requested PWM mode */
    uint32_t val;
    val = ((uint32_t)(1)) << BP_TPM_CnSC_ELSA;
    val |= (2 << BP_TPM_CnSC_MSA);
    TPM_HAL_SetChnMsnbaElsnbaVal(tpmBaseAddr, channel, val);
    while (!(TPM_HAL_GetChnMsnbaVal(tpmBaseAddr, channel))) { }
    TPM_HAL_SetCpwms(tpmBaseAddr, 0);
    
    uMod = freq / 50 - 1;
    uCnv = uMod * uDutyCyclePercent / 100;
    /* For 100% duty cycle */
    if(uCnv >= uMod)
    {
        uCnv = uMod + 1;
    }
    TPM_HAL_SetMod(tpmBaseAddr, uMod);
    TPM_HAL_SetChnCountVal(tpmBaseAddr, channel, uCnv);

    /* Set the TPM clock */
    TPM_HAL_SetClockMode(tpmBaseAddr, s_tpmClockSource);

    return true;
}

bool initSG90(void){
    
    bool startStatus;
    
    //Initialise TPM drive
    
    PORT_HAL_SetMuxMode(PORTB_BASE, 11u, kPortMuxAlt2);
    
    startStatus = initTPM();
    
    if (startStatus == true){
        SEGGER_RTT_WriteString(0, "\n\t TPM initialised.");
    }
    
    else{
        SEGGER_RTT_WriteString(0, "\n\t TPM initialisation failed.");
    }
    
    tpm_general_config_t info = {
        false,
        false,
        false,
        false,
        false,
        0
    };

    tpm_general_config_t *tpm_config;

    tpm_config = &info;

    TPM_DRV_Init(0, (tpm_general_config_t *)tpm_config); // Pass 0 to initialise TPM0
    
     Initialise to 90
    
    tpm_pwm_param_t infom = {
        kTpmEdgeAlignedPWM,
        kTpmHighTrue,
        50,
        7
    };

    tpm_pwm_param_t *pwm_param;

    pwm_param = &infom;

    SEGGER_RTT_WriteString(0, "\nGot to here: 2");

    startStatus = TPM_DRV_PwmStart(0, (tpm_pwm_param_t *)pwm_param, 0);
    
    startStatus = startPWM(7);
    
    if (startStatus == true){
        SEGGER_RTT_WriteString(0, "\n\t SG90 initialised.");
        return startStatus;
    }
    
    else{
        SEGGER_RTT_WriteString(0, "\n\t SG90 initialisation failed.");
        return startStatus;
    }
}

uint32_t trackLight(uint32_t percentage, tpm_pwm_param_t *param){
    
    uint16_t sensorClockData;
    uint16_t sensorAntiData;
    
    sensorClockData = printSensorDataMultiplex(false);
    sensorAntiData = printSensorDataMultiplex(true);
    
    uint16_t differenceClock = abs(sensorClockData - sensorAntiData);
    SEGGER_RTT_printf(0,"\n Difference clockwise is: %d", differenceClock);
    uint16_t sum = sensorClockData + sensorAntiData;
    SEGGER_RTT_printf(0,"\n Sum is: %d", sum);
    float percent = ((float)differenceClock*100.0f)/(float)sensorClockData;
    SEGGER_RTT_printf(0,"\n percent is: %d", percent);
    
    //Change degrees to track light by a little bit
    
    if ( percent < 5.0f){}
    else if (sensorClockData > sensorAntiData){
        percentage +=1;
        if (percentage > 10){
            percentage = 10;
        }
    }
    else{
        percentage -= 1;
        if (percentage < 5){
            percentage = 5;
        }
    }

    SEGGER_RTT_printf(0,"\n percentage is: %d", percentage);
    
    param->uDutyCyclePercent = percentage;
    
    TPM_DRV_PwmStart(0, param, 0);
    
    return percentage;
    
}

void pwmPulse(float degrees){
    
    bool pulseStatus;
    float onTime = 1 + degrees/180;
    float offTime = 20 - onTime;
    
    uint32_t percentage = onTime/offTime*100;
    
    // Write to GPIO pin with the pulse
    
    tpm_pwm_param_t *new_param;
    
    tpm_pwm_param_t new_info = {
        kTpmEdgeAlignedPWM,
        kTpmHighTrue,
        50,
        percentage
    };
    
    new_param = &new_info;
    
    pulseStatus = TPM_DRV_PwmStart(0, new_param, 0);
}

void
printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, int i2cPullupValue)
{
    /*
     *    A 32-bit counter gives us > 2 years of before it wraps, even if sampling at 60fps
     */
    uint32_t    readingCount = 0;
    uint32_t    numberOfConfigErrors = 0;


    #ifdef WARP_BUILD_ENABLE_DEVAMG8834
    numberOfConfigErrors += configureSensorAMG8834(    0x3F,/* Initial reset */
                    0x01,/* Frame rate 1 FPS */
                    i2cPullupValue
                    );
    #endif
    #ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
    numberOfConfigErrors += configureSensorMMA8451Q(0x00,/* Payload: Disable FIFO */
                    0x01,/* Normal read 8bit, 800Hz, normal, active mode */
                    i2cPullupValue
                    );
    #endif
    #ifdef WARP_BUILD_ENABLE_DEVMAG3110
    numberOfConfigErrors += configureSensorMAG3110(    0x00,/*    Payload: DR 000, OS 00, 80Hz, ADC 1280, Full 16bit, standby mode to set up register*/
                    0xA0,/*    Payload: AUTO_MRST_EN enable, RAW value without offset */
                    i2cPullupValue
                    );
    #endif
    #ifdef WARP_BUILD_ENABLE_DEVL3GD20H
    numberOfConfigErrors += configureSensorL3GD20H(    0b11111111,/* ODR 800Hz, Cut-off 100Hz, see table 21, normal mode, x,y,z enable */
                    0b00100000,
                    0b00000000,/* normal mode, disable FIFO, disable high pass filter */
                    i2cPullupValue
                    );
    #endif
    #ifdef WARP_BUILD_ENABLE_DEVBME680
    numberOfConfigErrors += configureSensorBME680(    0b00000001,    /*    Humidity oversampling (OSRS) to 1x                */
                            0b00100100,    /*    Temperature oversample 1x, pressure overdsample 1x, mode 00    */
                            0b00001000,    /*    Turn off heater                            */
                            i2cPullupValue
                    );

    if (printHeadersAndCalibration)
    {
        SEGGER_RTT_WriteString(0, "\r\n\nBME680 Calibration Data: ");
        for (uint8_t i = 0; i < kWarpSizesBME680CalibrationValuesCount; i++)
        {
            #ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
            SEGGER_RTT_printf(0, "0x%02x", deviceBME680CalibrationValues[i]);
            if (i < kWarpSizesBME680CalibrationValuesCount - 1)
            {
                SEGGER_RTT_WriteString(0, ", ");
            }
            else
            {
                SEGGER_RTT_WriteString(0, "\n\n");
            }

            OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
            #endif
        }
    }
    #endif

    #ifdef WARP_BUILD_ENABLE_DEVHDC1000
    numberOfConfigErrors += writeSensorRegisterHDC1000(kWarpSensorConfigurationRegisterHDC1000Configuration,/* Configuration register    */
                    (0b1000000<<8),
                    i2cPullupValue
                    );
    #endif

    #ifdef WARP_BUILD_ENABLE_DEVCCS811
    uint8_t        payloadCCS811[1];
    payloadCCS811[0] = 0b01000000;/* Constant power, measurement every 250ms */
    numberOfConfigErrors += configureSensorCCS811(payloadCCS811,
                    i2cPullupValue
                    );
    #endif
    #ifdef WARP_BUILD_ENABLE_DEVBMX055
    numberOfConfigErrors += configureSensorBMX055accel(0b00000011,/* Payload:+-2g range */
                    0b10000000,/* Payload:unfiltered data, shadowing enabled */
                    i2cPullupValue
                    );
    numberOfConfigErrors += configureSensorBMX055mag(0b00000001,/* Payload:from suspend mode to sleep mode*/
                    0b00000001,/* Default 10Hz data rate, forced mode*/
                    i2cPullupValue
                    );
    numberOfConfigErrors += configureSensorBMX055gyro(0b00000100,/* +- 125degrees/s */
                    0b00000000,/* ODR 2000 Hz, unfiltered */
                    0b00000000,/* normal mode */
                    0b10000000,/* unfiltered data, shadowing enabled */
                    i2cPullupValue
                    );
    #endif


    if (printHeadersAndCalibration)
    {
        SEGGER_RTT_WriteString(0, "Measurement number, RTC->TSR, RTC->TPR,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

        #ifdef WARP_BUILD_ENABLE_DEVAMG8834
        for (uint8_t i = 0; i < 64; i++)
        {
            #ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
            SEGGER_RTT_printf(0, " AMG8834 %d,", i);
            OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
            #endif
        }
        SEGGER_RTT_WriteString(0, " AMG8834 Temp,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        #endif

        #ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
        SEGGER_RTT_WriteString(0, " MMA8451 x, MMA8451 y, MMA8451 z,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVMAG3110
        SEGGER_RTT_WriteString(0, " MAG3110 x, MAG3110 y, MAG3110 z, MAG3110 Temp,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVL3GD20H
        SEGGER_RTT_WriteString(0, " L3GD20H x, L3GD20H y, L3GD20H z, L3GD20H Temp,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVBME680
        SEGGER_RTT_WriteString(0, " BME680 Press, BME680 Temp, BME680 Hum,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVBMX055
        SEGGER_RTT_WriteString(0, " BMX055acc x, BMX055acc y, BMX055acc z, BMX055acc Temp,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, " BMX055mag x, BMX055mag y, BMX055mag z, BMX055mag RHALL,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, " BMX055gyro x, BMX055gyro y, BMX055gyro z,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVCCS811
        SEGGER_RTT_WriteString(0, " CCS811 ECO2, CCS811 TVOC, CCS811 RAW ADC value, CCS811 RAW R_REF value, CCS811 RAW R_NTC value,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVHDC1000
        SEGGER_RTT_WriteString(0, " HDC1000 Temp, HDC1000 Hum,");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        #endif
        SEGGER_RTT_WriteString(0, " RTC->TSR, RTC->TPR, # Config Errors");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\n\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
    }


    while(1)
    {
        #ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, "%u, %d, %d,", readingCount, RTC->TSR, RTC->TPR);
        #endif

        #ifdef WARP_BUILD_ENABLE_DEVAMG8834
        printSensorDataAMG8834(hexModeFlag);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
        printSensorDataMMA8451Q(hexModeFlag);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVMAG3110
        printSensorDataMAG3110(hexModeFlag);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVL3GD20H
        printSensorDataL3GD20H(hexModeFlag);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVBME680
        printSensorDataBME680(hexModeFlag);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVBMX055
        printSensorDataBMX055accel(hexModeFlag);
        printSensorDataBMX055mag(hexModeFlag);
        printSensorDataBMX055gyro(hexModeFlag);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVCCS811
        printSensorDataCCS811(hexModeFlag);
        #endif
        #ifdef WARP_BUILD_ENABLE_DEVHDC1000
        printSensorDataHDC1000(hexModeFlag);
        #endif


        #ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, " %d, %d, %d\n", RTC->TSR, RTC->TPR, numberOfConfigErrors);
        #endif

        if (menuDelayBetweenEachRun > 0)
        {
            OSA_TimeDelay(menuDelayBetweenEachRun);
        }

        readingCount++;
    }
}


void
loopForSensor(    const char *  tagString,
        WarpStatus  (* readSensorRegisterFunction)(uint8_t deviceRegister, int numberOfBytes),
        volatile WarpI2CDeviceState *  i2cDeviceState,
        volatile WarpSPIDeviceState *  spiDeviceState,
        uint8_t  baseAddress,
        uint8_t  minAddress,
        uint8_t  maxAddress,
        int  repetitionsPerAddress,
        int  chunkReadsPerAddress,
        int  spinDelay,
        bool  autoIncrement,
        uint16_t  sssupplyMillivolts,
        uint8_t  referenceByte,
        uint16_t adaptiveSssupplyMaxMillivolts,
        bool  chatty
        )
{
    WarpStatus        status;
    uint8_t            address;
    if((minAddress < baseAddress) || (baseAddress <= maxAddress))
    {
         address = baseAddress;
    }
    else
    {
        address = minAddress;
    }
    int            readCount = repetitionsPerAddress + 1;
    int            nSuccesses = 0;
    int            nFailures = 0;
    int            nCorrects = 0;
    int            nBadCommands = 0;
    uint16_t        actualSssupplyMillivolts = sssupplyMillivolts;


    if (    (!spiDeviceState && !i2cDeviceState) ||
        (spiDeviceState && i2cDeviceState) )
    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
#endif
    }

    enableSssupply(actualSssupplyMillivolts);
    SEGGER_RTT_WriteString(0, tagString);

    /*
     *    Keep on repeating until we are above the maxAddress, or just once if not autoIncrement-ing
     *    This is checked for at the tail end of the loop.
     */
    while (true)
    {
        for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
        {
            status = readSensorRegisterFunction(address+j, 1 /* numberOfBytes */);
            if (status == kWarpStatusOK)
            {
                nSuccesses++;
                if (actualSssupplyMillivolts > sssupplyMillivolts)
                {
                    actualSssupplyMillivolts -= 100;
                    enableSssupply(actualSssupplyMillivolts);
                }

                if (spiDeviceState)
                {
                    if (referenceByte == spiDeviceState->spiSinkBuffer[2])
                    {
                        nCorrects++;
                    }

                    if (chatty)
                    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                        SEGGER_RTT_printf(0, "\r\t0x%02x --> [0x%02x 0x%02x 0x%02x]\n",
                            address+j,
                            spiDeviceState->spiSinkBuffer[0],
                            spiDeviceState->spiSinkBuffer[1],
                            spiDeviceState->spiSinkBuffer[2]);
#endif
                    }
                }
                else
                {
                    if (referenceByte == i2cDeviceState->i2cBuffer[0])
                    {
                        nCorrects++;
                    }

                    if (chatty)
                    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                        SEGGER_RTT_printf(0, "\r\t0x%02x --> 0x%02x\n",
                            address+j,
                            i2cDeviceState->i2cBuffer[0]);
#endif
                    }
                }
            }
            else if (status == kWarpStatusDeviceCommunicationFailed)
            {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                SEGGER_RTT_printf(0, "\r\t0x%02x --> ----\n",
                    address+j);
#endif

                nFailures++;
                if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
                {
                    actualSssupplyMillivolts += 100;
                    enableSssupply(actualSssupplyMillivolts);
                }
            }
            else if (status == kWarpStatusBadDeviceCommand)
            {
                nBadCommands++;
            }

            if (spinDelay > 0)
            {
                OSA_TimeDelay(spinDelay);
            }
        }

        if (autoIncrement)
        {
            address++;
        }

        if (address > maxAddress || !autoIncrement)
        {
            /*
             *    We either iterated over all possible addresses, or were asked to do only
             *    one address anyway (i.e. don't increment), so we're done.
             */
            break;
        }
    }

    /*
     *    We intersperse RTT_printfs with forced delays to allow us to use small
     *    print buffers even in RUN mode.
     */
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
    SEGGER_RTT_printf(0, "\r\n\t%d/%d success rate.\n", nSuccesses, (nSuccesses + nFailures));
    OSA_TimeDelay(50);
    SEGGER_RTT_printf(0, "\r\t%d/%d successes matched ref. value of 0x%02x.\n", nCorrects, nSuccesses, referenceByte);
    OSA_TimeDelay(50);
    SEGGER_RTT_printf(0, "\r\t%d bad commands.\n\n", nBadCommands);
    OSA_TimeDelay(50);
#endif


    return;
}



void
repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress, uint8_t pullupValue, bool autoIncrement, int chunkReadsPerAddress, bool chatty, int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts, uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte)
{
    if (warpSensorDevice != kWarpSensorADXL362)
    {
        enableI2Cpins(pullupValue);
    }

    switch (warpSensorDevice)
    {
        case kWarpSensorADXL362:
        {
            /*
             *    ADXL362: VDD 1.6--3.5
             */
#ifdef WARP_BUILD_ENABLE_DEVADXL362
            loopForSensor(    "\r\nADXL362:\n\r",        /*    tagString            */
                    &readSensorRegisterADXL362,    /*    readSensorRegisterFunction    */
                    NULL,                /*    i2cDeviceState            */
                    &deviceADXL362State,        /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x2E,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tADXL362 Read Aborted. Device Disabled :(");
#endif
            break;
        }

        case kWarpSensorMMA8451Q:
        {
            /*
             *    MMA8451Q: VDD 1.95--3.6
             */
#ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
            loopForSensor(    "\r\nMMA8451Q:\n\r",        /*    tagString            */
                    &readSensorRegisterMMA8451Q,    /*    readSensorRegisterFunction    */
                    &deviceMMA8451QState,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x31,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tMMA8451Q Read Aborted. Device Disabled :(");
#endif
            break;
        }

        case kWarpSensorBME680:
        {
            /*
             *    BME680: VDD 1.7--3.6
             */
#ifdef WARP_BUILD_ENABLE_DEVBME680
            loopForSensor(    "\r\nBME680:\n\r",        /*    tagString            */
                    &readSensorRegisterBME680,    /*    readSensorRegisterFunction    */
                    &deviceBME680State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x1D,                /*    minAddress            */
                    0x75,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\nBME680 Read Aborted. Device Disabled :(");
#endif
            break;
        }

        case kWarpSensorBMX055accel:
        {
            /*
             *    BMX055accel: VDD 2.4V -- 3.6V
             */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
            loopForSensor(    "\r\nBMX055accel:\n\r",        /*    tagString            */
                    &readSensorRegisterBMX055accel,    /*    readSensorRegisterFunction    */
                    &deviceBMX055accelState,    /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x39,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tBMX055accel Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorBMX055gyro:
        {
            /*
             *    BMX055gyro: VDD 2.4V -- 3.6V
             */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
            loopForSensor(    "\r\nBMX055gyro:\n\r",        /*    tagString            */
                    &readSensorRegisterBMX055gyro,    /*    readSensorRegisterFunction    */
                    &deviceBMX055gyroState,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x39,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tBMX055gyro Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorBMX055mag:
        {
            /*
             *    BMX055mag: VDD 2.4V -- 3.6V
             */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
            loopForSensor(    "\r\nBMX055mag:\n\r",        /*    tagString            */
                    &readSensorRegisterBMX055mag,    /*    readSensorRegisterFunction    */
                    &deviceBMX055magState,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x40,                /*    minAddress            */
                    0x52,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\t BMX055mag Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorMAG3110:
        {
            /*
             *    MAG3110: VDD 1.95 -- 3.6
             */
#ifdef WARP_BUILD_ENABLE_DEVMAG3110
            loopForSensor(    "\r\nMAG3110:\n\r",        /*    tagString            */
                    &readSensorRegisterMAG3110,    /*    readSensorRegisterFunction    */
                    &deviceMAG3110State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x11,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tMAG3110 Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorL3GD20H:
        {
            /*
             *    L3GD20H: VDD 2.2V -- 3.6V
             */
#ifdef WARP_BUILD_ENABLE_DEVL3GD20H
            loopForSensor(    "\r\nL3GD20H:\n\r",        /*    tagString            */
                    &readSensorRegisterL3GD20H,    /*    readSensorRegisterFunction    */
                    &deviceL3GD20HState,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x0F,                /*    minAddress            */
                    0x39,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tL3GD20H Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorLPS25H:
        {
            /*
             *    LPS25H: VDD 1.7V -- 3.6V
             */
#ifdef WARP_BUILD_ENABLE_DEVLPS25H
            loopForSensor(    "\r\nLPS25H:\n\r",        /*    tagString            */
                    &readSensorRegisterLPS25H,    /*    readSensorRegisterFunction    */
                    &deviceLPS25HState,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x08,                /*    minAddress            */
                    0x24,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tLPS25H Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorTCS34725:
        {
            /*
             *    TCS34725: VDD 2.7V -- 3.3V
             */
#ifdef WARP_BUILD_ENABLE_DEVTCS34725
            loopForSensor(    "\r\nTCS34725:\n\r",        /*    tagString            */
                    &readSensorRegisterTCS34725,    /*    readSensorRegisterFunction    */
                    &deviceTCS34725State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x1D,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tTCS34725 Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorSI4705:
        {
            /*
             *    SI4705: VDD 2.7V -- 5.5V
             */
#ifdef WARP_BUILD_ENABLE_DEVSI4705
            loopForSensor(    "\r\nSI4705:\n\r",        /*    tagString            */
                    &readSensorRegisterSI4705,    /*    readSensorRegisterFunction    */
                    &deviceSI4705State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x09,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tSI4705 Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorHDC1000:
        {
            /*
             *    HDC1000: VDD 3V--5V
             */
#ifdef WARP_BUILD_ENABLE_DEVHDC1000
            loopForSensor(    "\r\nHDC1000:\n\r",        /*    tagString            */
                    &readSensorRegisterHDC1000,    /*    readSensorRegisterFunction    */
                    &deviceHDC1000State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x1F,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tHDC1000 Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorSI7021:
        {
            /*
             *    SI7021: VDD 1.9V -- 3.6V
             */
#ifdef WARP_BUILD_ENABLE_DEVSI7021
            loopForSensor(    "\r\nSI7021:\n\r",        /*    tagString            */
                    &readSensorRegisterSI7021,    /*    readSensorRegisterFunction    */
                    &deviceSI7021State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x09,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tSI7021 Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorCCS811:
        {
            /*
             *    CCS811: VDD 1.8V -- 3.6V
             */
#ifdef WARP_BUILD_ENABLE_DEVCCS811
            loopForSensor(    "\r\nCCS811:\n\r",        /*    tagString            */
                    &readSensorRegisterCCS811,    /*    readSensorRegisterFunction    */
                    &deviceCCS811State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0xFF,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tCCS811 Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorAMG8834:
        {
            /*
             *    AMG8834: VDD ?V -- ?V
             */
#ifdef WARP_BUILD_ENABLE_DEVAMG8834
            loopForSensor(    "\r\nAMG8834:\n\r",        /*    tagString            */
                    &readSensorRegisterAMG8834,    /*    readSensorRegisterFunction    */
                    &deviceAMG8834State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0xFF,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tAMG8834 Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorAS7262:
        {
            /*
             *    AS7262: VDD 2.7--3.6
             */
#ifdef WARP_BUILD_ENABLE_DEVAS7262
            loopForSensor(    "\r\nAS7262:\n\r",        /*    tagString            */
                    &readSensorRegisterAS7262,    /*    readSensorRegisterFunction    */
                    &deviceAS7262State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x2B,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tAS7262 Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        case kWarpSensorAS7263:
        {
            /*
             *    AS7263: VDD 2.7--3.6
             */
#ifdef WARP_BUILD_ENABLE_DEVAS7263
            loopForSensor(    "\r\nAS7263:\n\r",        /*    tagString            */
                    &readSensorRegisterAS7263,    /*    readSensorRegisterFunction    */
                    &deviceAS7263State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x2B,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tAS7263 Read Aborted. Device Disabled :( ");
#endif
            break;
        }

        default:
        {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
            SEGGER_RTT_printf(0, "\r\tInvalid warpSensorDevice [%d] passed to repeatRegisterReadForDeviceAndAddress.\n", warpSensorDevice);
#endif
        }
    }

    if (warpSensorDevice != kWarpSensorADXL362)
    {
        disableI2Cpins();
    }
}



int
char2int(int character)
{
    if (character >= '0' && character <= '9')
    {
        return character - '0';
    }

    if (character >= 'a' && character <= 'f')
    {
        return character - 'a' + 10;
    }

    if (character >= 'A' && character <= 'F')
    {
        return character - 'A' + 10;
    }

    return 0;
}



uint8_t
readHexByte(void)
{
    uint8_t        topNybble, bottomNybble;

    topNybble = SEGGER_RTT_WaitKey();
    bottomNybble = SEGGER_RTT_WaitKey();

    return (char2int(topNybble) << 4) + char2int(bottomNybble);
}



int
read4digits(void)
{
    uint8_t        digit1, digit2, digit3, digit4;

    digit1 = SEGGER_RTT_WaitKey();
    digit2 = SEGGER_RTT_WaitKey();
    digit3 = SEGGER_RTT_WaitKey();
    digit4 = SEGGER_RTT_WaitKey();

    return (digit1 - '0')*1000 + (digit2 - '0')*100 + (digit3 - '0')*10 + (digit4 - '0');
}

WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
    i2c_status_t    status;
    uint8_t        commandBuffer[1];
    uint8_t        payloadBuffer[1];
    i2c_device_t    i2cSlaveConfig =
            {
                .address = i2cAddress,
                .baudRate_kbps = gWarpI2cBaudRateKbps
            };

    commandBuffer[0] = commandByte;
    payloadBuffer[0] = payloadByte;

    status = I2C_DRV_MasterSendDataBlocking(
                        0    /* instance */,
                        &i2cSlaveConfig,
                        commandBuffer,
                        (sendCommandByte ? 1 : 0),
                        payloadBuffer,
                        (sendPayloadByte ? 1 : 0),
                        gWarpI2cTimeoutMilliseconds);

    return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}



WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength)
{
    uint8_t        inBuffer[payloadLength];
    spi_status_t    status;

    enableSPIpins();
    status = SPI_DRV_MasterTransferBlocking(0        /* master instance */,
                        NULL        /* spi_master_user_config_t */,
                        payloadBytes,
                        inBuffer,
                        payloadLength    /* transfer size */,
                        1000        /* timeout in microseconds (unlike I2C which is ms) */);
    disableSPIpins();

    return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}



void
powerupAllSensors(void)
{
    WarpStatus    status;

    /*
     *    BMX055mag
     *
     *    Write '1' to power control bit of register 0x4B. See page 134.
     */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
    status = writeByteToI2cDeviceRegister(    deviceBMX055magState.i2cAddress        /*    i2cAddress        */,
                        true                    /*    sendCommandByte        */,
                        0x4B                    /*    commandByte        */,
                        true                    /*    sendPayloadByte        */,
                        (1 << 0)                /*    payloadByte        */);
    if (status != kWarpStatusOK)
    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, "\r\tPowerup command failed, code=%d, for BMX055mag @ 0x%02x.\n", status, deviceBMX055magState.i2cAddress);
#endif
    }
    #else
    SEGGER_RTT_WriteString(0, "\r\tPowerup command failed. BMX055 disabled \n");
#endif
}



void
activateAllLowPowerSensorModes(bool verbose)
{
    WarpStatus    status;



    /*
     *    ADXL362:    See Power Control Register (Address: 0x2D, Reset: 0x00).
     *
     *    POR values are OK.
     */



    /*
     *    BMX055accel: At POR, device is in Normal mode. Move it to Deep Suspend mode.
     *
     *    Write '1' to deep suspend bit of register 0x11, and write '0' to suspend bit of register 0x11. See page 23.
     */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
    status = writeByteToI2cDeviceRegister(    deviceBMX055accelState.i2cAddress    /*    i2cAddress        */,
                        true                    /*    sendCommandByte        */,
                        0x11                    /*    commandByte        */,
                        true                    /*    sendPayloadByte        */,
                        (1 << 5)                /*    payloadByte        */);
    if ((status != kWarpStatusOK) && verbose)
    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, "\r\tPowerdown command failed, code=%d, for BMX055accel @ 0x%02x.\n", status, deviceBMX055accelState.i2cAddress);
#endif
    }
    #else
    SEGGER_RTT_WriteString(0, "\r\tPowerdown command abandoned. BMX055 disabled\n");
#endif

    /*
     *    BMX055gyro: At POR, device is in Normal mode. Move it to Deep Suspend mode.
     *
     *    Write '1' to deep suspend bit of register 0x11. See page 81.
     */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
    status = writeByteToI2cDeviceRegister(    deviceBMX055gyroState.i2cAddress    /*    i2cAddress        */,
                        true                    /*    sendCommandByte        */,
                        0x11                    /*    commandByte        */,
                        true                    /*    sendPayloadByte        */,
                        (1 << 5)                /*    payloadByte        */);
    if ((status != kWarpStatusOK) && verbose)
    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, "\r\tPowerdown command failed, code=%d, for BMX055gyro @ 0x%02x.\n", status, deviceBMX055gyroState.i2cAddress);
#endif
    }
    #else
    SEGGER_RTT_WriteString(0, "\r\tPowerdown command abandoned. BMX055 disabled\n");
#endif



    /*
     *    BMX055mag: At POR, device is in Suspend mode. See page 121.
     *
     *    POR state seems to be powered down.
     */



    /*
     *    MMA8451Q: See 0x2B: CTRL_REG2 System Control 2 Register (page 43).
     *
     *    POR state seems to be not too bad.
     */



    /*
     *    LPS25H: See Register CTRL_REG1, at address 0x20 (page 26).
     *
     *    POR state seems to be powered down.
     */



    /*
     *    MAG3110: See Register CTRL_REG1 at 0x10. (page 19).
     *
     *    POR state seems to be powered down.
     */



    /*
     *    HDC1000: currently can't turn it on (3V)
     */



    /*
     *    SI7021: Can't talk to it correctly yet.
     */



    /*
     *    L3GD20H: See CTRL1 at 0x20 (page 36).
     *
     *    POR state seems to be powered down.
     */
#ifdef WARP_BUILD_ENABLE_DEVL3GD20H
    status = writeByteToI2cDeviceRegister(    deviceL3GD20HState.i2cAddress    /*    i2cAddress        */,
                        true                /*    sendCommandByte        */,
                        0x20                /*    commandByte        */,
                        true                /*    sendPayloadByte        */,
                        0x00                /*    payloadByte        */);
    if ((status != kWarpStatusOK) && verbose)
    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, "\r\tPowerdown command failed, code=%d, for L3GD20H @ 0x%02x.\n", status, deviceL3GD20HState.i2cAddress);
#endif
    }
    #else
    SEGGER_RTT_WriteString(0, "\r\tPowerdown command abandoned. L3GD20H disabled\n");
#endif



    /*
     *    BME680: TODO
     */



    /*
     *    TCS34725: By default, is in the "start" state (see page 9).
     *
     *    Make it go to sleep state. See page 17, 18, and 19.
     */
#ifdef WARP_BUILD_ENABLE_DEVTCS34725
    status = writeByteToI2cDeviceRegister(    deviceTCS34725State.i2cAddress    /*    i2cAddress        */,
                        true                /*    sendCommandByte        */,
                        0x00                /*    commandByte        */,
                        true                /*    sendPayloadByte        */,
                        0x00                /*    payloadByte        */);
    if ((status != kWarpStatusOK) && verbose)
    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, "\r\tPowerdown command failed, code=%d, for TCS34725 @ 0x%02x.\n", status, deviceTCS34725State.i2cAddress);
#endif
    }
    #else
    SEGGER_RTT_WriteString(0, "\r\tPowerdown command abandoned. TCS34725 disabled\n");
#endif




    /*
     *    SI4705: Send a POWER_DOWN command (byte 0x17). See AN332 page 124 and page 132.
     *
     *    For now, simply hold its reset line low.
     */
#ifdef WARP_BUILD_ENABLE_DEVSI4705
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
#endif



    /*
     *    PAN1326.
     *
     *    For now, simply hold its reset line low.
     */
#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
#ifdef WARP_BUILD_ENABLE_DEVPAN1326
    GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);
#endif
#endif
}