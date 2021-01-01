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
#include "devMultiplex.h"

#define WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF

volatile WarpI2CDeviceState         deviceINA219State;

volatile WarpI2CDeviceState         deviceVEML7700State;

volatile WarpI2CDeviceState         deviceMultiplexState;

volatile i2c_master_state_t i2cMasterState;
volatile spi_master_state_t spiMasterState;
volatile spi_master_user_config_t spiUserConfig;

volatile uint32_t gWarpI2cBaudRateKbps = 200;
volatile uint32_t gWarpSpiBaudRateKbps = 200;
volatile uint32_t gWarpI2cTimeoutMilliseconds = 5;
volatile uint32_t gWarpSpiTimeoutMicroseconds = 5;
volatile uint32_t gWarpMenuPrintDelayMilliseconds    = 10;

// My Declarations

void initSG90(void);
void initSoftwarePWM(void);
void softwarePWM(float degreesOfRotation);
int read4digits(void);

void enableSPIpins(void)
{
    CLOCK_SYS_EnableSpiClock(0);

    // PTA8 -> MOSI
    PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);

    // PTA9 -> SCK
    PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

    /*
     *    Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
     *
     */
    uint32_t calculatedBaudRate;
    spiUserConfig.polarity = kSpiClockPolarity_ActiveHigh;
    spiUserConfig.phase = kSpiClockPhase_FirstEdge;
    spiUserConfig.direction = kSpiMsbFirst;
    spiUserConfig.bitsPerSec = gWarpSpiBaudRateKbps * 1000;
    SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
    SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}

void disableSPIpins(void)
{
    SPI_DRV_MasterDeinit(0);

    // PTA8 -> GPIO
    PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAsGpio);

    // PTA9 -> GPIO
    PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAsGpio);

//    GPIO_DRV_ClearPinOutput(kSSD1331PinMOSI);
//    GPIO_DRV_ClearPinOutput(kSSD1331PinSCK);

    CLOCK_SYS_DisableSpiClock(0);
}

void enableI2Cpins(uint8_t pullupValue)
{
    CLOCK_SYS_EnableI2cClock(0);

    /*    Warp KL03_I2C0_SCL    --> PTB3    (ALT2 == I2C)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

    /*    Warp KL03_I2C0_SDA    --> PTB4    (ALT2 == I2C)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

    I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}

void disableI2Cpins(void)
{
    I2C_DRV_MasterDeinit(0 /* I2C instance */);

    /*    Warp KL03_I2C0_SCL    --> PTB3    (GPIO)            */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

    /*    Warp KL03_I2C0_SDA    --> PTB4    (GPIO)            */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);

    /*
     *    Drive the I2C pins low
     */
//    GPIO_DRV_ClearPinOutput(kMAX30105PinI2C0_SDA);
//    GPIO_DRV_ClearPinOutput(kMAX30105PinI2C0_SCL);

    CLOCK_SYS_DisableI2cClock(0);
}

int main(void)
{
    uint8_t                    key;
    uint16_t menuI2cPullupValue = 32768;
    
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

    // Enable I2C
    enableI2Cpins(menuI2cPullupValue);
    
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

    // Initialise and configure all devices
    initINA219( 0x40 /* i2cAddress */, &deviceINA219State);
    initVEML7700(  0x10 /* i2cAddress */, &deviceVEML7700State);
    initMultiplex(0x70, &deviceMultiplexState);
    
    // Initialise variables
    
    uint16_t                    GainSetting;
    uint16_t                   IntegrationTimeMilliseconds;
    uint16_t                  GainConfig = 0;
    uint16_t                  ITConfig = 0;
    
    while (1)
    {
        SEGGER_RTT_WriteString(0, "\r\n\n\n\n[ *\t\t\t\tW\ta\tr\tp\t(rev. b)\t\t\t* ]\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

        SEGGER_RTT_WriteString(0, "\rSelect:\n");
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
        SEGGER_RTT_WriteString(0, "\r- '4': Track light with software PWM\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '5': PWM & SG90 testing\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '6': Track light with hardware PWM\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\rEnter selection> ");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        key = SEGGER_RTT_WaitKey();
        
        switch (key)
        {
            case 'n':
            {
                
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                
                // Write to config register
                writeSensorRegisterINA219(0x00, 0x019F, menuI2cPullupValue);
                
                // Write to calibration register
                writeSensorRegisterINA219(0x05, 0x2000, menuI2cPullupValue);

                
                SEGGER_RTT_WriteString(0, "\nCompleted configuration for current sensor.\n");
                
                SEGGER_RTT_WriteString(0, "INA219 sensor data below:\n");
                
                for (int i = 0; i < 1500; i++){
                    printSensorDataINA219();
                    OSA_TimeDelay(100);
                }
                
                
                break;

            }
            case '0':
            {
                // User sets gain parameter
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
                
                // User sets integration time
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
                
                // Place these in 16 bit command code
                
                uint16_t ALSConfiguration = (GainConfig << 12) | (ITConfig << 6);
                
                SEGGER_RTT_printf(0, "\r\n\tALSConfiguration is %d", ALSConfiguration);
                
                // Write command code to configuration register
                
                writeSensorRegisterVEML7700(0x00, ALSConfiguration, menuI2cPullupValue);
                
                // Minimum time delay before reading any sensors after turning on the ALS is 2.5ms
                
                OSA_TimeDelay(2.5);
                
                SEGGER_RTT_WriteString(0, "\nCompleted configuration for ALS.\n");
                
                break;
            }
            case '1':
            {
                // User sets gain parameter
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
                
                // User sets integration time
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
                
                // Create 16 bit command code with user options
                
                uint16_t ALSConfiguration = (GainConfig << 12) | (ITConfig << 6);
                
                SEGGER_RTT_printf(0, "\r\n\tALSConfiguration is %d", ALSConfiguration);
            
                
                // Write command code to config registers
                
                // First VEML7700
                
                writeSensorRegisterMultiplex(0b00000001);
                writeSensorRegisterVEML7700(0x00, ALSConfiguration, menuI2cPullupValue);
                
                SEGGER_RTT_WriteString(0,"\nFirst write successful");
                
                // Second VEML7700
                
                writeSensorRegisterMultiplex(0b00000010);
                writeSensorRegisterVEML7700(0x00, ALSConfiguration, menuI2cPullupValue);
                
                SEGGER_RTT_WriteString(0,"\nSecond write successful");
                
                // Minimum time delay before reading any sensors after turning on the ALS is 2.5ms
                OSA_TimeDelay(2.5);
                
                SEGGER_RTT_WriteString(0, "\nCompleted configuration for ALS.\n");
                
                break;
            }
                
            case '2':
            {
                
                // User inputs time delay between each run as well as how many readings
                
                SEGGER_RTT_WriteString(0, "\r\n\tEnabling I2C pins...\n");
                
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
                
                for (int i = 0; i < NumberOfReadings; i++){
                    
                    CurrentValue = printSensorDataVEML7700();
                    
                    if (CurrentValue == 0)
                    {
                        SEGGER_RTT_WriteString(0, "Read failed from the sensor. Exiting Loop.");
                        break;
                    }
                    
                    SEGGER_RTT_printf(0, " %d,", CurrentValue);
                    
                    OSA_TimeDelay(menuDelayBetweenEachRun);
            
                }
                break;

            }
                
            case '3':
            {
                
                // User inputs time delay between each run as well as how many readings
                
                SEGGER_RTT_WriteString(0, "\r\n\tEnabling I2C pins...\n");
                
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
                
                for (int i = 0; i < NumberOfReadings; i++){
                    
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
                break;

            }
                
            case '4':
            {
                SEGGER_RTT_WriteString(0, "\r\n\tStarting motor...\n");
                
                // Initialise the software PWM
                initSoftwarePWM();
                
                // Initialise parameters for loop
                float degreesOfRotation = 90;
                uint16_t sensor1data;
                uint16_t sensor2data;
                
                // While loop to track light
                while(1){
                    // Obtain sensor readings, shifted to account for sensor differences (see test results in report)
                    sensor1data = printSensorDataMultiplex(false);
                    sensor2data = printSensorDataMultiplex(true);
                    sensor2data = sensor2data + 300;
                    
                    //Activate actuator based upon readings
                    // Do not rotate if similar readings
                    if (abs(sensor1data-sensor2data) < 100){
                        // Software pwm the same
                        softwarePWM(degreesOfRotation);
                    }
                    // Else rotate toward sensor 1 if higher
                    else if (sensor1data > sensor2data){
                        degreesOfRotation += 5;
                        // Upper limit of 180 degrees of rotation
                        if (degreesOfRotation > 180){
                            degreesOfRotation = 180;
                        }
                        //Start software pwm
                        softwarePWM(degreesOfRotation);
                    }
                    // Else rotate toward sensor 2
                    else {
                        degreesOfRotation -= 5;
                        // Lower limit of 0 degrees of rotation
                        if (degreesOfRotation < 0){
                            degreesOfRotation = 0;
                        }
                        softwarePWM(degreesOfRotation);
                    }
                }
                
                break;
                
            }
                
            case '5':
            {
                SEGGER_RTT_WriteString(0, "\r\n\tStarting SG90 test...\n");
                // Initialise the SG90 and TPM pins
                initSG90();
                
                // Create a parameter structure for PWM
                tpm_pwm_param_t param = {
                    .mode = kTpmEdgeAlignedPWM,
                    .edgeMode = kTpmHighTrue,
                    .uFrequencyHZ = 50,
                    .uDutyCyclePercent = 5
                };
                
                // While loop to test motor, increases in rotation until max, then resets to 0 degrees (duty cycle 5%)
                
                while(1){
                    
                    bool test = TPM_DRV_PwmStart(0,&param,0);
                    if (!test){
                        SEGGER_RTT_WriteString(0,"\nFailed to start PWM.");
                    }
                    OSA_TimeDelay(1000);
                    param.uDutyCyclePercent += 1;
                    if (param.uDutyCyclePercent >= 100){
                        param.uDutyCyclePercent = 5;
                    }
                }
                break;
            }
            case '6':
            {
                // Initialise the SG90 and TPM pins
                initSG90();
                
                // Create a parameter structure for PWM
                tpm_pwm_param_t param = {
                    .mode = kTpmEdgeAlignedPWM,
                    .edgeMode = kTpmHighTrue,
                    .uFrequencyHZ = 50,
                    .uDutyCyclePercent = 5
                };
                
                // Initialise parameters for loop
                uint16_t sensor1data;
                uint16_t sensor2data;
                
                // While loop to track light and rotate solar panel
                
                while(1){
                    
                    // Set PWM on Channel 0 with parameters
                    TPM_DRV_PwmStart(0,&param,0);
                    
                    // Obtain sensor readings, shifted to account for sensor differences (see test results in report)
                    sensor1data = printSensorDataMultiplex(false);
                    sensor2data = printSensorDataMultiplex(true);
                    sensor2data = sensor2data + 1910;
                    
                    //Activate actuator based upon readings
                    // Do not rotate if simlar readings
                    if (abs(sensor1data-sensor2data) < 2000){
                        // Do nothing
                        OSA_TimeDelay(300);
                    }
                    // Else rotate toward sensor 1 if higher
                    else if (sensor1data > sensor2data){
                        param.uDutyCyclePercent += 1;
                        // Upper limit of 10% duty cycle
                        if (param.uDutyCyclePercent > 10){
                            param.uDutyCyclePercent = 10;
                        }
                        OSA_TimeDelay(300);
                    }
                    else {
                        param.uDutyCyclePercent -= 1;
                        // Lower limit of 5% duty cycle
                        if (param.uDutyCyclePercent < 5){
                            param.uDutyCyclePercent = 5;
                        }
                        OSA_TimeDelay(300);
                    }
                }
                break;
            }
        }
    }

    return 0;
}

// Function that initialises the TPM module for PWM to control the SG90

void initSG90(void){
    
    // Create config class for TPM
    tpm_general_config_t driverInfo;
    
    // Set PTB11 to Alt2 i.e. TPM0 Ch0
    PORT_HAL_SetMuxMode(PORTB_BASE,11u,kPortMuxAlt2);
    
    //CLOCK_SYS_SetConfiguration(&g_defaultClockConfigRun);
    
    memset(&driverInfo, 0, sizeof(driverInfo));
    
    // Initialise TPM driver
    TPM_DRV_Init(0,&driverInfo);
    
    //Set the clock to the high freq (48MHz) module divded by 2, so 24MHz
    TPM_DRV_SetClock(0, kTpmClockSourceModuleHighFreq, kTpmDividedBy2);
    }

// Function that initialises PWM in software

void initSoftwarePWM(void){
    
    // Set up PTB11 as GPIO
    PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);
    
    // Initiate a label for the pin
    //PWM_pin = GPIO_MAKE_PIN(HW_GPIOB,11);
    
    // Initialise at 90 degrees of rotation. I.e. 1.5ms out of 20ms
    GPIO_DRV_ClearPinOutput(PWM_pin);
    OSA_TimeDelay(10);
    for (int i; i<10;i++){
        
        GPIO_DRV_SetPinOutput(PWM_pin);
        OSA_TimeDelay(1.5);
        GPIO_DRV_ClearPinOutput(PWM_pin);
        OSA_TimeDelay(18.5);
        }
}

// Function that creates a PWM wave with a duty cycle corresponding to the given rotation. I.e. 1ms = 0 degrees and 2ms = 180 degrees at 50Hz

void softwarePWM(float degreesOfRotation){
    
    // Calculate on and off time based on input degrees
    
    float onTime = (1.0 + degreesOfRotation/180.0);
    float offTime = 20.0 - onTime;
    
    // Drive the pin low and high based on these calculations
    
    GPIO_DRV_SetPinOutput(PWM_pin);
    OSA_TimeDelay(onTime);
    GPIO_DRV_ClearPinOutput(PWM_pin);
    OSA_TimeDelay(offTime);
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

/*
 
 
 
 
 
 
 
 
 
 BELOW ARE THE FAILED ATTEMPTS TO GET PWM TO WORK.
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 */


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
