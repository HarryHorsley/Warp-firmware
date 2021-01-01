//
//  devVEML7700.h
// 
//
//  Created by Harry Horsley on 17/11/2020.
//

#ifndef WARP_BUILD_ENABLE_DEVVEML7700
#define WARP_BUILD_ENABLE_DEVVEML7700
#endif

void        initVEML7700(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);

WarpStatus    readSensorRegisterVEML7700(uint8_t deviceRegister, int numberOfBytes);
WarpStatus    writeSensorRegisterVEML7700(uint8_t deviceRegister,
                    uint16_t payload, uint16_t menuI2cPullupValue);
//WarpStatus    configureSensorINA219(uint8_t payload_SETUP1, uint8_t payload_SETUP2, uint8_t payload_CALIB1, uint8_t payload_CALIB2, uint16_t menuI2cPullupValue);
WarpStatus    readSensorSignalVEML7700(WarpTypeMask signal,
                    WarpSignalPrecision precision,
                    WarpSignalAccuracy accuracy,
                    WarpSignalReliability reliability,
                    WarpSignalNoise noise);
uint16_t       printSensorDataVEML7700();

