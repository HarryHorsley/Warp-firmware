//
//  devINA219.h
//  
//
//  Created by Harry Horsley on 17/11/2020.
//

#ifndef WARP_BUILD_ENABLE_DEVINA219
#define WARP_BUILD_ENABLE_DEVINA219
#endif

void        initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);

WarpStatus    readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus    writeSensorRegisterINA219(uint8_t deviceRegister,
                    uint16_t payloadBtye,
                    uint16_t menuI2cPullupValue);
WarpStatus    configureSensorINA219(uint16_t payload_SETUP, uint16_t payload_CALIB, uint16_t menuI2cPullupValue);
WarpStatus    readSensorSignalINA219(WarpTypeMask signal,
                    WarpSignalPrecision precision,
                    WarpSignalAccuracy accuracy,
                    WarpSignalReliability reliability,
                    WarpSignalNoise noise);
void        printSensorDataINA219(bool hexModeFlag);

