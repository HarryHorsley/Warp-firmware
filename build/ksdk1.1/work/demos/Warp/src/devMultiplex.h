//
//  devMultiplex.h
// 
//
//  Created by Harry Horsley on 17/11/2020.
//

#ifndef WARP_BUILD_ENABLE_DEVMultiplex
#define WARP_BUILD_ENABLE_DEVMultiplex
#endif

void        initMultiplex(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);

WarpStatus    writeSensorRegisterMultiplex(uint8_t deviceRegister);
uint16_t        printSensorDataMultiplex(bool sensorChoice);

