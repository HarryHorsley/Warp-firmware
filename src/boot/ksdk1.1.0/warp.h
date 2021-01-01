#include "fsl_spi_master_driver.h"

#define    min(x,y)    ((x) < (y) ? (x) : (y))
#define    USED(x)        (void)(x)

typedef enum
{
    kWarpTypeMaskTemperature    = (1 <<  0),
    kWarpTypeMaskPressure        = (1 <<  1),
    kWarpTypeMaskHumidity        = (1 <<  2),
    kWarpTypeMaskC02Concentration    = (1 <<  3),

    kWarpTypeMaskInfrared        = (1 <<  4),
    kWarpTypeMaskColor        = (1 <<  5),

    kWarpTypeMaskAccelerationX    = (1 <<  6),
    kWarpTypeMaskAccelerationY    = (1 <<  7),
    kWarpTypeMaskAccelerationZ    = (1 <<  8),

    kWarpTypeMaskAngularRateX    = (1 <<  9),
    kWarpTypeMaskAngularRateY    = (1 << 10),
    kWarpTypeMaskAngularRateZ    = (1 << 11),

    kWarpTypeMaskMagneticX        = (1 << 12),
    kWarpTypeMaskMagneticY        = (1 << 13),
    kWarpTypeMaskMagneticZ        = (1 << 14),

    kWarpTypeMaskFMStationID    = (1 << 15),

    kWarpTypeMaskLambda450V        = (1 << 16),
    kWarpTypeMaskLambda500B        = (1 << 17),
    kWarpTypeMaskLambda550G        = (1 << 18),
    kWarpTypeMaskLambda570Y        = (1 << 19),
    kWarpTypeMaskLambda600O        = (1 << 20),
    kWarpTypeMaskLambda650R        = (1 << 21),
    
    kWarpTypeMaskShunt             = (1 << 22),
    kWarpTypeMaskBusVoltage     = (1 << 23),
    kWarpTypeMaskPower          = (1 << 24),
    kWarpTypeMaskCurrent        = (1 << 25),
    
    kWarpTypeMaskALSOut         = (1 << 26),


    /*
     *    Always keep these two as the last items.
     */
    kWarpTypeMaskTime,
    kWarpTypeMaskMax,
} WarpTypeMask;

typedef enum
{
    /*
     *    Always keep this as the last item.
     */
    kWarpSignalPrecisionMax
} WarpSignalPrecision;


typedef enum
{
    /*
     *    Always keep this as the last item.
     */
    kWarpSignalAccuracyMax
} WarpSignalAccuracy;

typedef enum
{
    /*
     *    Always keep this as the last item.
     */
    kWarpSignalReliabilityMax
} WarpSignalReliability;

typedef enum
{
    /*
     *    Always keep this as the last item.
     */
    kWarpSignalNoiseMax
} WarpSignalNoise;

typedef enum
{
    kWarpStatusOK                = 0,

    kWarpStatusDeviceNotInitialized,
    kWarpStatusDeviceCommunicationFailed,
    kWarpStatusBadDeviceCommand,

    /*
     *    Generic comms error
     */
    kWarpStatusCommsError,

    /*
     *    Power mode routines
     */
    kWarpStatusPowerTransitionErrorVlpr2Wait,
    kWarpStatusPowerTransitionErrorVlpr2Stop,
    kWarpStatusPowerTransitionErrorRun2Vlpw,
    kWarpStatusPowerTransitionErrorVlpr2Vlpr,
    kWarpStatusErrorPowerSysSetmode,
    kWarpStatusBadPowerModeSpecified,


    /*
     *    Always keep this as the last item.
     */
    kWarpStatusMax
} WarpStatus;

typedef enum
{
    /*
     *    NOTE: This order is depended on by POWER_SYS_SetMode()
     *
     *    See KSDK13APIRM.pdf Section 55.5.3
     */
    kWarpPowerModeWAIT,
    kWarpPowerModeSTOP,
    kWarpPowerModeVLPR,
    kWarpPowerModeVLPW,
    kWarpPowerModeVLPS,
    kWarpPowerModeVLLS0,
    kWarpPowerModeVLLS1,
    kWarpPowerModeVLLS3,
    kWarpPowerModeRUN,
} WarpPowerMode;

typedef enum
{
    kWarpSensorADXL362,
    kWarpSensorMMA8451Q,
    kWarpSensorINA219,
    kWarpSensorVEML7700,
    kWarpSensorBME680,
    kWarpSensorBMX055accel,
    kWarpSensorBMX055gyro,
    kWarpSensorBMX055mag,
    kWarpSensorTMP006B,
    kWarpSensorMAG3110,
    kWarpSensorL3GD20H,
    kWarpSensorLPS25H,
    kWarpSensorTCS34725,
    kWarpSensorSI4705,
    kWarpSensorHDC1000,
    kWarpSensorSI7021,
    kWarpSensorAMG8834,
    kWarpSensorCCS811,
    kWarpSensorPAN1326,
    kWarpSensorAS7262,
    kWarpSensorAS7263,
    kWarpSensorSCD30,
} WarpSensorDevice;

typedef enum
{
    kWarpModeDisableAdcOnSleep    = (1 << 0),
} WarpModeMask;


typedef enum
{
    kWarpSizesI2cBufferBytes        = 4,
    kWarpSizesSpiBufferBytes        = 4, /* Was 3 bytes */
    kWarpSizesBME680CalibrationValuesCount    = 41,
} WarpSizes;

typedef struct
{
    uint8_t            i2cAddress;
    WarpTypeMask        signalType;
    uint8_t            i2cBuffer[kWarpSizesI2cBufferBytes];

    WarpStatus        deviceStatus;
} WarpI2CDeviceState;

typedef enum
{
    kWarpSensorConfigurationRegisterMMA8451QF_SETUP            = 0x09,
    kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1        = 0x2A,
    
    kWarpSensorConfigurationRegisterINA219Config            = 0x00,
    kWarpSensorConfigurationRegisterINA219Calibration       = 0x05,

    kWarpSensorConfigurationRegisterVEML7700Config          = 0x00,
    kWarpSensorConfigurationRegisterVEML7700HighThresh      = 0x01,
    kWarpSensorConfigurationRegisterVEML7700LowThresh       = 0x02,
    kWarpSensorConfigurationRegisterVEML7700PSM             = 0x03,
    
} WarpSensorConfigurationRegister;

typedef enum
{
    kWarpSensorOutputRegisterMMA8451QOUT_X_MSB            = 0x01,
    kWarpSensorOutputRegisterMMA8451QOUT_X_LSB            = 0x02,
    kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB            = 0x03,
    kWarpSensorOutputRegisterMMA8451QOUT_Y_LSB            = 0x04,
    kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB            = 0x05,
    kWarpSensorOutputRegisterMMA8451QOUT_Z_LSB            = 0x06,
    
    kWarpSensorOutputRegisterINA219ShuntVoltage         = 0x01,
    kWarpSensorOutputRegisterINA219BusVoltage           = 0x02,
    kWarpSensorOutputRegisterINA219Power                = 0x03,
    kWarpSensorOutputRegisterINA219Current              = 0x04,

    kWarpSensorOutputRegisterVEML7700ALSOut             = 0x04,
    kWarpSensorOutputRegisterVEML7700WhiteOut           = 0x05,
    kWarpSensorOutputRegisterVEML7700ALSTrigger         = 0x06,
    
} WarpSensorOutputRegister;

typedef struct
{
    /*
     *    For holding ksdk-based error codes
     */
    spi_status_t        ksdk_spi_status;

    WarpTypeMask        signalType;

    uint8_t            spiSourceBuffer[kWarpSizesSpiBufferBytes];
    uint8_t            spiSinkBuffer[kWarpSizesSpiBufferBytes];
    WarpStatus        deviceStatus;
} WarpSPIDeviceState;

typedef struct
{
    WarpTypeMask        signalType;
    WarpStatus        deviceStatus;
} WarpUARTDeviceState;

typedef struct
{
    uint8_t    errorCount;
} WarpPowerManagerCallbackStructure;

typedef enum
{
    kWarpThermalChamberMemoryFillEvenComponent    = 0b00110011,
    kWarpThermalChamberMemoryFillOddComponent    = 0b11001100,
    kWarpThermalChamberMMA8451QOutputBufferSize    = 3,
    kWarpThermalChamberKL03MemoryFillBufferSize    = 200,
    kWarpThermalChamberBusyLoopCountOffset        = 65535,
    kWarpThermalChamberBusyLoopAdder        = 99,
    kWarpThermalChamberBusyLoopMutiplier        = 254,
} WarpThermalChamber;

typedef struct
{
    /*
     *    Fill up the remaining memory space using an array
     *    The size of the array is highly dependent on
     *    the firmware code size
     */
    uint8_t        memoryFillingBuffer[kWarpThermalChamberKL03MemoryFillBufferSize];
    uint8_t        outputBuffer[kWarpThermalChamberMMA8451QOutputBufferSize];
} WarpThermalChamberKL03MemoryFill;

WarpStatus    warpSetLowPowerMode(WarpPowerMode powerMode, uint32_t sleepSeconds);
void        enableI2Cpins(uint8_t pullupValue);
void        disableI2Cpins(void);
void        enableSPIpins(void);
void        disableSPIpins(void);
