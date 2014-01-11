#ifndef __i2cAccel_h__
#define __i2cAccel_h__

#include "WPILib.h"
#include "Math.h"
const static long double PI = 3.1415926535;
class I2C;

class i2cAccel : SensorBase
{
public:
	explicit i2cAccel(UINT32 slot);  //constructor
	virtual ~i2cAccel();   //destructor
	double getXAccel();    //fetch x accel val from chip
	double getYAccel();    // ""
	double getZAccel();    // ""
	double getAngleOfElevation(double zAccel);  // acos to find angle of tip
	double getAngleOfElevation(double &zAccel, double &xAccel, char &tip);
	double getAngleOfElevationBetter(double yAccel);  //(better?) way to get angle
	I2C *m_i2c;   //I2C connection
private:
	//device and register addresses used to read and write to accelerometer
	static const UINT8 kAddress = 0x3A;  //device address on the bus
	static const UINT8 kDevID = 0x00;    //device ID for confirming correct device
	static const UINT8 kDataFormat = 0x31;  //reg address for Data Format Control Settings
	static const UINT8 kDataOutputRate = 0x2C;  //reg address for Data Output Rate Settings
	static const UINT8 kPowerControl = 0x2D;  //reg address for Power Control Settings
	static const UINT8 kFIFOControl = 0x38;   //reg address for FIFO Control Settings
	static const UINT8 kData = 0x32;    //reg address for x-axis accel
	static const UINT8 kOffsetX = 0x1E;  //reg addresses for axes offsets
	static const UINT8 kOffsetY = 0x1F;
	static const UINT8 kOffsetZ = 0x20;
};

#endif
