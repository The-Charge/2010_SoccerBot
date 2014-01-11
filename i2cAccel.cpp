#include "i2cAccel.h"
#include "DigitalModule.h"
#include "I2C.h"
#include "Utility.h"
//#include "WPIStatus.h"


i2cAccel::i2cAccel(UINT32 slot)
{
	DigitalModule *module = DigitalModule::GetInstance(slot);  //tells the cRIO which slot to look at (I think)
	m_i2c = module->GetI2C(kAddress);    //build the I2C class to handle comms to the accelerometer
	m_i2c->Write(kPowerControl, 0x08);   //wake up the chip
	m_i2c->Write(kFIFOControl, 0x9F);  //format chip on how to store readings
	m_i2c->Write(kDataFormat, 0x0A);    //begin taking readings
	m_i2c->Write(kDataOutputRate, 0x0F);  //set collection speed to uber fast
	UINT8 id;
	m_i2c->Read(kDevID, 1, &id);         //confirm im talking to device
	if (id == 0xE5)
		cout<<"ID Confirmed"<<endl;
	else
		cout<<"ID Check Failed"<<endl;
	
	//Accelerometer calibaration:
	bool zeroed = false;
	signed char adjust = 0x00;
	while (!zeroed){ 
		m_i2c->Write(kOffsetX, adjust);  //Until x and y axis are reading near zero, adjust offset
		double accel = getXAccel();
		if (accel >= -0.0156 && accel <= 0.0156)
			zeroed = true;
		else if (accel < -0.0156)
			adjust++;
		else
			adjust--;	
	}
	cout<<"X-Axis Calibrated!"<<endl;
	adjust = 0x00;
	zeroed = false;
	while (!zeroed){
		m_i2c->Write(kOffsetY, adjust);
		double accel = getYAccel();
		if (accel >= -0.0156 && accel <= 0.0156)
			zeroed = true;
		else if (accel < -0.0156)
			adjust++;
		else
			adjust--;	
	}
	cout<<"Y-Axis Calibrated!"<<endl;
	adjust = 0x00;
	zeroed = false;
	while (!zeroed){                
		m_i2c->Write(kOffsetZ, adjust);      //Until z axis is reading near one, adjust offset
		double accel = getZAccel();
		if (accel >= 0.9844 && accel <= 1.0156)
			zeroed = true;
		else if (accel < 0.9844)
			adjust++;
		else
			adjust--;	
	}
	cout<<"Z-Axis Calibrated!"<<endl;

	//double checks that all axes are well-calibrated
    if (abs((int)(getXAccel()*10000)) < 156 && abs((int)(getYAccel()*10000)) < 156 && abs((int)((getZAccel() - 1)*10000)) < 156)
    	cout<<"ALL AXES CALIBRATED SUCCESSFULLLY!";
    else
    	cout<<"POSSIBLE CALIBRATION ERROR!";
 
}

i2cAccel::~i2cAccel()   //idk, destructor
{
	delete m_i2c;
	m_i2c = NULL;
}
   //as per manufacturer recommendation, to avoid inconsistent or cross-timed data collections, all data registers
   //are read simultaneously on the accelerometer.
double i2cAccel::getXAccel()   // (hopefully) reads and returns the x-axis acceleration from the sensor
{
	INT16 xAccel[3];  //imma gonna redo this later, only need one get accel function
	m_i2c->Read(kData, 6, (UINT8 *)&xAccel);  //supposed to read all data registers at once
	xAccel[0] = ((xAccel[0] >> 8) & 0xFF) | (xAccel[0] << 8);  //flip bytes (I2C is weird like that)
	double scaled = xAccel[0] * 0.0039;  //scalar to put value read from accel into mg (Milli-g's)
	return scaled;      
}

double i2cAccel::getYAccel()   //  reads and returns the y-axis acceleration from the sensor
{
	INT16 yAccel[3];
	m_i2c->Read(kData, 6, (UINT8 *)&yAccel);
	yAccel[1] = ((yAccel[1] >> 8) & 0xFF) | (yAccel[1] << 8);
	double scaled = yAccel[1] * 0.0039;
	return scaled;
}

double i2cAccel::getZAccel()  //   reads and returns the z-axis acceleration from the sensor
{
	INT16 zAccel[3];
	m_i2c->Read(kData, 6, (UINT8 *)&zAccel);
	zAccel[2] = ((zAccel[2] >> 8) & 0xFF) | (zAccel[2] << 8);
	double scaled = zAccel[2] * 0.0039;
	return scaled;
}

double i2cAccel::getAngleOfElevation(double zAccel)  //simple trig to solve for angle of elevation
{
	double ang = acos(zAccel);
	return ang * 180 / PI;
}

double i2cAccel::getAngleOfElevationBetter(double yAccel){  //alternate trig to solve for elevation
	double ang = asin(yAccel);
	return ang * 180 / PI;
}


/* This one can also tell which way the robot is elevated
 * B = "BACK"
 * L = "LEVEL"
 * F = "FORWARD"
 * could easily be changed to find left/right tips too
 */
double i2cAccel::getAngleOfElevation(double &zAccel, double &xAccel, char &tip)  //not used, could be later
{
	double ang = acos(zAccel);
	if (xAccel < -0.2)
		tip = 'B';
	else if (xAccel < 0.2)
		tip = 'L';
	else
		tip = 'F';
	return ang * 180 / PI;
}
