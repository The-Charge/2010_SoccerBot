#include "WPILib.h"
#include "360Object.h"
///#include "DashboardDataSender.h"
#include "i2cAccel.h"


#include "Target.h"
#include "Math.h"
#include "PIDController.h"


/* Summary of our class hierarchy:
 *	@depreciated: not really updated, lol
 * 
 * MainRobotClass has-a:
 * 	-Zoidberg
 * 	-Joystick
 *  -Two 360 Controllers
 * 
 * Zoidberg has-a:  (all publicly accessible)
 * 	-Gyro
 * 	-Accelerometer
 * 	-Camera
 * 	-SoccerBot (behaviors get sent MainRobotClass's Joystick)
 * 	-Methods:
 * 		-Lift()
 *		-CameraStuff
 * 
 * SoccerBot has behaviors and members for:
 * 	-Encoders
 *	-Servos
 *	-Timers
 * 	-Motor Controllers
 *	-One 360 Controller
 *  -Methods:
 * 		-TankDrive(...)
 * 		-ArcadeDrive(...)
 * 		-DriveXFeet(...)
 * 		-SpinXDegrees(...)
 * 		-SetLeftRightMotorSpeeds(...)
 *		-TestShooter(...)
 *		-TestTensioner(...)
 *		-Shoot() //BAD!
 *		-ReadRelativeEncoder()
 *		-ReadAbsoluteEncoder()
 *		-IsCocked()
 *		-IsUncocked()
 * 
 */

//global tasks
Task *autoshoot;
///DashboardDataSender *m_Dashboard;//allows dashboard to be updated ANYWHERE
Task *detect;//allows task EVERYWHERE

//global variables for the camera
bool pidena=false;
double servocen;
static SEM_ID horizontal_angle_sem;//semiphore for horizontal angle
static SEM_ID rangesem;
float HoriAng;//horizontal angle wrapper
//LEAVE THIS ALONE
double outputValue;
float range;

//global constants for PWM Output Channels
const int PWMOUT_RIGHT_WHEELS = 1;
const int PWMOUT_LEFT_WHEELS = 2;
const int PWMOUT_SHOOTER = 3;
const int PWMOUT_TENSIONER = 4;
const int PWMOUT_CAMERA_PAN = 9;
const int PWMOUT_CAMERA_TILT = 10;
const int PWMOUT_LIFTER = 5;
const int PWMOUT_LIFTER_RELEASE = 8;

//global constants for Digital Input Channels
const int DIGIN_RIGHT_WHEEL_ENCODER_A = 2;
const int DIGIN_RIGHT_WHEEL_ENCODER_B = 1;
const int DIGIN_LEFT_WHEEL_ENCODER_A = 3;
const int DIGIN_LEFT_WHEEL_ENCODER_B = 4;

//global constants for Analog Input Channels
const int ANIN_GYRO = 1;
const int ANIN_ABSOLUTE_ENCODER_SHOOTER = 3;
const int ANIN_ABSOLUTE_ENCODER_TENSIONER = 2;
const int ANIN_PHOTO_BALL_SENSOR = 7;

//global directional constants
const int FORWARD = 1;
const int BACKWARD = -1;
const int LEFT = -1;
const int RIGHT = 1;

//Calibration values
//ENCODER_TO_FEET_RATIO is in encoderclicks/foot, multiply by encoder->Get() to determine feet traveled
const double ENCODER_TO_FEET_RATIO = .00415304;
const double DRIVEXFEET_ERROR = 0.1;
const double ROTATEXDEGREES_ERROR = 0.0;
//left .5 right .8
//left and right boundaries for cock point and tensioner and hooker
//.6945-forward
		//.8385
const double LEFT_BOUND_COCK = .7;
const double RIGHT_BOUND_COCK = 1;
const double LEFT_BOUND_TENSION = 2.77243;//low tension
const double RIGHT_BOUND_TENSION = 1.56366;
const int LEFT_BOUND_HOOKER = 50; //latches the hooker
const int RIGHT_BOUND_HOOKER = 140; //releases her

//Cyprus channels!
//@depreciated: chkcyp doesn't use these constants... yet.
const int CYP_ZONE_1 = 1;
const int CYP_ZONE_2 = 2;
const int CYP_ZONE_3 = 3;
const int CYP_DUKES = 4;
const int CYP_GTFO = 4;
const int CYP_BUMPY = 5;
const int CYP_469A = 6;
const int CYP_469B = 7;
const int CYP_469C = 8;
const int CYP_WAIT = 1;
const int CYP_BUTTON_THAT_BLOWS_UP_THE_EARTH = 5;

bool autoshoot_task_started;

/*
 * cypress demo class
 * @depreciated: ummmm whaaa?
 */
class Cypress
{
public:
	DriverStation *m_ds;
	bool enhanced;
	/*pass it a true value if you need to use inhanced io, ie if you have more than 4 analog ins or have rewired the cypress
	 */
	
	Cypress::Cypress(bool enhan)
	{
		m_ds=DriverStation::GetInstance();
		enhanced=enhan;
	}
	
	/*
	 * labeling is reversed so it should be more logical
	 */
	bool Cypress::GetDigitalIn(int channel)
	{
		int getchan=abs(channel-8)+1;
		
		if(enhanced)
			return m_ds->GetEnhancedIO().GetDigital(getchan);
		else
			return m_ds->GetDigitalIn(getchan);
			
	}
	/*
	 * labeling is reversed to match digital ins
	 */
	void Cypress::SetDigitalOut(int channel,bool value)
	{
		int getchan=abs(channel-8)+1;
		if(enhanced)
		{
			
			getchan+=8;
			m_ds->GetEnhancedIO().SetDigitalOutput(getchan,value);
		}
		else
			m_ds->SetDigitalOut(getchan,value);
		
	}
	/*
	 * labeling is reversed to match digitals
	 */
	float Cypress::GetAnalogIn(int channel)
	{
		int getchan=abs(channel-4)+1;
		if(enhanced)
			return m_ds->GetEnhancedIO().GetAnalogIn(getchan);
		else
			return m_ds->GetAnalogIn(getchan);
		
		
	}
};

/*
 * ABSOLUTE ENCODER CLASS
 */
class AbsoluteEncoder
{
public:
	float vpd;
	AnalogChannel *enc;

	//Constructor
	AbsoluteEncoder::AbsoluteEncoder(int channel)
	{
		vpd=0.0138111111; //default volts per degree
		enc=new AnalogChannel(channel);
	}
	
	
	//Constructor
	AbsoluteEncoder::AbsoluteEncoder(int channel, int slot)
	{
		enc=new AnalogChannel(slot,channel);
	}
	
	
	//@return: an angle from 0 to 360 degrees
	//@depreciated: Not sure if this has been tested
	float AbsoluteEncoder::GetAngle()
	{
		return enc->GetVoltage()/vpd;
	}
	
	
	//@return: the raw voltage of the absolute encoder
	float AbsoluteEncoder::GetRawVoltage()
	{
		return enc->GetVoltage();
	}
	
	
	//Sets the volts per degree value of the absolute encoder (probably remains at default value)
	void AbsoluteEncoder::SetVPD(float _vpd)
	{
		vpd = _vpd;
	}
};

/*
 * DRIVE CLASS
 */
class SoccerBot
{
public:
	//Members:
	SpeedController *m_leftWheels, *m_rightWheels;
	SpeedController *m_shooter, *m_tensioner, *m_lifter;
	Encoder *m_leftEncoder, *m_rightEncoder;
	AbsoluteEncoder *m_shooterAbsoluteEncoder, *m_tensionerAbsoluteEncoder;
	Servo *m_panServ, *m_tiltServ, *m_hookerServ;
	Object360 *m_Control; //needed for overrides and (dont tell murray) break conditions
	Timer *m_shotTimer, *m_hookerTimer;

	/* SoccerBot() Default Constructor 
	 *
	 * Uses the global constants for inputs/outputs to instantiate objects
	 * Starts/Resets Gyros and Encoders and Timers
	 * Sets Motor values to 0
	 * 
	 */
	SoccerBot::SoccerBot(Object360 *m_ControllerDrive)
	{
		//Instantiation!

		//Instantiate 360 Controller
		m_Control = m_ControllerDrive;
		
		//Instantiate Motor Controllers
		
		m_rightWheels = new Jaguar(1,PWMOUT_RIGHT_WHEELS);
		m_leftWheels = new Jaguar(1,PWMOUT_LEFT_WHEELS);
		m_shooter = new Jaguar(1,PWMOUT_SHOOTER);
		m_tensioner = new Jaguar(1,PWMOUT_TENSIONER);
		m_lifter = new Jaguar(1,PWMOUT_LIFTER);
		
		//Instantiate Encoders
		
		m_rightEncoder = new Encoder(1,DIGIN_RIGHT_WHEEL_ENCODER_A, 1,DIGIN_RIGHT_WHEEL_ENCODER_B);
		m_leftEncoder = new Encoder(1,DIGIN_LEFT_WHEEL_ENCODER_A,1, DIGIN_LEFT_WHEEL_ENCODER_B);
		m_shooterAbsoluteEncoder = new AbsoluteEncoder(ANIN_ABSOLUTE_ENCODER_SHOOTER);
		m_tensionerAbsoluteEncoder = new AbsoluteEncoder(ANIN_ABSOLUTE_ENCODER_TENSIONER);
		
		//Instantiate Servos
		
		m_panServ = new Servo(1,PWMOUT_CAMERA_PAN); //@depreciated: unpluged/not used
		m_tiltServ = new Servo(1,PWMOUT_CAMERA_TILT); //@depreciated: unpluged/not used
		m_hookerServ = new Servo(1,PWMOUT_LIFTER_RELEASE);
		
		//Set Initial values
		cout << "Setting initial SoccerBot values" << endl;
		
		//Set Motor Default Values
		m_rightWheels->Set(0);
		m_leftWheels->Set(0);
		m_shooter->Set(0);
		m_tensioner->Set(0);
		
		//Reset and start encoders/gyro
		m_rightEncoder->Start();
		m_rightEncoder->Reset();
		m_leftEncoder->Start();
		m_leftEncoder->Reset();
		
		//Reset and start timers
		m_shotTimer = new Timer();
		m_hookerTimer = new Timer();
		m_shotTimer->Start();
		m_hookerTimer->Start();
		m_shotTimer->Reset();
		m_hookerTimer->Reset();
	}

	/* void ArcadeDrive(float y, float x)
	 *
	 * Drives the robot with arcade drive (usefull for autoaim)
	 * @depreciated: not used (sorry matt z!)
	 *
	 * @param y: the value of the y  - not the  itself, send m_controllerDrive->GetLeftY()
	 * @param x: the value of the x 
	 * 
	 */
	void SoccerBot::ArcadeDrive(float y, float x)
	{
		//create floats
		float moveValue = y;
		float rotateValue = x;
		float leftMotorSpeed, rightMotorSpeed;
				
		//calculate left and right motor speeds
		if (moveValue > 0.0)
		{
			if (rotateValue > 0.0)
			{
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = max(moveValue, rotateValue);
			} 
			else
			{
				leftMotorSpeed = max(moveValue, -rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
		}

		else
		{
			if (rotateValue > 0.0)
			{
				leftMotorSpeed = -max(-moveValue, rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			} 
			else
			{
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = -max(-moveValue, -rotateValue);
			}
		}
		
		//need to invert left motor
		leftMotorSpeed*=-1;
		
		//set motor speeds		
		SetLeftRightMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
	}
		
	/* void TankDrive(float leftValue, float rightValue)
	 * 
	 * This function drives the robot like tank drive... self explanatory much?
	 * 
	 * @param leftValue: A float from -1 to 1 representing the speed of the left motors
	 * @param rightValue: A float from -1 to 1 representing the speed of the right motors
	 *
	 * @depreciated: which way is forward?
	 * 
	 */
	void SoccerBot::TankDrive(float leftValue, float rightValue)
	{
		leftValue*=-1; //left motor is reversed

		// square the inputs (while preserving the sign) to increase fine control while permitting full power
		if 	(leftValue >= 0.0)	{leftValue = (leftValue * leftValue);} 
		else					{leftValue = -(leftValue * leftValue);}
		if 	(rightValue >= 0.0)	{rightValue = (rightValue * rightValue);} 
		else					{rightValue = -(rightValue * rightValue);}
		
		//Set motor speeds
		SetLeftRightMotorSpeeds(leftValue, rightValue);
	}
	
	/* void Spin(double speed, int direction)
	 *
	 * This function spins the robot using tank drive using speed
	 * 
	 * @param speed: a double representing how fast the robot will turn
	 * @param direction: use the global constants of LEFT or RIGHT to indicate direction you would like to turn
	 *
	 * @depreciated: not used nor tested yet
	 */
	void SoccerBot::Spin(double speed, int direction)
	{
		if (speed < 0)
		{
			speed = speed * -1;
		}
		if (direction == LEFT)
		{
			SetLeftRightMotorSpeeds(-1 * speed, -1 * speed);
		} 
		else if (direction == RIGHT)
		{
			SetLeftRightMotorSpeeds(speed, speed);
		}
	}
	
	/* void SetLeftRightMotorSpeeds(float leftSpeed, float rightSpeed)
	 *
	 * This function sets the left and right motor speeds
	 * Protects Jaguars from exceeding max/min values
	 * 
	 * @param leftSpeed: What to set the left motors to (positive is forward)
	 * @param rightSpeed: What to set the right motors to (negative is forward)
	 * 
	 * @depreciated: When driving the robot, TankDrive() will offer better control
	 */
	void SoccerBot::SetLeftRightMotorSpeeds(float leftSpeed, float rightSpeed)
	{
		//Protect the motors from be set past their extremes
		if (leftSpeed > 1)		{leftSpeed = 1;}
		if (leftSpeed < -1)		{leftSpeed = -1;}
		if (rightSpeed > 1)		{rightSpeed = 1;}
		if (rightSpeed < -1)	{rightSpeed = -1;}		
		
		//Prevent stalling the motors and wasting power
		if (rightSpeed > -.1 && rightSpeed < 0.1)	{rightSpeed = 0;}
		if (leftSpeed > -.1 && leftSpeed < 0.1)		{leftSpeed = 0;}
				
		//Set motor speeds
		m_leftWheels->Set(leftSpeed);
		m_rightWheels->Set(rightSpeed);
	}
	
	/* void RotateXDegrees(double x, int direction, Gyro *m_gyro, bool quick = false)
	 *
	 * This function turns the robot using tank drive x degrees left or right... and it works! (thanks tristan)
	 * Breaks if button A is pressed or the robot has finished its rotation
	 * 
	 * @param x: a double representing how many degrees the robot will turn
	 * @param direction: use the global constants of LEFT or RIGHT to indicate direction you would like to turn
	 * @param m_gyro: Zoidberg's gyro to use to measure rotation
	 * @param quick: if true, will spin at full speed, if false, will slow down to be precise (default false)
	 */
	void SoccerBot::RotateXDegrees(double x, int direction, Gyro *m_gyro, bool quick = false)
	{
		//Create a variable to keep track of degrees rotated
		float degreesrotated = 0;
		
		//Create a variable to keep track of error
		float error = x;
		
		//create variable to know starting position
		float start = m_gyro->GetAngle();
		
		//while the robot hasn't rotated enough, turn
		while (degreesrotated * direction < x)
		{
			degreesrotated = m_gyro->GetAngle() - start;
			error = x - degreesrotated * direction;
			
			if (quick)
			{
				if (error > 20)	{SetLeftRightMotorSpeeds(1 *direction, 1 * direction);} 
				else			{SetLeftRightMotorSpeeds(.3 * direction, .3 * direction);}
			}

			else
			{
				//turn faster based on error (two positive numbers will rotate the bot b/c one motor is reversed)
				if (error > 60)		{SetLeftRightMotorSpeeds(1 * direction, 1 * direction);} 
				else if (error > 45){SetLeftRightMotorSpeeds(.7 * direction, .7 * direction);} 
				else if (error > 30){SetLeftRightMotorSpeeds(.45 * direction, .45 * direction);} 
				else				{SetLeftRightMotorSpeeds(.25 * direction, .25 * direction);}
			}
			
			//Stop and break loop if the robot has rotated enough
			if (degreesrotated * direction + ROTATEXDEGREES_ERROR >= x || m_Control->GetA())
			{
				SetLeftRightMotorSpeeds(0, 0);
				degreesrotated = degreesrotated + (3*x*direction); //breaks the loop
			}
		}
	}
	
	/* void DriveXFeet(double x, int direction)
	 *
	 * This function drives the robot x feet forwards or backwards
	 * 
	 * @param x: a double representing how many feet the robot is going to drive
	 * @param direction: use the global constants of FORWARD or BACKWARD to indicate direction you would like to drive
	 *
	 */
	void SoccerBot::DriveXFeet(double x, int direction)
	{
		//Begin by reseting the encoders
		m_rightEncoder->Reset();
		m_leftEncoder->Reset();
		
		//set the distance traveled (in feet) to 0
		double distanceTraveled = 0;
		
		//main control loop, while distance traveled < x feet
		while (distanceTraveled * direction < (x - DRIVEXFEET_ERROR))
		{
			//update distance traveled (average of two wheels)
			distanceTraveled = (m_rightEncoder->Get() + m_leftEncoder->Get()) / 2.0 * ENCODER_TO_FEET_RATIO;
			
			if (x > .2) //slow start
			{
				while (distanceTraveled * direction < .1)
				{
					//update distance traveled
					distanceTraveled = (m_rightEncoder->Get() + m_leftEncoder->Get()) / 2.0 * ENCODER_TO_FEET_RATIO;
					SetLeftRightMotorSpeeds(.3 * direction, -.3 * direction);
				}
				while (distanceTraveled * direction < .2)
				{
					//update distance traveled
					distanceTraveled = (m_rightEncoder->Get() + m_leftEncoder->Get()) / 2.0 * ENCODER_TO_FEET_RATIO;
					SetLeftRightMotorSpeeds(.5 * direction, -.5 * direction);
				}
			}
			
			//Drive faster if there is a long ways to go
			if (distanceTraveled * direction < (x - DRIVEXFEET_ERROR - 2)) 		 //need to travel >2 feet: 100% power
				SetLeftRightMotorSpeeds(1 * direction, -1 * direction);
			else if (distanceTraveled * direction < (x - DRIVEXFEET_ERROR - 1))  //else: need to travel >1 feet: 90% power
				SetLeftRightMotorSpeeds(.9 * direction, -.9 * direction);
			else if (distanceTraveled * direction < (x - DRIVEXFEET_ERROR - .5)) //else: need to travel >.5 feet: 70% power
				SetLeftRightMotorSpeeds(.7 * direction, -.7 * direction);
			else if (distanceTraveled * direction< (x - DRIVEXFEET_ERROR - .25)) //else: need to travel >.25 feet: 50% power
				SetLeftRightMotorSpeeds(.5 * direction, -.5 * direction);
			else
				SetLeftRightMotorSpeeds(.3 * direction, -.3 * direction); 		 //else: creep at 30% power

			//stop if the distance is equal to x Feet
			if (distanceTraveled * direction + DRIVEXFEET_ERROR >= x)
			{
				SetLeftRightMotorSpeeds(0, 0);
			}
		}
	}
	
	/* void TestShooter(float speed)
	 * 
	 * drives the shooter at speed
	 * 
	 * @param speed: -1 for forward, 0 for stop, 1 for backwards, SHOULD ALWAYS SHOOT FORWARDS
	 * 
	 */
	void SoccerBot::TestShooter(float speed)
	{
		//Mr. Matt says driving this motor backwards is a bad idea
		if (speed > 0) {speed = 0;}
		if (speed < -1){speed = -1;}
		m_shooter->Set(speed);
	}
	
	/* void Shoot()
	 * 
	 * shoots once
	 * 
	 * @depreciated: this function breaks the robot
	 * @depreciated: not used
	 * 
	 */
	void SoccerBot::Shoot()
	{
		cout << "shot load\n";
		//rotate past our cock point
		while (!(m_shooterAbsoluteEncoder->GetRawVoltage() > LEFT_BOUND_COCK
				- .5 && m_shooterAbsoluteEncoder->GetRawVoltage()
				< RIGHT_BOUND_COCK -.5))
			m_shooter->Set(-1);
		
		
		//stop rotating
		m_shooter->Set(0);
		cout<<"done shot\n";
		
	}
	
	/* void TestTensioner(float speed)
	 * 
	 * @param speed: how fast to move the tensioner
	 *
	 * @depreciated: which way is forwards?
	 * 
	 */
	void SoccerBot::TestTensioner(float speed)
	{
		if (speed > 1) {speed = 1;}
		if (speed < -1){speed = -1;}
		
		//prevent it from hitting the limit switch
		if (m_tensionerAbsoluteEncoder->GetRawVoltage() > RIGHT_BOUND_TENSION && speed > 0)
			m_tensioner->Set(speed);
		else if (m_tensionerAbsoluteEncoder->GetRawVoltage() < LEFT_BOUND_TENSION && speed < 0)
			m_tensioner->Set(speed);
		else //not within the safe range
			m_tensioner->Set(0);
	}
	
	/* void ReadAbsoluteEncoder()
	 * 
	 *  couts the voltage of both absolute encoders
	 * 
	 *  @depreciated: used for testing only
	 * 
	 */
	void SoccerBot::ReadAbsoluteEncoder()
	{
		cout << "ShooterAbsoluteEncoder: "
			<< m_shooterAbsoluteEncoder->GetRawVoltage() << endl
			<< "TensionerAbsoluteEncoder: "
			<< m_tensionerAbsoluteEncoder->GetRawVoltage() << endl;
	}
	
	/* void ReadRelativeEncoder()
	 * 
	 * Same as Absolute, but for drive wheels (outputs in clicks and feet)
	 *
	 * @depreciated: used for testing only
	 *
	 */
	void SoccerBot::ReadRelativeEncoder()
	{
		cout << "Left Wheel Encoder: " << m_leftEncoder->Get() << endl;
		cout << "Right Wheel Encoder: " << m_rightEncoder->Get() << endl;
	}
		
	/* bool IsCocked()
	 *
	 * @return: true if kicker is between the cock bounds
	 *
	 */
	bool SoccerBot::IsCocked()
	{
		if (m_shotTimer->Get() > 1)
		{
			return (m_shooterAbsoluteEncoder->GetRawVoltage() > LEFT_BOUND_COCK
					&& m_shooterAbsoluteEncoder->GetRawVoltage() < RIGHT_BOUND_COCK);
		}
		else
		{
			return (m_shooterAbsoluteEncoder->GetRawVoltage() > LEFT_BOUND_COCK +.5 
					&& m_shooterAbsoluteEncoder->GetRawVoltage() < RIGHT_BOUND_COCK +.5);
		}
	}
		
	/* bool IsUncocked()
	 *
	 * @return: true if kicker .4 away from and not between the cock bounds
	 *
	 */
	bool SoccerBot::IsUncocked()
	{
		return (m_shooterAbsoluteEncoder->GetRawVoltage() < LEFT_BOUND_COCK-.4
				|| m_shooterAbsoluteEncoder->GetRawVoltage() > RIGHT_BOUND_COCK +.4);
	}
};

/*
 * PID CLASS
 * @depreciated: not used because we dont autoaim :'(
 */
class SamplePIDOutput : public PIDOutput
{
public:
	//Constructor
	SamplePIDOutput::SamplePIDOutput(SoccerBot *socbot)
	{
		m_socbot=socbot;
	}
	
	//PID Write
	void SamplePIDOutput::PIDWrite(float output)
	{
		if (pidena)
		{
			m_socbot->ArcadeDrive(m_socbot->m_Control->GetLeftY(), output);
			outputValue=-output;
		}
	}
private:
	SoccerBot *m_socbot;
};

void findelpises(); //needs to be declared here, matt z.

/*
 * ROBOT CLASS
 */
class Zoidberg
{
public:
	//members:
	SoccerBot *m_SoccerBot;
	i2cAccel *m_accelSensor; //Tristan's i2cAccel class
	Gyro *m_gyro;
	double xAccel, yAccel, zAccel; // accel vals
	double xJerk, yJerk, zJerk; //rate change of accel values
	PIDOutput *pidOutput;	//for autoaim
	PIDController *m_turnController; //for autoaim
	DriverStation *m_ds; //@depreciated: why does Zoidberg need a ds? shouldn't this be in the Main class? NO ITS FINE DON'T TOUCH WHAT YOU DON'T UNDERSTAND
	bool enabledpid;

	/* Zoidberg Default Constructor
	 * 
	 */
	Zoidberg::Zoidberg(Object360 *m_ControllerDrive)
	{
		//Set up SoccerBot, pass the controller
		m_SoccerBot = new SoccerBot(m_ControllerDrive);
		
		//Set up Gyro
		m_gyro=new Gyro(1);
		m_gyro->SetSensitivity(0.00729);
		
		//Set up Dashboard
///		m_Dashboard=new DashboardDataSender();
		
		//Set up Driver Station (for inputs)
		m_ds= DriverStation::GetInstance();
		
		//Set up Accelerometer (not used right now)
		//m_accelSensor = new i2cAccel(2);
		xAccel = yAccel = xJerk = yJerk = zJerk = 0;
		zAccel = 1;
		
		//Set up PID @depreciated: not used
		/*pidOutput = new SamplePIDOutput(m_SoccerBot);
		printf("Initializing PIDController\n");
		cout << "starting pid";
		m_turnController= new PIDController( 0.025, // P
				0.000000000001, // I
				0.5, // D
				m_gyro, // source
				pidOutput, // output
				0.005); // period
		m_turnController->SetInputRange(-360.0, 360.0);
		m_turnController->SetOutputRange(-0.6, 0.6);
		m_turnController->SetTolerance(1.0 / 90.0 * 100);
		m_turnController->Disable();*/
		cout<<"pid initialized";
		enabledpid=false; //PID disabled for now
		
		//Set up Servo parameters for camera @depreciated: not used
		servocen = (m_SoccerBot->m_panServ->GetMaxAngle() + m_SoccerBot->m_panServ->GetMinAngle()) / 2;
		
		//Set default lifter value
		Lift(0);
	}
	
	/* void Bump()
	 *
	 * Takes the robot up to the top of the bump, then stops
	 * @depreciated: not yet tested, TRISTAIN WE NEED YOU!
	 * !MainRobotProgram::IsOperatorControl();
	 */
	void Zoidberg::Bump()
	{
		bool leveled = true;
		bool overBump = false;
		//this is all modded from my old code, (which never got tested more than once), 
		//and now everything must be reversed, so it probly wont work the first time
		m_SoccerBot->SetLeftRightMotorSpeeds(1, -1); //i believe this is right assuming we go over FORWARDS
		resetMotionData();
		
		//this loop should exit when robot impacts the bump
		while (zAccel > 0.76 || yAccel < 0.24 &&!m_ds->IsOperatorControl()) //if this doesnt work, yAccel might need to be reversed
		{
			collectMotionData();
			if (!leveled  && zAccel > 0.88)
			{
				cout<<"ERROR CODE 1"<<endl;    //zAccel values need increasing
 			}
			if (zAccel < 0.85)
				leveled = false;
		}
		leveled = false;
		cout << "Hit the bump" << endl;
		Wait(0.35); //if the robot flies over the bump, reduce this, if it stalls, increase it
		
		//this loop should exit when robot is just over the peak of the bump
		while (yAccel > -0.10 && !m_ds->IsOperatorControl())
		{
			collectMotionData();
			if (overBump && zAccel < 0.85)
				cout<<"ERROR CODE 2"<<endl;  //yAccel values need increasing (
			if (zAccel > 0.95 && yAccel > 0 && !overBump)
				overBump = true;
		}
		Wait(.6);
		cout<<"Going down the bump"<<endl;
		//when on the other side, stop.  (or take the next line out, the autonomous zone 2 should take over after this anyway.)
		m_SoccerBot->SetLeftRightMotorSpeeds(0, 0);
		//may be a good idea to put a rotateXDegrees here (or at least check alignment) to ensure the bot hasnt turned at all.
		resetMotionData();
	}

	/* void Lift(float speed)
	 *
	 * @param speed: how fast to lift the robot
	 *
	 */
	void Zoidberg::Lift(float speed)
	{
		m_SoccerBot->m_lifter->Set(speed);
	}

	/* void LatchHooker()
	 *
	 * Sets the servo controling the hook to the latched position
	 *
	 */
	void Zoidberg::LatchHooker()
	{
		m_SoccerBot->m_hookerServ->SetAngle(LEFT_BOUND_HOOKER);
	}

	/* void ReleaseHooker()
	 *
	 * Sets the hooker servo to the release position
	 *
	 */
	void Zoidberg::ReleaseHooker()
	{
		if (m_SoccerBot->m_hookerTimer->Get()> 100)//we're in the finale
		{
			m_SoccerBot->m_hookerServ->SetAngle(RIGHT_BOUND_HOOKER);
		}
	}

	/* void SetPID()
	 *
	 * Updates the PID values to some values on the driverstation inputs
	 * @depreciated: not used 
	 */
	void Zoidberg::SetPID()
	{
		m_turnController->SetPID(m_ds->GetAnalogIn(1), m_ds->GetAnalogIn(2), m_ds->GetAnalogIn(3));
		printf("i value: %f \n", m_turnController->GetI());
		printf("ana in %f \n", m_ds->GetAnalogIn(2));
	}

	/* void Aim()
	 *
	 * @depreciated: not used! PROBABLY DOESNT EVEN WORK
	 *
	 */
	void Zoidberg::Aim()
	{
		cout<<"enabled";
		float gyroAngle = m_gyro->PIDGet();

		float setPoint = gyroAngle + getHorizontalAngle();
		if (setPoint - gyroAngle < -80 || setPoint - gyroAngle > 80)
			setPoint=gyroAngle;
		m_turnController->SetSetpoint(setPoint);
		if(!enabledpid)
			m_turnController->Enable();
		enabledpid=true;
	}

	/* void StopAim()
	 *
	 * Disables the PID and prevents use of the camera to auto-aim
	 *
	 */
	void Zoidberg::StopAim()
	{
		m_turnController->Disable();
		enabledpid=false;
	}

	/* void ZeroCamera()
	 *
	 * @depreciated: not used
	 *
	 */
	void Zoidberg::ZeroCamera()
	{	
		//if (fabs(getHorizontalAngle())>fabs(m_SoccerBot->m_panServ->GetAngle()))
		m_SoccerBot->m_panServ->SetAngle((m_SoccerBot->m_panServ->GetMaxAngle()+m_SoccerBot->m_panServ->GetMinAngle())/2);
		/*else if (fabs(m_SoccerBot->m_panServ->GetAngle())>fabs(getHorizontalAngle()))
		 m_SoccerBot->m_panServ->Set(m_SoccerBot->m_panServ->GetAngle()- getHorizontalAngle());	*/
	}

	/* void viewMotionData()
	 * 
	 * couts all the data relevent to the accelerometer
	 *  
	 */
	void Zoidberg::viewMotionData()
	{
		cout<<"X Accel: "<< xAccel << endl
			<<"Y Accel: "<< yAccel << endl
			<<"Z Accel: "<< zAccel << endl
			<<"X Jerk: " << xJerk << endl
			<<"Y Jerk: " << yJerk << endl
			<<"Z Jerk: " << zJerk << endl;
	}

	/* void resetMotionData()
	 *
	 * Sets Accelerations and Jerks to default values (as if the robot were at rest)
	 *
	 */
	void Zoidberg::resetMotionData()
	{
		xAccel = yAccel = xJerk = yJerk = zJerk = 0;
		zAccel = 1; //gravity!
	}

	/* static float getRange()
	 *
	 * @depreciated: matt... plz explain
	 *
	 */
	static float Zoidberg::getRange()
	{
		Synchronized syn2(rangesem);
		return range;
	}
	
	/* float getRange(Target target)
	 * 
	 * Get the range to the target in feet
	 * @param target: the target to get the range of
	 * 
	 * @return: how far away the target is (in feet)
	 * 
	 */
	static float Zoidberg::getRange(Target target)
	{
		//similar triangle problem, known height of target, known focal length, known size on sensor
		float focal =4.0;
		vector<Target> targets;
		targets[0]=target;

		float sizeonsen=target.m_majorRadius*(.25f*25.4);
		float range=0.0;
		if(target.m_bothFound==true)
		range=(838.2f*focal)/sizeonsen;

		else
		range=(584.2f*focal)/sizeonsen;
		range=range/25.4;
		range=range/12;
		range/=.88348937434982909793431416257988f; //precision <= necessary

		return range;
	}

	/* gets the pan servo angle*/
	float Zoidberg::getServoAngle()
	{
		return m_SoccerBot->m_panServ->GetAngle();
	}

	/*gets the gyro angle*/
	float Zoidberg::getGyroAngle()
	{
		return m_gyro->GetAngle();
	}

	/*starts the camera for getting images back to dashboard and image processing*/
	void Zoidberg::startCamera()
	{
		//detect=new Task("targeting",(FUNCPTR)(findelpises));
		//detect->Start();
	}

	/*
	 * returns the horizontal angle of the primary target, if we don't have a target it could be problematic and things would not be much good at all
	 * @return: horizontal angle in degrees
	 * @depreciated: units are unsure
	 */
	float Zoidberg::getHorizontalAngle()
	{
		Synchronized sync(horizontal_angle_sem);
		return HoriAng;
	}

	/* void collectMotionData
	 * 
	 * @depreciated: math is not well commented 
	 * 
	 * This Function collects motion data for use in functions, it works to cancel out noise with a moving average
	 */
	void Zoidberg::collectMotionData()
	{
		double accel;
		accel = m_accelSensor->getXAccel(); //noise cancelling using a moving average calculation
		xJerk += (0.1 * ((accel - xAccel) - xJerk));
		xAccel += (0.1 * (accel - xAccel));
		accel = m_accelSensor->getYAccel();
		yJerk += (0.1 * ((accel - yAccel) - yJerk));
		yAccel += (0.1 * (accel - yAccel));
		accel = m_accelSensor->getZAccel();
		zJerk += (0.1 * ((accel - zAccel) - zJerk));
		zAccel += (0.1 * (accel - zAccel));
	}
};

/*Auto Shooter task
 * 
 * Shoots the shooter while the ball sensor is tripped
 *
 * @depreciated: maybe this breaks the robot? sometimes maybe sorta?
 * 
 */
void Shooter(Zoidberg *m_robotDrive, AnalogChannel *opticalSensor)
{//TODO: fixthis 
	cout << "Automated Shooting Enabled\n";
	bool seeyou=false;
	while (autoshoot_task_started==true)
	{
		if (opticalSensor->GetVoltage() >2)
		{
			if (!seeyou)
			{
				cout << "Target Aquired" << endl;
			}
			seeyou=true;
			while(!m_robotDrive->m_SoccerBot->IsUncocked())
			{
			m_robotDrive->m_SoccerBot->TestShooter(-1);
			}
			m_robotDrive->m_SoccerBot->TestShooter(0);
			while(!m_robotDrive->m_SoccerBot->IsCocked()&&autoshoot_task_started==true)
			{
				m_robotDrive->m_SoccerBot->m_shooter->Set(-.8);
			}
			m_robotDrive->m_SoccerBot->m_shooter->Set(0);
			Wait(.2);
			
			printf("pew");
		} 
		else
		{
			if (seeyou)
			{
				cout << "Target lost" << endl;
			}
			seeyou=false;
			
			/*if (!m_robotDrive->m_SoccerBot->IsCocked())
			{
				while(!m_robotDrive->m_SoccerBot->IsCocked()&&autoshoot_task_started==true)
				{m_robotDrive->m_SoccerBot->m_shooter->Set(-.8);}
				m_robotDrive->m_SoccerBot->m_shooter->Set(0);
			}
			else if(autoshoot_task_started==true)
			{
				m_robotDrive->m_SoccerBot->m_shooter->Set(0);
			}*/
		}
	}
}


/*
 * MAIN CLASS
 */
class MainRobotClass : public IterativeRobot
{
	
	//Members:
	
	//Objects
	Timer *m_timer;
	AnalogChannel *m_ballSensor;
	Object360 *m_ControllerDrive, *m_ControllerShoot;
	Joystick *m_ControllerTest;
	Zoidberg *m_robotDrive;

	//Driver Station
	DriverStation *m_ds; // driver station object
	UINT32 m_priorPacketNumber; // keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond; // keep track of the ds packets received in the current second

	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;

	//Button Pressing Variables
	bool DriveAPressed;
	bool ShootRClickPressed;
	bool LeftClickPressed, AutoAimPressed;//Not well named or used?

	//Driver-Specific Variables
	bool isRenee, isBFinneyInverted, isBFinneySlow, isJeffTension; //@depreciated, never inverted, never isRenee
	int tensionSetPoint;
	bool cameraControl; //@depreciated: never true!

	//Variables for Autonomous Mode
	int ballsKicked;
	bool dukes;
	UINT8 zone;
	double autonomousWaitTime;
	bool GTFO;
	bool bumpy;
	float autospeed;

	//Task Variables
	
	
	//Special bools
	bool SWITCHTHATBLOWSUPTHEWORLD;
	bool havecyp;//if this is false panic

public:
	/* MainRobotClass Default Constructor
	 * 
	 */
	MainRobotClass::MainRobotClass(void)
	{
		//Controllers
		
		m_ControllerDrive = new Object360(1);
		m_ControllerShoot = new Object360(2);
		m_ControllerTest = new Joystick(3);
		
		//Zoidberg
		m_robotDrive = new Zoidberg(m_ControllerDrive);
		
		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;
		
		//Other Objects
		m_timer = new Timer();
		m_ballSensor = new AnalogChannel(ANIN_PHOTO_BALL_SENSOR);
		
		//Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
		
		//Set driver variables to defaults
		isRenee = false;
		isBFinneyInverted = isBFinneySlow = isJeffTension = false;
		tensionSetPoint = 0;
		cameraControl = false;
		
		//No buttons are pressed;
		LeftClickPressed = ShootRClickPressed = DriveAPressed = AutoAimPressed = false;
		
		//Tasks are not yet started
		autoshoot_task_started = false;
		dukes=false;
		bumpy=false;
		autospeed=0.0f;
		SWITCHTHATBLOWSUPTHEWORLD=false;
		havecyp=true;
		
	}
	
	/* void chkcyp()
	 *
	 * this sets the digital outputs (LEDs), for the inverse of the digital inputs (active low switches)
	 *
	 * @depreciated: doesn't use CYP globals >:O
	 *
	 */
	void chkcyp()
	{
		if (!m_ds->GetEnhancedIO().GetDigital(1))
			m_ds->GetEnhancedIO().SetDigitalOutput(9, true);
		else
			m_ds->GetEnhancedIO().SetDigitalOutput(9, false);
		if (!m_ds->GetEnhancedIO().GetDigital(2))
			m_ds->GetEnhancedIO().SetDigitalOutput(10, true);
		else
			m_ds->GetEnhancedIO().SetDigitalOutput(10, false);
		if (!m_ds->GetEnhancedIO().GetDigital(3))
			m_ds->GetEnhancedIO().SetDigitalOutput(11, true);
		else
			m_ds->GetEnhancedIO().SetDigitalOutput(11, false);
		if (!m_ds->GetEnhancedIO().GetDigital(4))
			m_ds->GetEnhancedIO().SetDigitalOutput(12, true);
		else
			m_ds->GetEnhancedIO().SetDigitalOutput(12, false);
		if (!m_ds->GetEnhancedIO().GetDigital(5))
			m_ds->GetEnhancedIO().SetDigitalOutput(13, true);
		else
			m_ds->GetEnhancedIO().SetDigitalOutput(13, false);
		if (!m_ds->GetEnhancedIO().GetDigital(6))
			m_ds->GetEnhancedIO().SetDigitalOutput(14, true);
		else
			m_ds->GetEnhancedIO().SetDigitalOutput(14, false);
		if (!m_ds->GetEnhancedIO().GetDigital(7))
			m_ds->GetEnhancedIO().SetDigitalOutput(15, true);
		else
			m_ds->GetEnhancedIO().SetDigitalOutput(15, false);
		if (!m_ds->GetEnhancedIO().GetDigital(8))
			m_ds->GetEnhancedIO().SetDigitalOutput(16, true);
		else
			m_ds->GetEnhancedIO().SetDigitalOutput(16, false);
	}
	
	
	/* double GetTension()
	 * this returns the tension as a percentage 
	 * this is used for dashboard
	 *
	 * @deprecated the equation maybe incorrect and needs to be checked/fixed BEFORE STATES 
	 */
	double GetTension()
	{
		return ((m_robotDrive->m_SoccerBot->m_tensionerAbsoluteEncoder->GetRawVoltage()
				* -82.7287242403) + 229.359596946); //TODO: check equation now that it is fixed
	}
	
	/********************************** Init Routines *************************************/

	void MainRobotClass::RobotInit(void)
	{
		cout << "Entering Robot Init" << endl;
		cout << "TEAM 2619 THE CHARGE" << endl;
		
		//create tasks for the shooter
		autoshoot = new Task("autoshooter",(FUNCPTR)(Shooter));
		
		//Set up our watchdog
		GetWatchdog().SetEnabled(false);
		GetWatchdog().SetExpiration(10000); //@depreciated: this probably isnt safe...
		//Wait(1);
		
		//Determine if renee is driving
		isRenee = false;
		
		m_timer->Start();
		
		m_robotDrive->Lift(0);
	}
	
	void MainRobotClass::DisabledInit(void)
	{
		cout << "Entering Disabled Init" << endl;
	}
	
	void MainRobotClass::AutonomousInit(void)
	{
		cout << "Entering Autonomous Init" << endl;
		
		m_autoPeriodicLoops = 0; // Reset the loop counter for autonomous mode
		m_robotDrive->m_gyro->Reset();
		ballsKicked = 0;
				
		//Determine what zone we are in based on Cyprus inputs
				
		//Determine how to end the autonomous period
		if(havecyp)
		{
			if(zone==3&&!m_ds->GetEnhancedIO().GetDigital(5))
			{
				dukes=true;
				bumpy=false;
				GTFO=false;		
			}
			else if(!m_ds->GetEnhancedIO().GetDigital(5))
			{
				dukes=false;
				bumpy=false;
				GTFO=true;
			}
			else if(!m_ds->GetEnhancedIO().GetDigital(4)&&zone==3)
			{
				//zone=3;
				bumpy=true;
				GTFO=false;
				dukes=false;
			}
			if(m_ds->GetEnhancedIO().GetDigital(5))
			{
				dukes=false;
				GTFO=false;
				bumpy=false;
			}
		}
			autonomousWaitTime = (3.3-m_ds->GetEnhancedIO().GetAnalogIn(4)) * 3;
			autonomousWaitTime <.5 ? autonomousWaitTime=0:autonomousWaitTime=autonomousWaitTime;
	
		
		autospeed = .25;//this value was used for the entirety of the Ann Arbor competition.  it worked.
		

		cout << "Will wait for: " << autonomousWaitTime << "seconds " << endl;
	}
	
	void MainRobotClass::TeleopInit(void)
	{
		cout << "Entering Teleop Init" << endl;
		m_telePeriodicLoops = 0; // Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0; // Reset the number of dsPackets in current second
	}
	
	
	/********************************** Periodic Routines *************************************/

	void MainRobotClass::DisabledPeriodic(void)
	{
		// feed the user watchdog at every period when disabled
		GetWatchdog().Feed();
		//m_ds->ClearError();
		//m_ds->GetEnhancedIO().GetFirmwareVersion();
		//m_ds->GetEnhancedIO();
		havecyp=false;//m_ds->GetEnhancedIO().GetDigitalConfig(9)==m_ds->GetEnhancedIO().kOutput;// ? havecyp=true:havecyp=false;
		
		//m_ds->GetError().GetCode()==0 || m_ds->GetEnhancedIO().GetError().GetCode()==0 ? havecyp=havecyp&&true:havecyp=havecyp||false;
		
		
		//m_ds->GetEnhancedIO().SetDigitalOutput(9,true);
		//havecyp=(!m_ds->GetEnhancedIO().StatusIsFatal())||havecyp;
		
		if (m_disabledPeriodicLoops==10)
		{
			//m_robotDrive->startCamera();
		}
		if (m_disabledPeriodicLoops>10)
		{ 
			//Dashboard Stuff
///			m_Dashboard->sendIOPortData(false,
///					m_robotDrive->getHorizontalAngle(), (float)GetTension(), autonomousWaitTime,
///					tensionSetPoint, m_robotDrive->getRange(), !isRenee,
///					(float)zone, isBFinneyInverted, GTFO);
		}
		if(havecyp)
		{
			autonomousWaitTime = (3.3-m_ds->GetEnhancedIO().GetAnalogIn(4)) * 3;
			chkcyp();
			autonomousWaitTime <.5 ? autonomousWaitTime=0:autonomousWaitTime=autonomousWaitTime;
		}
		else
			autonomousWaitTime=0;
		
		
		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
		
		//ensure the hooker is latched!
		m_robotDrive->LatchHooker();
		if(havecyp)
		{
			if (!m_ds->GetEnhancedIO().GetDigital(8))
				zone = 1;
			else if (!m_ds->GetEnhancedIO().GetDigital(7))
				zone = 2;
			else if (!m_ds->GetEnhancedIO().GetDigital(6))
				zone = 3;
		}
		else
		{
			if (m_ds->GetDigitalIn(1))
				zone = 1;
			else if (m_ds->GetDigitalIn(2))
				zone = 2;
			else if (m_ds->GetDigitalIn(3))
				zone = 3;
		}
		
		
		
	}
	
	void MainRobotClass::AutonomousPeriodic(void)
	{
		//Dashboard Stuff
///		m_Dashboard->sendIOPortData(false, m_robotDrive->getHorizontalAngle(),
///				(float)GetTension(), 0, tensionSetPoint,
///				m_robotDrive->getRange(), !isRenee, (float)zone,
///				isBFinneyInverted, GTFO);
		
		//reset gyro in first loop
		if (m_autoPeriodicLoops == 0)
		{
			m_robotDrive->m_gyro->Reset();
		}
		
		//if di 1,2,3 are all false then this will run
		
			if (havecyp&&!m_ds->GetEnhancedIO().GetDigital(1)&&!m_ds->GetEnhancedIO().GetDigital(2) &&!m_ds->GetEnhancedIO().GetDigital(3))
			{
				//OH CRAP 469 AUTO
				while (fabs(m_robotDrive->m_gyro->GetAngle())<185&&!IsOperatorControl())
					m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(1, 0);
				while (!IsOperatorControl())
					m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(1, -1);//this needs to be tested out
				//this should run us around one traction wheel and wedge us into the tunnel flooring it forwards to keep us in place
				//may God have mercy on whoever gets in our way
			} 
		
		
		else //not 469 mode
		{
			//Begin by cocking	
			while (!m_robotDrive->m_SoccerBot->IsCocked()&&!IsOperatorControl())
			{
				m_robotDrive->m_SoccerBot->m_shooter->Set(-1);
			}
			cout << "cocked" << endl;
			m_robotDrive->m_SoccerBot->m_shooter->Set(0);
			
			//Kick as many balls as are in the zone
			if (ballsKicked < zone)
			{
				//Dashboard Stuff
///				m_Dashboard->sendIOPortData(false,
///						m_robotDrive->getHorizontalAngle(),
///						(float)GetTension(), 0, tensionSetPoint,
///						m_robotDrive->getRange(), !isRenee, (float)zone,
///						isBFinneyInverted, GTFO);
				
				if (ballsKicked == 0)
				{
					m_robotDrive->m_gyro->Reset(); //current heading is 0
					m_timer->Reset(); //reset wait time
				}
				cout << "now waiting" << endl;
				while (m_ballSensor->GetVoltage() < 1 && !IsOperatorControl())
				{
					//Dashboard Stuff
///					m_Dashboard->sendIOPortData(false,
///							m_robotDrive->getHorizontalAngle(),
///							(float)GetTension(), 0, tensionSetPoint,
///							m_robotDrive->getRange(), !isRenee, (float)zone,
///							isBFinneyInverted, GTFO);
					
					GetWatchdog().Feed();
					
					//zone 1 tension: 40
					if 		(zone == 1 && ballsKicked == 0 && GetTension() >40){m_robotDrive->m_SoccerBot->m_tensioner->Set(-1);} 
					else if(zone==1&&ballsKicked==0&& GetTension()<40){m_robotDrive->m_SoccerBot->m_tensioner->Set(1);}
					else if (zone == 1 && ballsKicked == 0){m_robotDrive->m_SoccerBot->m_tensioner->Set(0);}
					
						
					//zone 2 tensions: 100, then 85ish
					if 		(zone == 2 && ballsKicked == 0 && GetTension() < 99){m_robotDrive->m_SoccerBot->m_tensioner->Set(1);} 
					else if (zone == 2 && ballsKicked == 0)						{m_robotDrive->m_SoccerBot->m_tensioner->Set(0);}
					if 		(zone == 2 && ballsKicked == 1 && GetTension() < 82){m_robotDrive->m_SoccerBot->m_tensioner->Set(1);} 
					else if (zone == 2 && ballsKicked == 1 && GetTension() > 87){m_robotDrive->m_SoccerBot->m_tensioner->Set(-1);} 
					else if (zone == 2 && ballsKicked == 1)						{m_robotDrive->m_SoccerBot->m_tensioner->Set(0);}
										
					//zone 3 tension: 100, 100, 100
					if 		(zone == 3 && ballsKicked == 0 && GetTension() < 99){m_robotDrive->m_SoccerBot->m_tensioner->Set(1);} 
					else if (zone == 3 && ballsKicked == 0)						{m_robotDrive->m_SoccerBot->m_tensioner->Set(0);}
					if 		(zone == 3 && ballsKicked == 1 && GetTension() < 99){m_robotDrive->m_SoccerBot->m_tensioner->Set(1);} 
					else if (zone == 3 && ballsKicked == 1)						{m_robotDrive->m_SoccerBot->m_tensioner->Set(0);}
					if 		(zone == 3 && ballsKicked == 2 && GetTension() > 90){m_robotDrive->m_SoccerBot->m_tensioner->Set(-1);} 
					else if (zone == 3 && ballsKicked == 2 && GetTension() < 90){m_robotDrive->m_SoccerBot->m_tensioner->Set(1);} 
					else if (zone == 3 && ballsKicked == 2)						{m_robotDrive->m_SoccerBot->m_tensioner->Set(0);}
										
					//creep forward using gyro to correct heading if necessary
					if (m_timer->Get() > autonomousWaitTime) //have waited long enough
					{
						if 		(m_robotDrive->m_gyro->GetAngle()> 3)	{m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(autospeed, -(autospeed+.1));}
						else if (m_robotDrive->m_gyro->GetAngle() < -3)	{m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds((autospeed+.1), -autospeed);}
						else /*robot is driving straight*/				{m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds((autospeed), -(autospeed));}
					}

					//creep forward using camera to aim the robot
					else if (false) //@depreciated, NOT USED! (i'd probably delete it...)
					{
						if (m_timer->Get() > autonomousWaitTime) //have waited long enough - waittime set in Autonomous Init
						{
							if (m_robotDrive->getHorizontalAngle()> 3 && m_robotDrive->getHorizontalAngle()<20)
								m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds((autospeed), -(autospeed+.1));
							else if (m_robotDrive->getHorizontalAngle() < -3 &&m_robotDrive->getHorizontalAngle() <-20)
								m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds((autospeed+.1), -(autospeed));
							else
								m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds((autospeed), -(autospeed));
						}
					}
				}
				
				//SHOOT!
				cout << "Waiting .2 Before Shooting" << endl;
				Wait(.2); //Autonomous Lag (time between sensing and kicking a ball)
				cout << "Fire Torpedo Number " << ballsKicked + 1 << endl;
				while (!m_robotDrive->m_SoccerBot->IsUncocked() && !IsOperatorControl())
				{
					m_robotDrive->m_SoccerBot->m_shooter->Set(-1);
				}
				m_robotDrive->m_SoccerBot->m_shooter->Set(0);
				cout << "Autonomous shot" << endl;
				
				//Stop and recock
				m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(0, 0);
				while (!m_robotDrive->m_SoccerBot->IsCocked() && !IsOperatorControl())
				{
					m_robotDrive->m_SoccerBot->m_shooter->Set(-1);
				}
				m_robotDrive->m_SoccerBot->m_shooter->Set(0);
				cout << "cocked" << endl;
				Wait(.1);
				
				//increment balls kicked
				ballsKicked++;
				
				//stop tensioning when all balls in the zone have been cleared
				if (ballsKicked == zone)
					m_robotDrive->m_SoccerBot->m_tensioner->Set(0);
				
			}
			
			//increment the number of autonomous perodic loops completed
			m_autoPeriodicLoops++;
			
			if (ballsKicked == zone)
			{
				cout << "Zone cleared in " << m_timer->Get() << "seconds" << endl;
				
				//Dukes of Hazard!!!
				if (dukes)
				{
					//full speed over the bump until a ball is seen!
					while (m_ballSensor->GetVoltage() < 2 && !IsOperatorControl())
					{
						m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(1,-1);
					}
					
					//SHOOT!
					cout << "Waiting .2 Before Shooting" << endl;
					Wait(.2);
					cout << "Fire Torpedo Number " << ballsKicked + 1 << endl;
					while (!m_robotDrive->m_SoccerBot->IsUncocked() && !IsOperatorControl())
					{
						m_robotDrive->m_SoccerBot->m_shooter->Set(-1);
					}
					m_robotDrive->m_SoccerBot->m_shooter->Set(0);
					cout << "Autonomous shot" << endl;
					
					//Stop and recock
					m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(0, 0);
					while (!m_robotDrive->m_SoccerBot->IsCocked() && !IsOperatorControl())
					{
						m_robotDrive->m_SoccerBot->m_shooter->Set(-1);
					}
					m_robotDrive->m_SoccerBot->m_shooter->Set(0);
					cout << "cocked" << endl;
					Wait(.1);
					
					//increment balls kicked
					ballsKicked++;
					
					while (ballsKicked < 6 && !IsOperatorControl())
					{
						//creep forward, adjusting heading using gyro
						if (m_robotDrive->m_gyro->GetAngle()> 3)
							m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(autospeed, -(autospeed+.1));
						else if (m_robotDrive->m_gyro->GetAngle() < -3)
							m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds((autospeed+.1), -autospeed);
						else
							m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds((autospeed), -(autospeed));
						
						//keep driving until a ball is seen
						if (m_ballSensor->GetVoltage()>2)
						{
							//SHOOT!
							cout << "Waiting .2 Before Shooting" << endl;
							Wait(.2);
							cout << "Fire Torpedo Number " << ballsKicked + 1 << endl;
							while (!m_robotDrive->m_SoccerBot->IsUncocked() && !IsOperatorControl())
							{
								m_robotDrive->m_SoccerBot->m_shooter->Set(-1);
							}
							m_robotDrive->m_SoccerBot->m_shooter->Set(0);
							cout << "Autonomous shot" << endl;
							
							//Stop and recock
							m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(0, 0);
							while (!m_robotDrive->m_SoccerBot->IsCocked() && !IsOperatorControl())
							{
								m_robotDrive->m_SoccerBot->m_shooter->Set(-1);
							}
							m_robotDrive->m_SoccerBot->m_shooter->Set(0);
							cout << "cocked" << endl;
							Wait(.1);
							
							//increment balls kicked
							ballsKicked++;
						}
					}
					//stop the robot
					m_robotDrive->m_SoccerBot->TankDrive(0, 0);
				} 
				
				//Use the accelerometer to go over the bump
				else if (bumpy)
				{
					//go over the bump... carefully
					m_robotDrive->Bump();
					
					//kick 2 more balls
					while (ballsKicked<5&& !IsOperatorControl())
					{
						if 		(m_robotDrive->m_gyro->GetAngle()> 3) 	{m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(autospeed, -(autospeed+.1));}
						else if (m_robotDrive->m_gyro->GetAngle() < -3)	{m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds((autospeed+.1), -autospeed);}
						else{m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds((autospeed), -(autospeed));}
						
						if (m_ballSensor->GetVoltage()>2)
						{
							//SHOOT!
							cout << "Waiting .2 Before Shooting" << endl;
							Wait(.2);
							cout << "Fire Torpedo Number " << ballsKicked + 1 << endl;
							while (!m_robotDrive->m_SoccerBot->IsUncocked() && !IsOperatorControl())
							{
								m_robotDrive->m_SoccerBot->m_shooter->Set(-1);
							}
							m_robotDrive->m_SoccerBot->m_shooter->Set(0);
							cout << "Autonomous shot" << endl;
							
							//Stop and recock
							m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(0, 0);
							while (!m_robotDrive->m_SoccerBot->IsCocked() && !IsOperatorControl())
							{
								m_robotDrive->m_SoccerBot->m_shooter->Set(-1);
							}
							m_robotDrive->m_SoccerBot->m_shooter->Set(0);
							cout << "cocked" << endl;
							Wait(.1);
							ballsKicked++;
						}
					}
					m_robotDrive->m_SoccerBot->TankDrive(0, 0);
				}
				
				//back up and spin move
				else if (zone == 3 && ballsKicked >= 3) // not dukes or bumpy
				{
					m_robotDrive->m_SoccerBot->DriveXFeet(3, BACKWARD);
					m_robotDrive->m_SoccerBot->RotateXDegrees(180.0, LEFT, m_robotDrive->m_gyro);
				}
				
				//turn towards the center line and get out of the way
				else if (GTFO && ballsKicked == zone && !IsOperatorControl())
				{
					m_robotDrive->m_SoccerBot->RotateXDegrees(90, RIGHT, m_robotDrive->m_gyro);
					m_robotDrive->m_SoccerBot->DriveXFeet(6, FORWARD);
					m_robotDrive->m_SoccerBot->SetLeftRightMotorSpeeds(0, 0);
					ballsKicked++;//skip stuff when we're done
				}
				//increment balls kicked to ensure these modes dont run twice	
				ballsKicked++;
			}
		}
	}
		
	/* void TeleopPeriodic
	 * 
	 * what our robot does in teleop enabled
	 * 
	 * @depreciated: not all of the couts are still necessary
	 * 
	 */
	void MainRobotClass::TeleopPeriodic(void)
	{
		
		//Update cyprus LEDs
		if(havecyp)
		{
			chkcyp();
		}
		
		//feed the user watchdog at every period when in autonomous
		GetWatchdog().Feed();
		
		/*this code is for the big missle switch on the cypress, it had to be hooked up to an analog input because
		 * we did not have another digital input, 1.5 seems like a good cutoff voltage*/
		if(havecyp)
		{
			if (m_ds->GetEnhancedIO().GetAnalogIn(5) < 1.5)
				SWITCHTHATBLOWSUPTHEWORLD=false;
		
			else
				SWITCHTHATBLOWSUPTHEWORLD=true;
		}

		//Check for Renee
		isRenee = false;
		
		//COCK
		if (!m_robotDrive->m_SoccerBot->IsCocked()&&!autoshoot_task_started)
		{
			m_robotDrive->m_SoccerBot->m_shooter->Set(-.8);
		}
		else if (!autoshoot_task_started)
		{
			m_robotDrive->m_SoccerBot->m_shooter->Set(0);
		}
		
		ballsKicked = 0;//makes autonomous repeatable

		if (m_telePeriodicLoops == 0)
		{
			cout << "Entering first TeleopPeriodic Loop" << endl;
			m_robotDrive->m_SoccerBot->m_hookerTimer->Reset();
		}
		
		
		// increment the number of periodic loops completed
		m_telePeriodicLoops++;
		
		
		// increment DS packets received
		m_dsPacketsReceivedInCurrentSecond++;
		
		//Dashboard Stuff
///		m_Dashboard->sendIOPortData(false, m_robotDrive->getHorizontalAngle(),
///				(float)GetTension(), 0, tensionSetPoint,
///				m_robotDrive->getRange(), !isRenee, (float)zone,
///				isBFinneyInverted, GTFO);
		
		//tension setpoint
		if (isJeffTension)
		{
			if (GetTension() < tensionSetPoint - 3)		{m_robotDrive->m_SoccerBot->TestTensioner(1);} 
			else if (GetTension() > tensionSetPoint + 3){m_robotDrive->m_SoccerBot->TestTensioner(-1);}
		}
		
		//Lifter stuff
		if (m_robotDrive->m_SoccerBot->m_hookerTimer->Get() < 100)
		{
			m_robotDrive->LatchHooker();
		}
				
		/* BUTTON MAPPING - DRIVE STICK (Brandon or MattZ)
		 * joysticks: drive
		 * Lclick:
		 * Rclick:
		 * LB: Spin 180 left & invert
		 * RB: Spin 180 right & invert
		 * LT: half power
		 * RT:
		 * A: enable spin mode, break out of rotatexdegrees (doesn't uninvert)
		 * B: 
		 * X: AutoAim
		 * Y:
		 * Start:
		 * Select:
		 */

		//Joysticks drive the robot if not in spin mode -- must check for inverted  & slow modes
		if (!DriveAPressed)
		{
			if (!isBFinneyInverted)
			{
				if (/*!isBFinneySlow this was just taken out 6/5/10 mz*/ true)
					m_robotDrive->m_SoccerBot->TankDrive(m_ControllerDrive->GetLeftY(), m_ControllerDrive->GetRightY());
				else
					m_robotDrive->m_SoccerBot->TankDrive(m_ControllerDrive->GetLeftY() * .6, m_ControllerDrive->GetRightY() * .6);
			} 
			else
			{
				if (!isBFinneySlow)
					m_robotDrive->m_SoccerBot->TankDrive(m_ControllerDrive->GetRightY() * -1, m_ControllerDrive->GetLeftY() * -1);
				else
					m_robotDrive->m_SoccerBot->TankDrive(m_ControllerDrive->GetRightY() * -.6, m_ControllerDrive->GetLeftY() * -.6);
			}
		}
		
		//LeftClick not used
		if (m_ControllerDrive->GetLeftClick())
		{
			
		}
		
		//RightClick not used
		if (m_ControllerDrive->GetRightClick())
		{
			
		}
		if (!autoshoot_task_started&&m_robotDrive->m_SoccerBot->IsCocked())
		{
			m_robotDrive->m_SoccerBot->m_shooter->Set(0);
		}
		
		//Left Bumper spins robot to the left 180 QUICKLY! (break loop by pressing A)
		if (m_ControllerDrive->GetLeftBumper())
		{
			//isBFinneyInverted = !isBFinneyInverted;
			m_robotDrive->m_SoccerBot->RotateXDegrees(180, LEFT, m_robotDrive->m_gyro, true);
		}
		
		//Right Bumper spins robot to the right 180 QUICKLY! (break loop by pressing A)
		if (m_ControllerDrive->GetRightBumper())
		{
			//isBFinneyInverted = !isBFinneyInverted;
			m_robotDrive->m_SoccerBot->RotateXDegrees(180, RIGHT, m_robotDrive->m_gyro, true);
		}
		
		//THIS WAS JUST COMMENTED OUT 6/5/10 mz
		//Left Trigger slows down robot
		/*if (m_ControllerDrive->GetTrigger() > .2 && !DriveAPressed)
		{
			isBFinneySlow = true;
		} 
		else if (!DriveAPressed)
			isBFinneySlow = false;*/
		
		isBFinneySlow=false;
		
		
		//Press A to enable spin-mode OR BREAK ROTATEXDEGREES
		//@depreciated: i dont think bfinney likes to use spin
		if (m_ControllerDrive->GetA()&&!DriveAPressed)
		{
			DriveAPressed=true;
			//This still works even if inverted
			m_robotDrive->m_SoccerBot->Spin(m_ControllerDrive->GetTrigger(), LEFT);
		} 
		else if (!m_ControllerDrive->GetA())
			DriveAPressed=false;
		
		//Button B Not Used
		if (m_ControllerDrive->GetB())
		{
			
		}
		
		//Button X used to aim
		//@depreciated: not used!
		if (m_ControllerDrive->GetX())
		{
			/*pidena=true;
			m_robotDrive->ZeroCamera();
			m_robotDrive->Aim();
		} 
		else
		{
			m_robotDrive->StopAim();
			pidena=false;*/
		}
		
		//Button Y not used
		if (m_ControllerDrive->GetY())
		{
			
		}
		
		//Start not used
		if (m_ControllerDrive->GetStart())
		{
			
		}
		if (!autoshoot_task_started&&m_robotDrive->m_SoccerBot->IsCocked())
		{
			m_robotDrive->m_SoccerBot->m_shooter->Set(0);
		}
		 //Jeff is Shooting
		{
			/* BUTTON MAPPING - SHOOT STICK JEFF (boom headshot)
			 * joysticks: left joy Y controls lifter
			 * Lclick: release mah hooker! (can also be done with the switch that blows up the world)
			 * Rclick: 
			 * LB: decrease tension
			 * RB: increase tension
			 * LT: shoots continuously //or is this done with RT?
			 * RT: 
			 * A:
			 * B: Zone 3 Tension
			 * X: Zone 1 Tension
			 * Y: Zone 2 Tension
			 * Start: enable Autoshoot task
			 * Select: disable Autoshoot task
			 */

			//Left Joystick used for lifting (if servo is released)
			//Right Joystick not used
			if (m_robotDrive->m_SoccerBot->m_hookerTimer->Get() > 100)
			{
				if (m_ControllerShoot->GetLeftY()<-.3)
					m_robotDrive->Lift(m_ControllerShoot->GetLeftY());
				else if (m_ControllerShoot->GetLeftY()>.3)
				{
					m_robotDrive->Lift(m_ControllerShoot->GetLeftY());
				} else
					m_robotDrive->Lift(0);
				
			} 			
						
			//Left click releases the hooker (as does the missle switch)
			if(havecyp&&SWITCHTHATBLOWSUPTHEWORLD)
			{
				m_robotDrive->ReleaseHooker();
			}
			if(!havecyp&&m_ControllerShoot->GetLeftClick())
			{
				m_robotDrive->ReleaseHooker();
			}
			/*
			if (m_ControllerShoot->GetLeftClick()||SWITCHTHATBLOWSUPTHEWORLD)
			{
				m_robotDrive->ReleaseHooker();
				//if(havecyp)
				//	m_ds->GetEnhancedIO().SetLEDs(4);//@depreciated: only used for debugging
			} 
			else
			{
				//if(havecyp)
				//	m_ds->GetEnhancedIO().SetLED(3, false);//@depreciated: only used for debugging
			}*/
			
			//Right click not used
			if (m_ControllerShoot->GetRightClick())
			{

			}			
			
			//RIGHT BUMPER increases tension, LEFT BUMPER decreases it
			if (m_ControllerShoot->GetRightBumper())
			{
				isJeffTension = false;
				m_robotDrive->m_SoccerBot->TestTensioner(1);
			} 
			else if (m_ControllerShoot->GetLeftBumper())
			{
				isJeffTension = false;
				m_robotDrive->m_SoccerBot->TestTensioner(-1);
			} 
			else if (!isJeffTension)
			{
				m_robotDrive->m_SoccerBot->TestTensioner(0);
			}			
			
			//Triggers control the shooter - left shooooooooots //@depreciated: or is it the right?
			if (m_ControllerShoot->GetTrigger()!= 0)
			{
				if (m_ControllerShoot->GetTrigger() < -.3) //RIGHT TRIGGER?
				{
					if (autoshoot_task_started)
						autoshoot->Stop();
					cout << "Autoshoot Terminated" << endl;
					autoshoot_task_started=false;
					m_robotDrive->m_SoccerBot->TestShooter(-1);
				} 
				else
				{
					m_robotDrive->m_SoccerBot->TestShooter(0);
				}
			}
			
			//Button A not used
			if (m_ControllerShoot->GetA())
			{
				
			}
			
			//Button B sets tension to zone 3
			if (m_ControllerShoot->GetB())
			{
				isJeffTension = true;
				tensionSetPoint = 100;
			}
			
			//Button X sets tension to zone 1
			if (m_ControllerShoot->GetX())
			{
				isJeffTension = true;
				tensionSetPoint = 20;
			}
			
			//Button Y sets tension to zone 2
			if (m_ControllerShoot->GetY())
			{
				isJeffTension = true;
				tensionSetPoint = 80;
			}
			
			//Start enables the autoshoot task
			if (m_ControllerShoot->GetStart() && !autoshoot_task_started)
			{
				cout << "Starting the Autoshooter" << endl;
				m_robotDrive->m_SoccerBot->TestShooter(0);
				autoshoot_task_started = autoshoot->Start((UINT32)m_robotDrive, (UINT32)m_ballSensor);
			}
			
			//Select disables the autoshoot task
			if (m_ControllerShoot->GetSelect() && autoshoot_task_started)
			{
				cout << "Terminating the Autoshooter" << endl;
				m_robotDrive->m_SoccerBot->TestShooter(0);
				autoshoot_task_started = false;
				autoshoot->Stop();
			}
		}
		if (!autoshoot_task_started&&m_robotDrive->m_SoccerBot->IsCocked()&&m_ControllerShoot->GetTrigger()== 0)
		{
			m_robotDrive->m_SoccerBot->m_shooter->Set(0);
		}
		
		/* BUTTON MAPPING - TEST STICK (for debugging purposes)
		 * Stick:
		 * Trigger: cout Button Mapping :D
		 * Button 2: cout GetTension()
		 * Button 3: cout Absolute Encoder and Relative Encoder values
		 * Button 4: cout hooker value
		 * Button 5: increase hooker servo
		 * Button 6: decrease hooker servo
		 * Button 7: cout camera servo values
		 * Button 8: 
		 * Button 9: cout GetGyroAngle()
		 * Button 10: reset Gyro
		 * Button 11: 
		 * Button 12: 
		 */

		//Trigger outputs button configuration for this joystick
		if (m_ControllerTest->GetTrigger())
		{
			printf("Current button mapping consists of the following:\nThrottle: lift\nStick:\nTrigger: cout Button Mapping :D\nButton 2: cout GetTension()\nButton 3: cout Absolute Encoder and Relative Encoder values\nButton 4: cout hooker servo\nButton 5: increase hoooker servo\nButton 6: decrease hooker servo\nButton 7: cout camera servo values\nButton 8: \nButton 9: cout GetGyroAngle()\nButton 10: reset Gyro\nButton 11: \nButton 12: \n");
		}
		
		//Button 2
		if (m_ControllerTest->GetRawButton(2))
		{
			cout << "Tension is at " << GetTension() << endl;
		}
		
		//Button 3
		if (m_ControllerTest->GetRawButton(3))
		{
			m_robotDrive->m_SoccerBot->ReadAbsoluteEncoder();
			m_robotDrive->m_SoccerBot->ReadRelativeEncoder();
			cout << endl;
		}
		//.6945-forward
		//.8385
		
		//Button 4
		if (m_ControllerTest->GetRawButton(4))
		{
			cout << "Tilt set to " << m_robotDrive->m_SoccerBot->m_tiltServ->GetAngle() << " degrees\n";
		}
				
		//Button 5
		if (m_ControllerTest->GetRawButton(5))
		{
			m_robotDrive->m_SoccerBot->m_tiltServ->SetAngle(m_robotDrive->m_SoccerBot->m_tiltServ->GetAngle() + 1);
		}
				
		//Button 6
		if (m_ControllerTest->GetRawButton(6))
		{
			m_robotDrive->m_SoccerBot->m_tiltServ->SetAngle(m_robotDrive->m_SoccerBot->m_tiltServ->GetAngle() - 1);
		}
				
		//Button 7
		if (m_ControllerTest->GetRawButton(7))
		{
			cout << "Camera pan set to "
					<< m_robotDrive->m_SoccerBot->m_panServ->GetAngle()
					<< " degrees\n";
			cout << "Camera tilt set to "
					<< m_robotDrive->m_SoccerBot->m_tiltServ->GetAngle()
					<< " degrees\n";
		}
		 
		//Button 8
		if (m_ControllerTest->GetRawButton(8))
		{
			
		}
		 
		//Button 9
		if (m_ControllerTest->GetRawButton(9))
		{
			cout << "Gyro at " << m_robotDrive->getGyroAngle() << " degrees\n";
		}
		 
		//Button 10
		if (m_ControllerTest->GetRawButton(10))
		{
			m_robotDrive->m_gyro->Reset();
		}
		 
		//Button 11
		if (m_ControllerTest->GetRawButton(11))
		{
			cout<<m_ControllerShoot->GetLeftY()<<"\n";
		}
				
		//Button 12
		if (m_ControllerTest->GetRawButton(12))
		{
			
		}
	}
	
	/********************************** Continuous Routines *************************************/

	/*
	 void AutonomousContinuous(void)	
	 {
	 }
	 
	 void DisabledContinuous(void)
	 {
	 }

	 void TeleopContinuous(void) 
	 {
	 }
	 */

	/********************************** Miscellaneous Routines *************************************/

};

START_ROBOT_CLASS(MainRobotClass)
;



















































































































































































































































































































































































































































































































//GO TEAM 2619 THE CHARGE!!!
