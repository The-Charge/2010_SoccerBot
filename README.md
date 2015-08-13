2010_SoccerBot
==============

Our 2010 SoccerBot built for WindRiver 2014 and stuffed into GitHub

Version 1.0.0
Edit: none
SVN Revision: 3
Just a starting point, no big changes
*************************************
Version 1.1.0
Edit: Jeff R
SVN Revision: 9
3:00 pm 2/6/10
* Gutted out all erroneous default code
* Added in wireframe of each class
* Still no member variables
* probably doesn't compile
* needs a lot of work\
* deleted "sample.txt" which explains how version 1.0.0 works
************************************
Version 1.2.0
Edit: Jeff R
SVN Revision: 10
3:45 pm 2/6/10
* Renamed MainRobotLogic to MainRobotClass
* Added the following global constants:
  * PWM Output channels
  * Digital Input channels
  * Analog Input channels
  * Directions (Forward, Backward, Left, Right)
  * Calibration variables - all random guesses
* Added member variables to classes
  * Gyro
  * Encoders
  * Absolute Encoders
  * Motor Controllers
  * SoccerBot
 * Added AbsoluteEncoder class for handling of Absolute Encoders
 * Added ClassName:: to the front of each member function
 * Added TODOs
 ***************************************
 Version 1.3.0
 Edit: Jeff R
 SVN Revision: 11
 4:30 pm 2/6/10
 * Implemented SoccerBot constructer
 * Implemented SoccerBot::DriveXFeet
 * Implemented SoccerBot::RotateXDegrees
 * Implemented SoccerBot::Spin
 * Implemented SoccerBot::SetLeftRightMotorSpeeds
 * Added Global constants as necessary to implement above functions
 ***************************************
 Version 1.3.1
 Edit: Jeff R
 SVN Revision: 12
 5:00 pm 2/6/10
 * Compiled
 * Added Several autonomous functions (commented out)
 * commented out AbsoluteEncoders
***************************************
Version 1.4.0
Edit: Adam Werries
SVN Revision 14
5:20 pm 2/6/2010
*compiled
*added code for 360 controller
	*variables in MainRobotClass necessary to init
	*included 360Object.h
*added code for Arcade/Tank drive and switching
	*necessary variables added in MainRobotClass
	*if statements in TeleopPeriodic to add
	switching between arcade and tank drive modes
	using the A button
	*then drives based on choice

***************************************
Version 1.4.1
Edit: Adam Werries
SVN Revision 15
6:00 pm 2/6/2010
*compiled
*implemented tankdrive in SoccerBot
*nothing is actually tested on robot yet.
***************************************
Version 1.5.0
Edit: Jeff R
SVN Revision: 19
___ pm 2/7/10
* Calculated the proper ENCODER_TO_FEET_RATIO using values from mr. matt
* Added controller logic to demonstrate SoccerBot::Spin
* Removed commented out autonomous (exists as Jeff's branch
* Deleted commented out Arcade/Tank code
* Added Comments to SoccerBot
* Added button inputs that will output encoder values
* Added couts to the constructors and init routines
* This version will be tested on the robot!!!
***************************************
Version 1.5.1
Edit: Adam W
SVN Revision: 22
5:16pm 2/7/2010
*make jeff's previous revision compile
	*commented out RotateXDegrees, gyro not implemented yet
	
***************************************
Version 1.6.0
Edit: Adam W
SVN Revision: 23
7:50pm 2/11/2010
*integrated AbsoluteEncoder from other branch
	*Matt's code, only really used GetRawVoltage though
*added readouts for the two Absolute Encoders
	*used GetY for Xbox controller to turn on reading
*added control of motors for shooter/tensioner with triggers/bumpers
	*triggers control Shooter, if right, forward .3, if left, backwards -.3, if none
	set to 0
	*bumpers control tensioner, if right, forward .05, if left, backwards -.05,
	if none set to 0
	
****************************************
****************************************
Version 1.6.5
Edit: Adam W
SVN Revision: 35
4:43pm 2/12/2010
*added CockShooter function, full power
	*found value using the ReadAbsoluteEncoder
	*set range
	*full power, its accurate/fast enough
*right trigger fire, left trigger Cock
	*RIGHT TRIGGER IS [-1,0], LEFT IS [0,1]
	*SHOOTER MOTOR NEEDS NEGATIVE VALUE
*added IterativeRobot.cpp (edited) to fix printf's all the time
	*can't test values otherwise :(
*added DashboardDataSender to enable dashboard (from Matt's code)
	*to read optical. wasn't working though.
*****************************************
Version 1.6.7
Edit: Adam W
SVN Revision: 36
7:37pm 2/12/2010
*made Cockshooter automatic
	*moves to cock position if triggers aren't being messed with
*right trigger shoots
	*moves forward the shooter, can actually keep shooting indefinitely
	without needing to stop to cock
	*no longer sends the trigger value. just sends -1
******************************************
*****************************************
Version 1.6.8
Edit: Adam W
SVN Revision: 38
7:55pm 2/12/2010
*re-added calls and handles for Arcade/Tank
	*uses A
*****************************************
Version 2.0
Edit: Adam W
SVN Rev: 70
7:20 2/15/2010
*added a crapload since last documentation....
*auto-shooter when ball is in the way while holding R trigger
	*voltage of optical sensor > 3 or something
*some acceleration code was added to keep robot from falling over
	*not FULLY done yet
*autonomous code started
	*commented out portion is the drive forward and hit stuff function
	*need to test DriveXFeet
*software limits for tensioner added
	*should be automated and run at init
*fixed cockpoint
	*also should be automated and have a correction ability
*probably some other stuff BUT NO ONE ELSE UPDATES THIS.
I WILL KILL ALL OF YOU.
	*seriously
*****************************************
Version 2.1
Edit: Adam W
SVN Rev: 73
8:02 2/18/2010
*added support for both controllers, one being for driving, two being for shooting
	*all driving required stuff on first
	*everything else (debugging too) on 2nd controller
*fixed some encoder stuff!
	*encoder to feet ratio along
	*reversed channel
	*****************************************
Version 2.2
Edit: Adam W
SVN Rev: 84
8:29 2/18/2010
*made cock function its own task, to not interrupt anything :)
	*named Cock! :)
*added sawyers auto code, edited with drivexfeet
*****************************************
Version 2.3 
edit:Matthew Z
SVN Rev: 85
6:58 2/19/2010

added code that impliments: 
Zoidberg::getRange() takes in a Target only 1 target though
Zoidberg::startCamera() it starts up the camera for image streaming, brightness 30 resolution max, whitebalance automatic
Zoidberg::findElip() it starts the task FRC_detect using the function findelpises() which detects elipses(targets)and also outputs data on the targets
Zoidberg::getServoAngle()
Zoidberg::getGyroAngle() 
Zoidberg::getHorizontalAngle();
both of these return angles of the respective items note that the servo angle returned is that of the pan servo

Other Changes:
copied and renamed AxisCamera.h to AxisCamera2.h (we access the camera more than once
included 
"Vision/AxisCamera.h"
"AxisCamera2.h"
"Target.h"
as well as adding them to the project

created global variables: 
Task *detect;
DashboardDataSender *m_Dashboard;
modified values in target.cpp
min match score from 500 to 980
threshold from 40 to 56 
****************************************
Version 2.4
Edit: Jeff R and Matt Z (kinda)
SVN Rev: 87ish
2:00pm 2/20/10

FINALLY CLEANED THE CODE!
* removed unnecessary commented out code
* added several depreciations
* edited the TODO.txt - LOOK AT IT!
* added block comments for functions/classes i understand
* renamed 'lemon' to a more meaningful name
* fixed indentation and whitespaces
* trying to prepare robot for a tag

Did Work
* added, then commented out, autonomous code for reading Cyprus
* fixed autocock and autoshoot tasks
*********************************
Version 2.5
Edit: Jeff R and Matt Z
SVN Revision: 93ish
5:30pm 2/20/10

Continued Cleaning
* Reordered stuff within Teleop Periodic
* added even more comments

Did Work
* Autonomous creeps up to and shoots a ball
* Fixed Dashboard Data Packer
* Preparing to TAG

*********************************
Version 3.0
Edit: Adam Werries
SVN Revision: 119ish
7:18pm 3/1/2010
* integrated camera aim branch w/ PID
*integrated camera zero function
******************************************
Version 3.2
Edit: Adam Werries
SVN Revision: 133
7:45pm 3/3/2010
*fixed channels for some servos/motors (global constants)
	*servos were off. so was lifter stuff
*fixed arcade to be...better...less sensitive
	*it was seriously borked. 
*cleaned a LOT
	*yay code conventions
*RotateXDegrees fixed immensely
	*has interrupts and is accurate and has speed mode
*DriveXFeet cleaned
	*it was a horrible mess
*removed Shoot()
	*didnt really work
*added testtensioner limits and speed adjusts
	*no more values randomly in
*added ReadRelativeEncoder
	*separate function. used to be in MainRobotClass
*IsCocked and IsUnCocked added many many revisions ago
	*yeah. they check if its cocked or not. not too complicated
*PIDOutput moved around and fixed. we can move forward while turning auto
	*the call to arcade drive includes values from the controller. that took some fancy
		point control
*basic Lift function added...5 minutes from now
	*must be tested. shoooould work.
*PID has decent functionality, Aim works decently
	*works well enough! jittery but we dont have time to fix it.
*ZeroCamera is in...only kinda maybe works.  --added some fixes
	*should have been fixed, it was doing strange jerky things before
*lots of GetAngle functions for various motors/sensors
	*we can get the angle of EVERYTHING. MWHAHAHA. its all in zoidberg.
*DashboardDataSender all fixed
	*was doing gross things
*autocock function all goooooone. caused problems with no benefit :(
	*it was doing scary mind control with the motors...evillllll
*test joystick added with button mappings
	*only for debugging sensors and the like. very niiiice.
*GetTension function added
	*checks tension. this also appears on dashboard
*autoshoot function moved around and such
	*called in a different place. still used though :)
*button mappings changed, see ButtonMappings.txt
	*not gonna bother explaining this. check the file.
*********************************************
	