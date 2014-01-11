#include "WPIlib.h"

/* Some constants that define what buttons on the 360
 * map to what. */

const short ButtonA = 1;
const short ButtonB = 2;
const short ButtonX = 3;
const short ButtonY = 4;
const short LeftBumper = 5;
const short RightBumper = 6;
const short Select = 7;
const short Start = 8;
const short LeftClick = 9;
const short RightClick = 10;

const short LeftXAxis = 1;
const short LeftYAxis = 2;
const short TriggerAxis = 3;
const short RightXAxis = 4;
const short RightYAxis = 5;

class Object360
{
private: Joystick *m_Controller360;
	
	public: Object360(int port)
	{
		/* Makes an instance of a new joystick that our
		 * 360 controller will use. Is in USB port 1.
		 * Has 6 axiseses and has 12 buttons (only 10
		 * buttons are actually on the 360 controller) */
		m_Controller360 = new Joystick(port,6,12);
	}
	
	public: bool GetRawButton(int button)
	{
		/*Gets the raw button state from the joystick class.
		true is press, false is unpressified */
		return m_Controller360->GetRawButton(button);
	}
	
	
	/* Easy functions to call to get the value of stuff. */
	public: bool GetA() { return GetRawButton(1); }
	public: bool GetB() { return GetRawButton(2); }
	public: bool GetX() { return GetRawButton(3); }
	public: bool GetY() { return GetRawButton(4); }
	public: bool GetRightBumper() { return GetRawButton(6); }
	public: bool GetLeftBumper() { return GetRawButton(5); }
	public: bool GetStart() { return GetRawButton(8); }
	public: bool GetSelect() { return GetRawButton(7); }
	public: bool GetLeftClick() { return GetRawButton(9); }
	public: bool GetRightClick() { return GetRawButton(10); }
	
	// (0,1] is right, [-1,0) is left. Range [-1,1]
	public: float GetLeftX() { return GetAxis(1); } 
	// (0,1] is down, [-1,0) is up. Range [-1,1] 
	public: float GetLeftY() { return GetAxis(2); }
	/* Both triggers are in this. 
	 * Right Trigger is [-1,0]
	 * Left Trigger is [0,1]
	 * Return value = Right Trigger + Left Trigger */
	public: float GetTrigger() { return GetAxis(3); }	
	// (0,1] is right, [-1,0) is left. Range [-1,1]
	public: float GetRightX() { return GetAxis(4); }
	// (0,1] is down, [-1,0) is up. Range [-1,1] 
	public: float GetRightY() { return GetAxis(5); }
	
	public: float GetAxis(int axis)
	{
		/*get the axis state from the joystick class
		negative on x is left, positive is right
		negative on y is up, positive is down */
		return m_Controller360->GetRawAxis(axis);
	}
	

};
