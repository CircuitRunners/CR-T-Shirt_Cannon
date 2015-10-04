package org.github.CircuitRunners;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Watchdog;
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;

public class TShirtCannon extends SimpleRobot {
   
    //constants
	//Ports
	{
		//Motor Controller Ports
		public static final int JAGUAR_DRIVE_PORT_FL = 1;
		public static final int JAGUAR_DRIVE_PORT_FR = 2;
		public static final int JAGUAR_DRIVE_PORT_RR = 3;
		public static final int JAGUAR_DRIVE_PORT_RL = 4;
		
		public static final int TALON_WINCH_PORT = 5;
		
		//Relay Port
		public static final int SPIKE_LAUNCH_PORT = 1;
		
		//Solenoid Ports
		public static final int SOLENOID_LEAK_PORT = 1;
		public static final int SOLENOID_LOAD_IN_PORT = 2;
		public static final int SOLENOID_LOAD_OUT_PORT = 3;
    }
	
	//Controller
	{
		//Special Joystick Values
		public static final int TRIGGER = 1;
		public static final int AXIS_THROTTLE = 4;
		public static final int HAT_HORIZONTAL = 5;
		public static final int HAT_VERTICAL = 6;
		
		//Xbox Button Map
		public static final int BUTTON_A = 1
		public static final int BUTTON_B = 2
		public static final int BUTTON_X = 3
		public static final int BUTTON_Y = 4
		public static final int BUTTON_LB = 5
		public static final int BUTTON_RB = 6
		public static final int BUTTON_BACK = 7
		public static final int BUTTON_START = 8
		public static final int BUTTON_LS = 9
		public static final int BUTTON_RS = 10
    }
	
	//Other Contants
	{
		//Constant Speeds
		public static final double WINCH_SPEED = 0.4;
		
		//Number of Buttons
		public static final int BUTTONS = 16;
		
		//Pi
		public static final double PI = Math.PI;
	}
    
	//Objects
    //Motor Contollers
    Jaguar drive1;
    Jaguar drive2;
    Jaguar drive3;
    Jaguar drive4;
    
    Talon winch;
    
    //Robot Drive
    RobotDrive drive;
    
    //Spike
    Relay launch;
    
    //Solenoid
    Solenoid leak;
    Solenoid in;
    Solenoid out;
    
    //Joystick
    Joystick joystick;
    
    //Driver Station
    DriverStation ds;
    
    //Driver Station LCD
    DriverStationLCD dsLCD;
    
    //Watchdog
    Watchdog dog;
    
    public TShirtCannon(){        
        //Construct Talons
        drive1 = new Jaguar(JAGUAR_DRIVE_PORT_FL);
        drive2 = new Jaguar(JAGUAR_DRIVE_PORT_RL);
        drive3 = new Jaguar(JAGUAR_DRIVE_PORT_FR);
        drive4 = new Jaguar(JAGUAR_DRIVE_PORT_RR);
        
        winch = new Talon(TALON_WINCH_PORT);
        
        drive = new RobotDrive(drive1, drive2 ,drive3, drive4);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        
        //Construct Relay
        launch = new Relay(SPIKE_LAUNCH_PORT);
        
        //Construct Solenoid
        leak = new Solenoid(SOLENOID_LEAK_PORT);
        in = new Solenoid(SOLENOID_LOAD_IN_PORT);
        out = new Solenoid(SOLENOID_LOAD_OUT_PORT);
        
        //Construct Joystick
        joystick = new Joystick(1);
        
        //The Watchdog is born!       
        dog = Watchdog.getInstance();
        dog.setExpiration(0.5);
        
        //RobotDrive safety
        drive.setSafetyEnabled(true);
        
        //Driver Station Instance
        ds = DriverStation.getInstance();
        
        //Driver Station LCD Instance
        dsLCD = DriverStationLCD.getInstance();
        
    }
    
    //Called once each Teleop mode
    public void operatorControl() {
    
        //While in test mode
        while(isOperatorControl()){
            
            //Drive
            //Watchdog
            dog.feed();
            
            //Get joystick values
            double joystick_X = joystick.getX();
            double joystick_Y = joystick.getY();
            double joystick_t = joystick.getTwist();
            double joystick_ang = joystick.getDirectionDegrees();
            double joystick_mag = joystick.getMagnitude();
            
			double ratioValue = (-joystick.getRawAxis(AXIS_THROTTLE) + 1) / 2;
			
            drive.mecanumDrive_Cartesian(ratioValue * -joystick_X, ratioValue * joystick_Y, ratioValue * joystick_t, 0);
                       
            //Winch
            winch.set(joystick.getRawButton(4) ? -WINCH_SPEED : joystick.getRawButton(6) ? WINCH_SPEED : 0);
            
            //Shooter
            //Launcher
            launch.set(joystick.getRawButton(TRIGGER) ? Relay.Value.kForward : Relay.Value.kOff);
            
            //Dump Air
            leak.set(joystick.getRawButton(2) ? !leakState);
            
            //Reload
			boolean inState = joystick.getRawButton(3) ? true : joystick.getRawButton(5) ? false : inState;
			boolean outState = joystick.getRawButton(3) ? false : joystick.getRawButton(5) ? true : outState;
			in.set(inState);
			out.set(outState);
            
            print();
            
        }
    }
    
    public void print() {
        // Driver Station LCD Output
            dsLCD.println(DriverStationLCD.Line.kUser1, 1, "Joystick X: " + joystick.getX() + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser2, 1, "Joystick Y: " + joystick.getY() + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser3, 1, "Joystick t: " + joystick.getTwist() + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser4, 1, "Ratio Value: " + ratioValue() + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser5, 1, "Joystick Mag: " + joystick.getMagnitude() + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser6, 1, "Joystick Angle: " + joystick.getDirectionDegrees() + "                 ");
            dsLCD.updateLCD();
    }
}	