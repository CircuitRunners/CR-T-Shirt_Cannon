package org.github.CircuitRunners;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

public class TShirtCannon extends IterativeRobot {

    // # Constants
	// ## Ports
    // Motor Controller Ports
    public static final int JAGUAR_DRIVE_PORT_FL = 1;
    public static final int JAGUAR_DRIVE_PORT_FR = 2;
    public static final int JAGUAR_DRIVE_PORT_RR = 3;
    public static final int JAGUAR_DRIVE_PORT_RL = 4;

    public static final int TALON_WINCH_PORT = 5;

    // Relay Port
    public static final int SPIKE_LAUNCH_PORT = 1;

    // Solenoid Ports
    public static final int SOLENOID_LEAK_PORT = 1;
    public static final int SOLENOID_LOAD_IN_PORT = 2;
    public static final int SOLENOID_LOAD_OUT_PORT = 3;
	
	// ## Controller
    // Special Joystick Values
    public static final int TRIGGER = 1;
    public static final int AXIS_THROTTLE = 4;
    /* public static final int HAT_HORIZONTAL = 5;
    public static final int HAT_VERTICAL = 6; */

    // Xbox Button Map
    /* public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_LB = 5;
    public static final int BUTTON_RB = 6;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;
    public static final int BUTTON_LS = 9;
    public static final int BUTTON_RS = 10; */

    // ## Constant Speeds
    public static final double WINCH_SPEED = 0.4;

    // ## Number of Buttons
    // public static final int BUTTONS = 16;
    
    // # Objects
    // Motor Contollers
    Jaguar drive1;
    Jaguar drive2;
    Jaguar drive3;
    Jaguar drive4;
    
    Talon winch;
    
    // Robot Drive
    RobotDrive drive;
    
    // Spike
    Relay launch;
    
    // Solenoid
    Solenoid leak;
    Solenoid in;
    Solenoid out;
    
    // Joystick
    Joystick joystick;
    
    // Driver Station
    // DriverStation ds;
    
    @Override
    public void robotInit() {
        // Construct Talons
        drive1 = new Jaguar(JAGUAR_DRIVE_PORT_FL);
        drive2 = new Jaguar(JAGUAR_DRIVE_PORT_RL);
        drive3 = new Jaguar(JAGUAR_DRIVE_PORT_FR);
        drive4 = new Jaguar(JAGUAR_DRIVE_PORT_RR);
        
        winch = new Talon(TALON_WINCH_PORT);
        
        drive = new RobotDrive(drive1, drive2, drive3, drive4);
        drive.setInvertedMotor(MotorType.kFrontRight, true);
        drive.setInvertedMotor(MotorType.kRearRight, true);
        
        // Construct Relay
        launch = new Relay(SPIKE_LAUNCH_PORT);
        
        // Construct Solenoid
        leak = new Solenoid(SOLENOID_LEAK_PORT);
        in = new Solenoid(SOLENOID_LOAD_IN_PORT);
        out = new Solenoid(SOLENOID_LOAD_OUT_PORT);
        
        // Construct Joystick
        joystick = new Joystick(1);
        
        //RobotDrive safety
        drive.setSafetyEnabled(true);
        
        //Driver Station Instance
        // ds = DriverStation.getInstance();
    }

    // Called periodically while in teleop mode
    public void teleopPeriodic() {
        // Get joystick values
        double joystick_X = joystick.getX();
        double joystick_Y = joystick.getY();
        double joystick_t = joystick.getTwist();
        //double joystick_ang = joystick.getDirectionDegrees();
        double joystick_mag = joystick.getMagnitude();

        double ratioValue = (-joystick.getRawAxis(AXIS_THROTTLE) + 1) / 2;

        if (joystick_mag <= 0.1) {
            joystick_X = 0;
            joystick_Y = 0;
        }

        if (joystick_t <= 0.1 || joystick_t >= -0.1) {
            joystick_t = 0;
        }

        drive.mecanumDrive_Cartesian(ratioValue * -joystick_X, ratioValue * joystick_Y, ratioValue * joystick_t, 0);

        // Winch
        winch.set(joystick.getRawButton(4) ? -WINCH_SPEED : joystick.getRawButton(6) ? WINCH_SPEED : 0);

        // Shooter
        // Launcher
        launch.set(joystick.getRawButton(TRIGGER) ? Relay.Value.kForward : Relay.Value.kOff);

        // Dump Air
        leak.set(joystick.getRawButton(2) != leak.get());

        // Reload
        in.set(joystick.getRawButton(3) || (!joystick.getRawButton(5) && in.get()));
        out.set(!joystick.getRawButton(3) && (joystick.getRawButton(5) || out.get()));
    }
    
    /*public void print() {
        // Driver Station LCD Output
        dsLCD.println(DriverStationLCD.Line.kUser1, 1, "Joystick X: " + joystick.getX() + "                 ");
        dsLCD.println(DriverStationLCD.Line.kUser2, 1, "Joystick Y: " + joystick.getY() + "                 ");
        dsLCD.println(DriverStationLCD.Line.kUser3, 1, "Joystick t: " + joystick.getTwist() + "                 ");
        dsLCD.println(DriverStationLCD.Line.kUser4, 1, "Ratio Value: " + (joystick.getThrottle()+1)/2 + "                 ");
        dsLCD.println(DriverStationLCD.Line.kUser5, 1, "Joystick Mag: " + joystick.getMagnitude() + "                 ");
        dsLCD.println(DriverStationLCD.Line.kUser6, 1, "Joystick Angle: " + joystick.getDirectionDegrees() + "                 ");
        dsLCD.updateLCD();
    }*/
}
