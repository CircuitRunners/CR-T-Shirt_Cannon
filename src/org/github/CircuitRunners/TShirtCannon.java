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
    
    //Special Joystick Values
    public static final int TRIGGER = 1;
    public static final int AXIS_THROTTLE = 4;
    public static final int HAT_HORIZONTAL = 5;
    public static final int HAT_VERTICAL = 6;
    
    //Constant Speeds
    public static final double WINCH_SPEED = 0.4;
    
    //Deadband
    public static final double DEADBAND_HIGH = 0.25;
    public static final double DEADBAND_LOW = -0.25;
    
    //Number of Buttons
    public static final int BUTTONS = 16;
    
    //Pi
    public static final double PI = Math.PI;
    
    //Motor Contollers
    Jaguar drive1;
    Jaguar drive2;
    Jaguar drive3;
    Jaguar drive4;
    
    Jaguar winch;
    
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
    
    //Button array
    boolean[] buttons;
    
    //Driver Station
    DriverStation ds;
    
    //Driver Station LCD
    DriverStationLCD dsLCD;
    
    //Watchdog
    Watchdog dog;
    
    TShirtCannon(){        
        //Construct Talons
        drive1 = new Jaguar(JAGUAR_DRIVE_PORT_FL);
        drive2 = new Jaguar(JAGUAR_DRIVE_PORT_RL);
        drive3 = new Jaguar(JAGUAR_DRIVE_PORT_FR);
        drive4 = new Jaguar(JAGUAR_DRIVE_PORT_RR);
        
        winch = new Jaguar(TALON_WINCH_PORT);
        
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
            
            if (ds.getDigitalIn(1)) additionDrive(joystick);
            else if (ds.getDigitalIn(2)) drive.mecanumDrive_Cartesian(joystick_X, joystick_Y, joystick_t, 0);
            else if (ds.getDigitalIn(3)) drive.mecanumDrive_Polar(joystick_mag, joystick_ang, joystick_t);
            
            //Shooter
            //Launcher
            if (joystick.getRawButton(TRIGGER)) launch.set(Relay.Value.kForward);
            else launch.set(Relay.Value.kOff);
                       
            //Winch
            if (joystick.getRawButton(4)) winch.set(-WINCH_SPEED);
            else if (joystick.getRawButton(6)) winch.set(WINCH_SPEED);
            else winch.set(0);
            
            //Dump Air
            if (joystick.getRawButton(2)) leak.set(true);
            else leak.set(false);
            
            //Reload
            if (joystick.getRawButton(3)) {
                in.set(true);
                out.set(false);
            } else if (joystick.getRawButton(5)) {
                in.set(false);
                out.set(true);
            }
            
            print();
            
        }
    }
    
    //Deadband method
    public double deadband(double d) {
        if(d > DEADBAND_LOW && d < DEADBAND_HIGH){
            d = 0;
        }
        return d;
    }
    
    //Set ratio coefficient
    public double ratioValue() {
       double f = joystick.getRawAxis(AXIS_THROTTLE);
       f = (-f + 1) / 2;
       return f;
    }
    
    //Hypotenuse
    public double hyp(double a, double b) {
        return Math.sqrt(MathUtils.pow(a, 2) + MathUtils.pow(b, 2));
    }
    
    public void additionDrive(Joystick joystick){
        
        double joystick_X = joystick.getX();
        double joystick_Y = joystick.getY();
        double joystick_t = joystick.getRawAxis(AXIS_THROTTLE);
        double joystick_v = joystick.getRawAxis(HAT_VERTICAL);
        double joystick_h = joystick.getRawAxis(HAT_HORIZONTAL);
        
        double angle = 0;
        double mag = 0;
        
        //Set Talons to Joystick Values
        drive1.set(ratioValue() * (deadband(joystick_Y) + deadband(joystick_X) + deadband(-joystick_t) + deadband(joystick_v)
                + deadband(joystick_h)));
        drive2.set(ratioValue() * (deadband(joystick_Y) + deadband(-joystick_X) + deadband(-joystick_t) + deadband(joystick_v)
                + deadband(-joystick_h)));
        drive3.set(ratioValue() * (deadband(-joystick_Y) + deadband(joystick_X) + deadband(-joystick_t) + deadband(-joystick_v)
                + deadband(joystick_h)));
        drive4.set(ratioValue() * (deadband(-joystick_Y) + deadband(-joystick_X) + deadband(joystick_t) + deadband(joystick_v) 
               + deadband(-joystick_h)));
        
        //Hat
        if (joystick_v == 0 && joystick_h == 0) {
            mag = ratioValue() * hyp(joystick_Y, joystick_X);
            angle = MathUtils.atan2(joystick_Y, joystick_X);
        } else if (joystick_v != 0) {
            mag = ratioValue() * joystick_v;
            angle = MathUtils.atan2(joystick_v, joystick_h);
        } else {
            mag = ratioValue() * joystick_h;
            angle = MathUtils.atan2(joystick_v, joystick_h);
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