import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Watchdog;
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;

public class TShirtCannon extends SimpleRobot {
   
    //constants
    //Motor Controller Ports
    public static final int TALON_PORT_FL = 1;
    public static final int TALON_PORT_FR = 3;
    public static final int TALON_PORT_RL = 2;
    public static final int TALON_PORT_RR = 4;
    
    public static final int WINCH_PORT = 5;
    
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
    RobotDrive driveRobot;
    
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
    
    //Driver Station LCD
    DriverStationLCD dsLCD;
    
    //Watchdog
    Watchdog dog;
    
    TShirtCannon(){        
        //Construct Talons
        drive1 = new Jaguar(TALON_PORT_FL);
        drive2 = new Jaguar(TALON_PORT_RL);
        drive3 = new Jaguar(TALON_PORT_FR);
        drive4 = new Jaguar(TALON_PORT_RR);
        
        winch = new Jaguar(WINCH_PORT);
        
        driveRobot = new RobotDrive(drive1, drive2 ,drive3, drive4);
        driveRobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        driveRobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        
        //Construct Relay
        launch = new Relay(8);
        
        //Construct Aolenoid
        leak = new Solenoid(1);
        in = new Solenoid(2);
        out = new Solenoid(3);
        
        //Construct Joystick
        joystick = new Joystick(1);
        
        //Construct Buttons
        buttons = new boolean[BUTTONS];
        
        //Set buttons to false
        for (int i = 1; i < BUTTONS; i++){
            buttons[i] = false;
        }
        
        //The Watchdog is born!       
        dog = Watchdog.getInstance();
        dog.setExpiration(0.5);
        
        //Watchdog is replaced!
        driveRobot.setSafetyEnabled(true);
        
        //Driverststion Instance
        
        //Construct SmartDashboard
        dsLCD = DriverStationLCD.getInstance();
        
    }
    
    //Called once each autonomous mode
    public void autonomous() {
        
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
            double joystick_h = joystick.getRawAxis(HAT_HORIZONTAL);
            double joystick_v = joystick.getRawAxis(HAT_VERTICAL);
            double joystick_ang = joystick.getDirectionDegrees();
            double joystick_mag = joystick.getMagnitude();
            
            //drive(joystick_X, joystick_Y, joystick_t, joystick_h, joystick_v);
            //Make a switch in dashbord for old code on/off?
            driveRobot.mecanumDrive_Polar(joystick_X, joystick_Y, joystick_t);
            
            //Shooter
            //Launcher
            if(joystick.getRawButton(TRIGGER)){
                launch.set(Relay.Value.kForward);
            }else{
                launch.set(Relay.Value.kOff);
            }
                       
            //Winch
            if(joystick.getRawButton(4)){
                winch.set(-WINCH_SPEED);
            }else if(joystick.getRawButton(6)){
                winch.set(WINCH_SPEED);
            }
            else{
                winch.set(0);
            }
            
            //Dump Air
            if(joystick.getRawButton(2)){
                leak.set(true);
            }else{
                leak.set(false);
            }
            
            //Reload
            if(joystick.getRawButton(3)){
                in.set(true);
                out.set(false);
            }else if(joystick.getRawButton(5)){
                in.set(false);
                out.set(true);
            }
            
            // Driver Station LCD Output
            dsLCD.println(DriverStationLCD.Line.kUser1, 1, "Joystick X: " + joystick_X + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser2, 1, "Joystick Y: " + joystick_Y + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser3, 1, "Joystick t: " + joystick_t + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser4, 1, "Ratio Value: " + ratioValue() + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser5, 1, "Joystick Mag: " + joystick_mag + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser6, 1, "Joystick Angle: " + joystick_ang + "                 ");
            dsLCD.updateLCD();
            
        }
    }
    
    //Deadband method
    public double deadband(double d){
        if(d > DEADBAND_LOW && d < DEADBAND_HIGH){
            d = 0;
        }
        return d;
    }
    
    //Set ratio coefficient
    public double ratioValue(){
       double f = joystick.getRawAxis(AXIS_THROTTLE);
       f = (-f + 1) / 2;
       return f;
    }
    
    //Hypotenuse
    public double hyp(double a, double b){
        return Math.sqrt(MathUtils.pow(a, 2) + MathUtils.pow(b, 2));
    }
   
    //Average
    public double avg(double a, double b){
        return (a + b) / 2;
    }
    
    public void drive(double joystick_X, double joystick_Y, double joystick_t,
            double joystick_h, double joystick_v){
        
        double angle = 0;
        double mag = 0;
        
        //Set Talons to Joystick Values
        drive1.set(ratioValue() * (deadband(joystick_Y) + deadband(joystick_X)
                + deadband(-joystick_t) + deadband(joystick_v) + deadband(joystick_h)));
        drive2.set(ratioValue() * (deadband(-joystick_Y) + deadband(joystick_X)
                + deadband(-joystick_t) + deadband(-joystick_v) + deadband(joystick_h)));
        drive2.set(ratioValue() * (deadband(joystick_Y) + deadband(-joystick_X)
                + deadband(-joystick_t) + deadband(joystick_v) + deadband(-joystick_h)));
        drive4.set(ratioValue() * (deadband(-joystick_Y) + deadband(-joystick_X)
                + deadband(-joystick_t) + deadband(-joystick_v) + deadband(-joystick_h)));

        //Hat
        if(joystick_v != 0){
            mag = ratioValue() * joystick_v;
            angle = MathUtils.atan2(joystick_v, joystick_h);
        }
        if(joystick_h != 0){
            mag = ratioValue() * joystick_h;
            angle = MathUtils.atan2(joystick_v, joystick_h);
        }else if(joystick_v == 0 && joystick_h == 0){
            mag = ratioValue() * hyp(joystick_Y, joystick_X);
            angle = MathUtils.atan2(joystick_Y, joystick_X);
        }
    }
}