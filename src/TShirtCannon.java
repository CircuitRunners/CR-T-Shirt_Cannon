import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;

public class TShirtCannon extends SimpleRobot {
   
    //constants
    //Motor Controller Ports
    public static final int JAGUAR_DRIVE_PORT_FR = 1;
    public static final int JAGUAR_DRIVE_PORT_RR = 2;
    public static final int JAGUAR_DRIVE_PORT_FL = 3;
    public static final int TALON_DRIVE_PORT_RL = 4;
    
    public static final int TALON_WINCH_PORT = 5;
    
    //Special Joystick Values
    public static final int TRIGGER = 1;
    public static final int AXIS_THROTTLE = 4;
    public static final int HAT_HORIZONTAL = 5;
    public static final int HAT_VERTICAL = 6;
    
    //Constant Speeds
    public static final double WINCH_SPEED = 0.4;
    
    //Deadband
    public static final double DEADZONE_MOVE = 0.15;
    public static final double DEADZONE_SPIN = 0.2;
    
    //Number of Buttons
    public static final int BUTTONS = 16;
    
    //Pi
    public static final double PI = Math.PI;
    
    //Motor Contollers
    Jaguar drive1;
    Jaguar drive2;
    Jaguar drive3;
    Talon drive4;
    
    Talon winch;
    
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
    
    //Driver Station
    DriverStation ds;
    
    //Driver Station LCD
    DriverStationLCD dsLCD;
    
    //Watchdog
    Watchdog dog;
    
    TShirtCannon(){        
        //Construct Talons
        drive1 = new Jaguar(JAGUAR_DRIVE_PORT_FL);
        drive2 = new Jaguar(TALON_DRIVE_PORT_RL);
        drive3 = new Jaguar(JAGUAR_DRIVE_PORT_FR);
        drive4 = new Talon(JAGUAR_DRIVE_PORT_RR);
        
        winch = new Talon(TALON_WINCH_PORT);
        
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
        
        //Driver Station Instance
        ds = DriverStation.getInstance();
        
        //Driver Station LCD Instance
        dsLCD = DriverStationLCD.getInstance();
        
    }
   
    
    public void robotInit(){
        dsLCD.println(DriverStationLCD.Line.kUser1, 1,
                "Connected to T-Shirt Cannon Robot!");
        dsLCD.println(DriverStationLCD.Line.kUser3, 1,
                "Make sure digital input 1 is enabled!");
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
            /*
            if(ds.getDigitalIn(1)){
                additionDrive(joystick_X, joystick_Y, joystick_t, joystick_h,
                        joystick_v);
            }else{
                if(ds.getDigitalIn(2)){
                    driveRobot.mecanumDrive_Polar(trigDrive(joystick)[0],
                                                  trigDrive(joystick)[1],
                                                  trigDrive(joystick)[2]);
                }else{
                    driveRobot.mecanumDrive_Polar(ratioValue() * deadzoneMove(joystick_X),
                                                  ratioValue() * deadzoneMove(joystick_Y),
                                                  ratioValue() * deadzoneMove(joystick_t));
                }
            }
            */
            driveRobot.mecanumDrive_Cartesian(ratioValue() * deadzoneMove(joystick.getY()), ratioValue() *
                    -deadzoneMove(joystick.getX()), ratioValue() * -deadzoneSpin(joystick.getTwist()), 0);
            
            //Shooter
            //Launcher
            if(joystick.getRawButton(TRIGGER)){
                launch.set(Relay.Value.kForward);

                Timer.delay(0.5);

                in.set(false);
                out.set(true);

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
            dsLCD.println(DriverStationLCD.Line.kUser1, 1, "Joystick X: "
                    + joystick_X + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser2, 1, "Joystick Y: "
                    + joystick_Y + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser3, 1, "Joystick t: "
                    + joystick_t + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser4, 1, "Ratio Value: "
                    + ratioValue() + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser5, 1, "Joystick Mag: "
                    + joystick_mag + "                 ");
            dsLCD.println(DriverStationLCD.Line.kUser6, 1, "Joystick Angle: "
                    + joystick_ang + "                 ");
            dsLCD.updateLCD();
            
        }
    }
    
    //Deadband method for move
    public double deadzoneMove(double d){
        if(d > -DEADZONE_MOVE && d < DEADZONE_MOVE){
            d = 0;
        }
        return d;
    }
    
    public double deadzoneSpin(double d) {
        if (d > -DEADZONE_SPIN && d < DEADZONE_SPIN) {
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
    
    public double[] trigDrive(Joystick joystick){
        double mag = ratioValue() *hyp(deadzoneMove(joystick.getX()),
                deadzoneMove(joystick.getX()));
        double angle = MathUtils.atan2(deadzoneMove(joystick.getY()),
                deadzoneMove(joystick.getX()));
        angle = Math.toDegrees(angle);
        double twist = deadzoneMove(joystick.getTwist());
        
        double[] ans = new double[3];
        ans[0] = mag;
        ans[1] = angle;
        ans[2] = twist;
        
        return ans;
    }
    
    public void additionDrive(double joystick_X, double joystick_Y, double joystick_t,
            double joystick_h, double joystick_v){
        
        double angle = 0;
        double mag = 0;
        
        //Set Talons to Joystick Values
        drive1.set(ratioValue() * (deadzoneMove(joystick_Y) + deadzoneMove(joystick_X)
                + deadzoneMove(-joystick_t) + deadzoneMove(joystick_v)
                + deadzoneMove(joystick_h)));
        drive2.set(ratioValue() * (deadzoneMove(joystick_Y) + deadzoneMove(-joystick_X)
                + deadzoneMove(-joystick_t) + deadzoneMove(joystick_v)
                + deadzoneMove(-joystick_h)));
        drive3.set(ratioValue() * (deadzoneMove(-joystick_Y) + deadzoneMove(joystick_X)
                + deadzoneMove(-joystick_t) + deadzoneMove(-joystick_v)
                + deadzoneMove(joystick_h)));
        drive4.set(ratioValue() * (deadzoneMove(-joystick_Y) + deadzoneMove(-joystick_X)
                + deadzoneMove(joystick_t) + deadzoneMove(joystick_v) 
               + deadzoneMove(-joystick_h)));
        
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
    
    public void disabled(){
        dsLCD.clear();
        dsLCD.updateLCD();
    }
}
