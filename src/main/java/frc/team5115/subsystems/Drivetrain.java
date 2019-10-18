package frc.team5115.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team5115.robot.Robot;

public class Drivetrain {
    //instances of the speed controllers
    private TalonSRX frontLeft;
    private TalonSRX frontRight;
    private TalonSRX backLeft;
    private TalonSRX backRight;
    private double lastAngle;
    private double targetAngle;
    /*
    private Encoder leftEncoder;
    private Encoder rightEncoder;
     */
    public Drivetrain() {  //instantiation of the objects
        frontLeft = new TalonSRX(1);
        frontRight = new TalonSRX(2);
        backLeft = new TalonSRX(3);
        backRight = new TalonSRX(4);
        // Magic
        frontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);



        /*leftEncoder = new Encoder(RobotMap.leftEncoder1, RobotMap.leftEncoder2, true, CounterBase.EncodingType.k4X);
        rightEncoder = new Encoder(RobotMap.rightEncoder1, RobotMap.rightEncoder2, true, CounterBase.EncodingType.k4X);
        leftEncoder.setDistancePerPulse(12.566 / 250);	// 250 CPR, 4 pulses per cycle, 12.566 inches circumference on wheel
        rightEncoder.setDistancePerPulse(12.566 / 250);
         */
    }

    public void drive(double y, double x, double throttle) { //Change the drive output
        //called lots of times per seconds.
        y *= -1;

        double leftSpd = (x + y) * throttle;
        double rightSpd = (x - y) * throttle;
        //set the outputs. let the magic occur
        frontLeft.set(ControlMode.PercentOutput, leftSpd);
        backLeft.set(ControlMode.PercentOutput, leftSpd);
        frontRight.set(ControlMode.PercentOutput, rightSpd);
        backRight.set(ControlMode.PercentOutput, rightSpd);
    }


    public void resetEncoders() { //no idea when this is called if ever
        //necessary for magic to occur.
        frontLeft.set(ControlMode.PercentOutput, 0);
        backLeft.set(ControlMode.PercentOutput, 0);
        frontRight.set(ControlMode.PercentOutput, 0);
        backRight.set(ControlMode.PercentOutput, 0);
    }

    void angleHold(double currentAngle, double targetAngle, double y) {

        double kP = 0.02;
        //double kD = 0.01;

        double P = kP*(targetAngle - currentAngle);
        //double D = kD*((currentAngle - lastAngle)/0.02); //finds the difference in the last tick.
        this.drive(y,P,1);
    }

    void angleHold(double currentAngle, double targetAngle) { //Overridden magic.
        this.angleHold(currentAngle, targetAngle, 0);
    }

    public void RBW(double x, double y) { //rotate by wire
        double currentAngle = Robot.manueverinatorinator.getGetYaw();
        targetAngle += x*1.5; //at 50 ticks a second, this is 50 degrees a second because the max x is 1.
        angleHold(currentAngle, targetAngle, y);
    }

    public void knightlyDrive(double x, double y) { //
        double currentAngle = Robot.manueverinatorinator.getGetYaw();
        targetAngle = targetAngle + x*1.5*y; //at 50 ticks a second, this is 50 degrees a second because the max x is 1.
        //The faster we are moving forward (switch this data with encoder data) The faster we should rotate.
        angleHold(currentAngle, targetAngle, y);
    }
}