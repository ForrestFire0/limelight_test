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
    }

    public void drive(double x, double y, double throttle) { //Change the drive output
        //called lots of times per seconds.
        x *= -1;

        double leftSpd = (y + x) * throttle;
        double rightSpd = (y - x) * throttle;
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

        double kP = 0.05;
        double kD = 0.05;

        double P = kP*(targetAngle - currentAngle);
        double D = kD*((currentAngle - lastAngle)/0.02); //finds the difference in the last tick.
        System.out.println("PID Stuff: P:" + P + " D: " + D);
        this.drive(P+D,y/3.5,1);
    }

    void angleHold(double currentAngle, double targetAngle) { //Overridden magic.
        this.angleHold(currentAngle, targetAngle, 0);
    }

    public void RBW(double x) { //rotate by wire
        double currentAngle = Robot.manueverinatorinator.getGetYaw();
        targetAngle += x; //at 50 ticks a second, this is 50 degrees a second because the max x is 1.
        System.out.println("Target Angle: " + (int) targetAngle + " Current Angle" + (int) currentAngle);
        angleHold(currentAngle, targetAngle);
    }
}