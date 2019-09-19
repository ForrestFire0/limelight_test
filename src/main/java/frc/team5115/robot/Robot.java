package frc.team5115.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.team5115.subsystems.*;

public class Robot extends TimedRobot {
    //Olivia
    //Yeab
    //marie
    public static Arm arm;
    public static Joystick joy;
    public static drivetrain dt;
    public static intakeMech intakemech;
        //this is a yeet.... yeet.
    public void robotInit() {
        joy = new Joystick(0);
        arm = new Arm();
        dt = new drivetrain();
        intakemech.intakeStill();
        intakemech = new intakeMech();
    }

    public void teleopPeriodic() {
        arm.moveArm();
        dt.drive(joy.getRawAxis(0), joy.getRawAxis(1), 0.35); //change thrott with 1-j.getRawAxis(3)
        intakemech.intakeOpen();
        
    }
}


