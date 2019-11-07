package frc.team5115.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.team5115.subsystems.*;
//todome make the navx a robot variable, not a manuverinator file.
//todome make a calibration method to move it to a certain distance and know the angle. Basically a calibration method for the angle of the limelight.
public class Robot extends TimedRobot {
    public static Joystick joy;
    public static Drivetrain dt;
    public static manueverinator manueverinatorinator;
    public static NavX navX;

    public void robotInit() {
        joy = new Joystick(0);
        dt = new Drivetrain();
        navX = new NavX();
        manueverinatorinator = new manueverinator();
        navX.navxAngleReset(); //if the button is pressed reset the navx angle. Do this when relative to the wall.
        dt.resetTargetAngle(); //set the target angle to where we are looking.
    }

    public void teleopPeriodic() {
        navX.runTick();
        if(joy.getRawButton(8)) {
            System.out.println("------------------");
            manueverinatorinator.lineUp(); //follow to the thingy.
        }

        else {
            dt.knightlyDrive(joy.getRawAxis(0), joy.getRawAxis(1));
            //dt.RBW(joy.getRawAxis(0), joy.getRawAxis(1)); //Drive by wire based on the angle.
        }

        if(joy.getRawButton(9)) { //press this button to calibrate.
            navX.navxAngleReset(); //if the button is pressed reset the navx angle. Do this when relative to the wall.
            dt.resetTargetAngle();
        }

    }

    @Override
    public void disabledInit() {
        System.out.println("Disabled.");
        dt.drive(0,0,0);
    }

    @Override
    public void teleopInit() {
        System.out.println("Starting! Reset dt Target Angle");
        dt.resetTargetAngle();
    }

}

/**
 * When the button is pressed,
 * Look for the target. If found,
 * Generate path to target, based on data currently found / averaged.
 * We have a path found. This path describes the angle we need to hold for this current angle. We use this curve, find the angle we want when in the position we are, then go to that angle. Forget the path.
 *
 */


