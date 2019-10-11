package frc.team5115.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.team5115.subsystems.*;

public class Robot extends TimedRobot {
    public static Joystick joy;
    public static drivetrain dt;
    public static manueverinator manueverinatorinator;

    public void robotInit() {
        joy = new Joystick(0);
        dt = new drivetrain();
        manueverinatorinator = new manueverinator();
    }

    public void teleopPeriodic() {
        if(joy.getRawButton(8)) {
            manueverinatorinator.followCurve(); //follow to the thingy.
        }
        else {
            dt.drive(joy.getRawAxis(1), joy.getRawAxis(0), 0.35); //change thrott with 1-j.getRawAxis(3)
        }

        if(joy.getRawButton(9)) { //press this button to calibrate.
            manueverinatorinator.navxAngleReset(); //if the button is pressed reset the navx angle. Do this when relative to the wall.
        }
    }
}

/**
 * When the button is pressed,
 * Look for the target. If found,
 * Generate path to target, based on data currently found / averaged.
 * We have a path found. This path describes the angle we need to hold for this current angle. We use this curve, find the angle we want when in the position we are, then go to that angle. Forget the path.
 *

 */


