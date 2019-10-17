package frc.team5115.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.team5115.subsystems.*;

public class Robot extends TimedRobot {
    public static Joystick joy;
    public static Drivetrain dt;
    public static manueverinator manueverinatorinator;

    public void robotInit() {
        joy = new Joystick(0);
        dt = new Drivetrain();
        manueverinatorinator = new manueverinator();
    }

    public void teleopPeriodic() {
        if(joy.getRawButton(8)) {
            System.out.println("------------------");
            manueverinatorinator.lineUp(); //follow to the thingy.
        }
        else {
            dt.RBW(joy.getRawAxis(0)); //Drive by wire based on the angle.
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


