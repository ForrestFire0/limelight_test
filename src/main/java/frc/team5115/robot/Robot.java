package frc.team5115.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.team5115.subsystems.*;

public class Robot extends TimedRobot {
    //Olivia
    //Yeab
    //marie
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
            //manueverinatorinator.aim();
            //manueverinatorinator.distancize();
            /*Please look at the bottom of manueverinator*/
        }
        //System.out.println("Println is working");
        else {
            dt.drive(joy.getRawAxis(1), joy.getRawAxis(0), 0.35); //change thrott with 1-j.getRawAxis(3)
        }
    }
}


