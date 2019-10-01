package frc.team5115.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team5115.robot.Robot;

// Docs: http://docs.limelightvision.io/en/latest/networktables_api.html

public class manueverinator {

    NetworkTable limelight;
    static NetworkTableEntry tx; // Measure of X offset angle
    static NetworkTableEntry ty; // Measure of Y offset angle
    static NetworkTableEntry ta; // Measure of image area
    NetworkTableEntry tx0;
    NetworkTableEntry ty0;
    NetworkTableEntry tv;
    NetworkTableEntry camtran;
    NetworkTableEntry LED;
    NetworkTableEntry CAM;
    NetworkTableEntry pipeline;

    // Variables needed in calculation(s) -> All are static because thats what limelight wants ¯\_(ツ)_/¯
    static double camera_height = 0; // The height of camera (fixed)
    static double target_height = 10; // Height of level 1 targets (fixed) MUST BE UPDATED
    static double camera_angle = 0; // Angle of inclination of limelight with respect to ground.
    static double angle_sum;
    public static double x_Angle; // X-Offset angle
    public static double y_Angle; // Y-offset angle with respect to ground
    public static double x_Offset; // The horizontal shift the robot needs to make in order to align
    public static double y_Offset; // The vertical shift the robot needs to make in order to align
    public static double hypotenuse; // Pythagorean of x-off and y-off
    public static double calc_Angle; // The angle we calculated

    double Kp = 0.1; //a modifier to the aim function.
    double min_command = 0.05; //if the output to the motor is too small, then just use this value.

    // Load in the network tables
    public manueverinator(){
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelight.getEntry("tx"); //Angle in x of degrees
        ty = limelight.getEntry("ty"); //Angle in y of degrees
        ta = limelight.getEntry("ta"); //Get area
        tv = limelight.getEntry("tv"); //have target?
        tx0 = limelight.getEntry("tx0"); //raw something or another.
        ty0 = limelight.getEntry("ty0"); //raw something or another.
        camtran = limelight.getEntry("camtran"); //raw something or another.
        LED = limelight.getEntry("ledMode");
        CAM = limelight.getEntry("camMode");
        pipeline = limelight.getEntry("pipeline");
    }

    // Check to see if targets are visible
    public boolean target() {
        return tv.equals(1);
    }

    // Return X-Offset angle from the limelight
    public double xm_Angle() {
        return tx.getDouble(0);
    }

    // Return Y-Offset angle from limelight
    public double ym_Angle() {
        return ty.getDouble(0);
    }

    // Set the camera into camera mode
    public void s_Camera() {
        LED.setNumber(1); //Leds off
        CAM.setNumber(1); //Camera to Camera mode, processing off.
    }

    // Set the camera into scanner mode
    public void s_Scanner(){
        LED.setNumber(3); //Leds on
        CAM.setNumber(0); //Camera on vision processing mode.
    }

    public static void gcamera_Angle() {
        calc_Angle = Math.atan((target_height - camera_height)/hypotenuse) + y_Angle;
        System.out.println(calc_Angle);
    }

    public static void calc_Distance() {
        angle_sum = ty.getDouble(0) + camera_angle; //finds the overall angle from the ground to the center of the reflector.
        y_Offset = (target_height - camera_height)/Math.tan(Math.toRadians(angle_sum)) + 0; // Add or subtract depending on inaccuracies.
        //x_Offset = y_Offset * Math.sin(Math.toRadians(tx.getDouble(0))); //this is faulty... it does calculate a distance... but not the one we want.
        // Display results
        System.out.println("TX: " + tx.getDouble(0)); // Print the x-angle
        System.out.println("TY: " + ty.getDouble(0)); // Print the y-angle
        System.out.println("Vertical distance to target: " + y_Offset);
        //System.out.println("Horizontal offset (not what you think it is): " + x_Offset); //this is wrong sadly...
        System.out.println("=============="); // Spacer
    }

    public void aim() {

        if(tv.getDouble(0) == 1) {
            double heading_error = -tx.getDouble(0); //how far off is it from the center of the screen
            double steering_adjust = 0; //a var that is the processes value that is added and taken from each wheel.
            double left_command = 0;
            double right_command = 0;
            if (heading_error > 1.0) //if it is an error greater than one (significant)
            {
                steering_adjust = Kp * heading_error - min_command;
            } else if (heading_error < 1.0) {
                steering_adjust = Kp * heading_error + min_command;
            }
            left_command += steering_adjust;
            right_command -= steering_adjust;

            Robot.dt.drive(0,0,0);
            //Help discovering errors:
            System.out.println("Heading Error: " + heading_error);
            System.out.println("Steering Adjustment: " + steering_adjust);
            //System.out.println("Left wheel set at: " + left_command);
            //System.out.println("Right wheel set at: " + right_command);
            //System.out.println("-----------");
            Robot.dt.drive(0,-steering_adjust,0.30);
        }
        else {
            System.out.println("No target found. Doing nothing.");
            //set motors to 0.
            Robot.dt.drive(0,0,0);

        }
    }

    public void distancize(double targetDistance) { //updates the distance of the hypotenuse of the distance.
        angle_sum = ty.getDouble(0) + camera_angle; //finds the overall angle from the ground to the center of the reflector.
        hypotenuse = (target_height - camera_height)/Math.tan(Math.toRadians(angle_sum)) + 0; // Add or subtract depending on inaccuracies.
        System.out.println("distance to target: " + y_Offset);
    }

    public void printCoolStuff() { //hopefully prints something useful.

        System.out.println("tx0: " + tx0.getDouble(0));
        System.out.println("ty0: " + ty0.getDouble(0));
        System.out.println("camtran: " + camtran.getString(""));

    }

}

/*
    |\
  X | \ hypotenuse
    |  \
    |   \
    ______
    xoffset

X = y offset.

does this edit go through???

 */




