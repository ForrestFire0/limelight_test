package frc.team5115.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team5115.robot.Robot;
import java.util.Arrays;

// Docs: http://docs.limelightvision.io/en/latest/networktables_api.html

public class manueverinator {

    NetworkTable limelight;
    static NetworkTableEntry tx; // Measure of X offset angle
    static NetworkTableEntry ty; // Measure of Y offset angle
    static NetworkTableEntry ta; // Measure of image area
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


    /* *
    *   update camera height
    *   update target height
    *   update camera angle
     */

    private double Kp = 0.2; //a modifier to the aim function.
    private double deadZoneDegrees = 5;

    // Load in the network tables
    public manueverinator(){
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelight.getEntry("tx"); //Angle in x of degrees
        ty = limelight.getEntry("ty"); //Angle in y of degrees
        ta = limelight.getEntry("ta"); //Get area
        tv = limelight.getEntry("tv"); //have target?
        camtran = limelight.getEntry("camtran"); //Raw 3d positioning
        LED = limelight.getEntry("ledMode");
        CAM = limelight.getEntry("camMode");
        pipeline = limelight.getEntry("pipeline");
    }

    // Return X-Offset angle from the limelight

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

    public void aim() {

        if(tv.getDouble(0) == 1) {
            double heading_error = -tx.getDouble(0); //how far off is it from the center of the screen
            double steering_adjust = 0; //a var that is the processes value that is added and taken from each wheel.
            if (Math.abs(heading_error) > deadZoneDegrees) //if it is an error greater than one (significant)
            {
                steering_adjust = Kp * heading_error;
            } else {
                steering_adjust = 0;
            }

            //Help discovering errors:
            System.out.println("Heading Error: " + heading_error);
            System.out.println("Steering Adjustment: " + steering_adjust);

            Robot.dt.drive(0,-steering_adjust,0.30); //drive baby
        }
        else {
            System.out.println("No target found. Stopping.");
            //In the furure, maybe we should add a scanning function.
            Robot.dt.drive(0,0,0);
        }
    }

    public void distancize(double targetDistance) { //updates the distance of the hypotenuse of the distance.
        angle_sum = ty.getDouble(0) + camera_angle; //finds the overall angle from the ground to the center of the reflector.
        hypotenuse = (target_height - camera_height)/Math.tan(Math.toRadians(angle_sum)) + 0; // Add or subtract depending on inaccuracies.
        System.out.println("distance to target: " + y_Offset);
    }

    public void debug() {
        System.out.println("tx: " + tx.getDouble(0));
        System.out.println("ty: " + ty.getDouble(0));
        System.out.println("tv: " + tv.getDouble(0));
        //System.out.println(camtran);

    }

    public void print3dStuff() {
        System.out.println("Rejoice! 3D numbers are apon us!");
        double[] _3dStuff = camtran.getDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
        System.out.println("X value" + _3dStuff[0]);
        System.out.println("1st Y value: " + _3dStuff[1]);
        System.out.println("2nd Y value: "_3dStuff[2]);

    }

    public void getRawCorners() { //this boy is a yeet.
        double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};

        double[] xCorners = limelight.getEntry("tcornx").getDoubleArray(mZeroArray); //get the raw corners. Use an empty array as the default value.
        double[] yCorners = limelight.getEntry("tcorny").getDoubleArray(mZeroArray);

        System.out.println("x raw corners" + Arrays.toString(xCorners));
        System.out.println("Y raw corners" + Arrays.toString(yCorners));
    }

}

/**
 * Things to fool around with
 * 1) First run debug() and
 * 2) then get aim running again.
 * 3) getRawCorners() - just see what this prints because it might be cool.
 * 4) distancize(10) - with this we can start to develop the bunny bots shooter code.
 * 5)
 *
 */

/*
    |\
  y | \ hypotenuse
    |  \
    |   \
    ______
    xoffset

y = y offset.

 */




