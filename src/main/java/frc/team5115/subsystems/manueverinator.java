package frc.team5115.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team5115.robot.Robot;

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
    double camera_height = 0; // The height of camera (fixed)
    double target_height = 10; // Height of level 1 targets (fixed) MUST BE UPDATED
    double camera_angle = 0; // Angle of inclination of limelight with respect to ground.
    double angle_sum;
    public double xOffset; // The horizontal shift the robot needs to make in order to align
    public double yOffset; // The vertical shift the robot needs to make in order to align
    public double hypotenuse; // Pythagorean of x-off and y-off
    private AHRS navx; //turn baby.
    private float getYaw;

    /* *
    *   update camera height
    *   update target height
    *   update camera angle
     */

    private double Kp = 0.2; //a modifier to the aim function.
    private double deadZoneDegrees = 5;
    private double turnKd = 0.02;
    private double followingTrackSpeed = 0.3;

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
        navx = new AHRS(SerialPort.Port.kMXP);


    }

    public void navxAngleReset() {
        navx.reset(); //reset to the field orientation
        System.out.println("Angle has been reset.");
        System.out.println(navx.getYaw());
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
        System.out.println("distance to target: " + yOffset);
    }

    public void debug() {
        System.out.println("tx: " + tx.getDouble(0));
        System.out.println("ty: " + ty.getDouble(0));
        System.out.println("tv: " + tv.getDouble(0));
        //System.out.println(camtran);
    }

    private void update3dPoints() {
        //System.out.println("Rejoice! 3D numbers are apon us!");
        double[] _3dStuff = camtran.getDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
        //System.out.println("X value" + _3dStuff[0]);
        //System.out.println("1st Y value: " + _3dStuff[1]);
        //System.out.println("2nd Y value: " + _3dStuff[2]);
        xOffset = _3dStuff[0];
        yOffset = _3dStuff[2];
    }

    /**
     *
     * @param x The x offset from the thingy
     * @param y the y offfset from the thingy
     * @return the angle the robot needs to hold.
     */



    private double findAngle(double x, double y) {
        double angle = 0;
        double targetY = locateTargetPoint();
        angle = getAngleFromTargetPoint(targetY);
        return safeAngle(angle);
    }

    private double safeAngle(double angleRequested) { //given the angle requested, the change we need to make, and how much more we can turn without it going off the screen, return the maximun angle we can turn, given that it is NOT safe.
        //THIS IS LIKE 99% working. I tested a crap ton on codeHS.
        //angleRequested: The new angle we want to hold, RTF (relative to field).
        //getYaw: The current angle held RTF.
        //currentOffset: where the target is in the cameras vision. NOT RTF.
        double currentOffset = tx.getDouble(0);
        getYaw = 10;
        double degreesLeft = 30 - Math.abs(currentOffset);//the amount of degrees we can move before we go off the field, ABS.
        //System.out.println("There are " + degreesLeft + " degrees left before the camera looses sight.");

        if (angleRequested-getYaw > 30 + currentOffset) { //to the right
            System.out.println("Limited. Trying to move TO FAR TO THE RIGHT");
            //limit it to only come way to the right. add degrees left to the current angle.
            return getYaw + degreesLeft;
        }
        else if (angleRequested-getYaw < -30 + currentOffset) { //to the LEFT
            System.out.println("Limited. Trying to move TO FAR TO THE LEFT");
            //limit it to only come way to the left. subtract degrees left to the current angle.
            //because it should be to the left, which is negative.
            return getYaw - degreesLeft;
        }

        //everything checked out. Send value back.
        return angleRequested;

    }


    private double getAngleFromTargetPoint(double _targetY) { //takes in two points, x and y, that are relative to the limelight target / wall. returns the angle that the robot needs to hold, relative to the wall.
        //an angle of 0 is strait at the target, while 90 is all the way
        double currentX = xOffset;
        double currentY = yOffset;
        double targetX = 0; //on the line out

        double deltaX = targetX - currentX; //get the difference in x values;
        double deltaY = Math.abs(_targetY - currentY); //get the difference in y values;

        System.out.println("Slope is equal too: " + (deltaY/deltaX));
        System.out.println("TargetY = " + _targetY);

        double radians = Math.atan2(deltaX,deltaY); //uses tangent to get angle.
        return Math.toDegrees(radians); //returns angle in radians.
    }

    private double locateTargetPoint() { //this finds the y value that we need to look at.
        return 24; //returning 2 feet out at the moment. Once we get better at following things then we can
        //return 2*yOffset/3 which will give us a nice curve.
    }

    private double getTurnValue(double currentAngle, double wantedAngle) { //positive turn right.
        double turnOffset;
        turnOffset = turnKd * (wantedAngle - currentAngle);
        return turnOffset; // the 0-1 value that we get based on the angle offset.
    }

    public void followCurve() {
        if(tv.getDouble(0) == 0) { //no target found.
            System.out.println("!!! ERROR !!! NO TARGET FOUND");
            return;
        }
        getYaw = relativize(navx.getYaw());  //get the yaw from the navx. MUST BE UPDATED TO BE RELATIVE TO THIS FRAME.

        update3dPoints();//acquire points, aka xoffset and y offset.
        System.out.println("X of 3D: " + xOffset + " Y of 3D: " + yOffset);
        double angleOffset = findAngle(xOffset,yOffset); //get the angle we need.
        System.out.println("Current Angle: " + getYaw + "Target Angle: " + angleOffset); //this returns the angle relative to the wall. 0 is strait at the wall, while 90 is completely right and -90 is completely left.
        double turnOffset = getTurnValue(getYaw, angleOffset);
        System.out.println("Wheel offsets: " + turnOffset);
        //Robot.dt.drive(turnOffset, followingTrackSpeed,0.30); //drive baby

    }

    private float relativize(float yaw) {
        //this function needs to change the frame of the current position to the way we are looking at the wall. To start, we will line it up with the wall to get a good reference.
        // in a real game we will need it to dynamically detect where it is lining up to.
        return yaw;
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




