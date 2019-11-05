package frc.team5115.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import frc.team5115.robot.Robot;

import javax.naming.spi.ObjectFactory;

// Docs: http://docs.limelightvision.io/en/latest/networktables_api.html

/**
 * The base class for all autonomous using the limelight.
 */
public class manueverinator {

    private static NetworkTableEntry tx; // Measure of X offset angle
    private static NetworkTableEntry ty; // Measure of Y offset angle
    private NetworkTableEntry tv;

    // Variables needed in calculation(s) -> All are static because thats what limelight wants ¯\_(ツ)_/¯

    private double xOffset; // The horizontal shift the robot needs to make in order to align. FROM THE CENTER OF THE ROBOT.
    private double yOffset; // The vertical shift the robot needs to make in order to align. FROM THE CENTER OF THE ROBOT

    private float getYaw;
    private double[] emptyDoubleArray = new double[6];

    private final double Kp = 0.2; //a modifier to the aim function.
    private final double deadZoneDegrees = 5;

    /**
     * Creates the limelight table entries.
     */
    // Load in the network tables
    public manueverinator() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelight.getEntry("tx"); //Angle in x of degrees
        ty = limelight.getEntry("ty"); //Angle in y of degrees
        tv = limelight.getEntry("tv"); //have target?


    }


    /**
     * aims directly at the target.
     */
    /*
     * DEAR FUTURE FORREST: There are two possible applications for this program.
     * <p>
     * If you want to keep on working on tracking the target WHEN THE TARGET MOVES:
     * When the limelight looses the target, it should keep the rate it had previously. This way, if the target moves away to fast, the robot will continue to move.
     * <p>
     * If you want to work on finding a stable target:
     * Update the program to search for a target.
     * The question now becomes which way to start rotating to find the target.
     * USE THE PREVIOUS CONTROL FROM THE JOYSTICK. This way the robot can keep going the way it was steered, and look that way first.
     * You can tell the drivers to look a little to the left of the target and then go right, therefore ensuring that the robot is at the correct angle of rotation.
     */
    public void aim() {
        if (tv.getDouble(0) == 1) {
            Robot.dt.angleHold(ty.getDouble(0), 0);
        } else {
            System.out.println("No target found. Stopping.");
            //In the future, maybe we should add a scanning function.
            Robot.dt.drive(0, 0, 0);
        }
    }

    /**
     * prints tx, ty, tv.
     */
    public void debug() {
        System.out.println("tx: " + tx.getDouble(0));
        System.out.println("ty: " + ty.getDouble(0));
        System.out.println("tv: " + tv.getDouble(0));
        //System.out.println(camtran);
    }

    private void update3dPoints() {

        //calculate values from navx and other things.
        System.out.println("update3DPoints: Using Math");
        final double targetHeight = 36; //is it 36 inches???
        final double cameraHeight = 8; //update but it probably doesnt matter.
        final double cameraAngle = 23; //update
        double hypotenuse = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(ty.getDouble(0) + cameraAngle)); //
        //System.out.println(ty.getDouble(0) + cameraAngle + " = angle");
        //System.out.print("/" + Math.tan(Math.toRadians(ty.getDouble(0) + cameraAngle)));
        double yaw = getYaw + tx.getDouble(0); //angle from the wall. Remember: negative is pointing to left, positive is to the right.
        xOffset = -sin(yaw) * hypotenuse;
        yOffset = -cos(yaw) * hypotenuse;
        //The following turns adjusts the x and y values from the limelight to get the

        final double relativeLLx = 0; //Positive value means the limelight is to the right of the center, while negative is to the left.
        final double relativeLLy = -13; //negative 20 means that the robot location is 20 inches. behind the limelight.
        //System.out.print("X=" + xOffset + " - " +
        //       relativeLLx*cos(getYaw) + " - " +
        //       relativeLLy*sin(getYaw));

        xOffset = xOffset - (relativeLLx * cos(getYaw)) - (relativeLLy * sin(getYaw));
        //System.out.println(" = " + xOffset);


        //System.out.print("Y=" + yOffset + " - " +
        //        relativeLLy*cos(getYaw) + " - " +
        //       relativeLLx*sin(getYaw));

        yOffset = yOffset - (relativeLLy * cos(getYaw)) - (relativeLLx * sin(getYaw));
        //System.out.println(" = " + yOffset);

    }

    private double sin(double n) {
        return Math.sin(Math.toRadians(n));
    }

    private double cos(double n) {
        return Math.cos(Math.toRadians(n));
    }


    private double findAngle() {
        if (yOffset > -30) { //We are close to the wall, so no matter making it anything but the goal point.
            return 0;
        }
        //else
        double targetY = locateTargetPoint();
        System.out.println("findAngle: TargetY: " + targetY);
        double angle = getAngleFromTargetPoint(targetY);

        angle = safeAngle(angle);

        return getYaw + ((angle - getYaw) / 2);


    }

    /**
     * @param angleRequested the angle you want the robot to hold.
     * @return safeAngle is the maximum angle the robot can hold.
     */
    private double safeAngle(double angleRequested) {
        //given the angle requested, the change we need to make, and how much more we can turn without it going off the screen, return the maximun angle we can turn, given that it is NOT safe.
        //THIS IS LIKE 99% working. I tested a crap ton on codeHS.
        //angleRequested: The new angle we want to hold, RTF (relative to field).
        //getYaw: The current angle held RTF.
        //currentOffset: where the target is in the cameras vision. NOT RTF.
        double currentOffset = tx.getDouble(0);
        double degreesLeft = 30 - Math.abs(currentOffset);//the amount of degrees we can move before we go off the field, ABS.
        //System.out.println("There are " + degreesLeft + " degrees left before the camera looses sight.");

        if (angleRequested - getYaw > 30 + currentOffset) { //to the right
            System.out.println("safeAnlge: Limited. Trying to move TO FAR TO THE RIGHT");
            //limit it to only come way to the right. add degrees left to the current angle.
            return getYaw + degreesLeft;
        } else if (angleRequested - getYaw < -30 + currentOffset) { //to the LEFT
            System.out.println("safeAnlge: Limited. Trying to move TO FAR TO THE LEFT");
            //limit it to only come way to the left. subtract degrees left to the current angle.
            //because it should be to the left, which is negative.
            return getYaw - degreesLeft;
        }

        //everything checked out. Send value back.
        return angleRequested;
    }

    /**
     * @param targetY how far out from the target you want to point at.
     * @return the angle we want to hold relative to the target. 0 is strait ahead.
     */
    private double getAngleFromTargetPoint(double targetY) {
        //takes in two points, x and y, that are relative to the limelight target / wall. returns the angle that the robot needs to hold, relative to the wall.
        //an angle of 0 is strait at the target, while 90 is all the way
        double currentX = xOffset;
        double currentY = yOffset;
        double targetX = 0; //on the line out

        double deltaX = targetX - currentX; //get the difference in x values;
        //System.out.print("getAngleFrom...: TgtX: " + (int) targetX + " - CurrntX: " + (int) currentX + " = ");
        //System.out.println((int) deltaX + " = deltaX");

        double deltaY = Math.abs(targetY - currentY); //get the difference in y values;
        //System.out.print("getAngleFrom...: TgtY " + (int) targetY + " - CurrntY" + (int) currentY + " = ");
        //System.out.println((int) deltaY + " = deltaY");

        double radians = Math.atan2(deltaX, deltaY); //uses tangent to get angle.
        return Math.toDegrees(radians); //returns angle in radians.
    }

    /**
     * @return the target point in 3d space. Minimum -30 inches.
     */
    private double locateTargetPoint() {
        //this finds the y value that we need to look at.
        //return -40; //returning 2 feet out from wall to the limelight at the moment. Once we get better at following things then we can
        return Math.min(0.67 * yOffset, -30); //takes the smaller of the two values. Once the calculated target point is further forward than -30, the program designateds -30 as the target location.
        //Also note that this is the center of the robot, not the front of the robot. Add the relativeLLy to get the distance to the front of the robot.
    }


    /**
     * Lines up the robot to the target found by the limelight.
     */
    public void lineUp() {
        if (tv.getDouble(0) == 0) { //no target found.
            System.out.println("main: ERROR : NO TARGET FOUND");
            Robot.dt.drive(0, 0, 0);
            return;
        }

        getYaw = relativize((int) Robot.navX.getYaw());  //get the yaw from the navx. MUST BE UPDATED TO BE RELATIVE TO THIS FRAME.

        update3dPoints();//acquire new points, aka xOffset and yOffset. Adjust them to be robot center oriented.

        //System.out.println("main: X: " + (int) xOffset + " Y: " + (int) yOffset + " Angle: " + (int) getYaw);

        double targetAngle = findAngle(); //get the angle we need. RELATIVE TO WALL

        //System.out.println("main: targetAngle: " + (int) targetAngle);

        double followingTrackSpeed = calcFollowingCalcSpeed(); //NOT how far out we start linearly slowing... higher num = slowing more.
        System.out.println(followingTrackSpeed);
        Robot.dt.angleHold(getYaw, targetAngle, followingTrackSpeed);//followingTrackSpeed);
    }

    /**
     * @return speed in which one should travel to the target. based on yOffset.
     * preconditions: yOffset
     */
    private double calcFollowingCalcSpeed() {
        int distance = 200; //The distance where the slowing begins.
        double max = -0.3;   //The distance maximum motor values.
        //yOffset is negative. divided by distance. Will become a
        return Math.max((yOffset / distance) - 0.3, -0.5); //subtract in order to ensure we always keep moving.
    }

    private float relativize(int yaw) { //this assumes two targets, one looking at 0 and 180.

        int modYaw = yaw + 89;
        int leftOver = (modYaw % 180) - 89;
        if (leftOver < -90) {
            leftOver += 180;
        }

        return leftOver;
    }
}


/** Things changed
 * relativize added. Check functionality of relativize.
 * Calc following speed added. Edit min and maxes to do things.
*/