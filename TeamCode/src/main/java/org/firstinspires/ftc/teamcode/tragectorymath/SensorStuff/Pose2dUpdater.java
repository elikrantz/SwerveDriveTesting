package org.firstinspires.ftc.teamcode.tragectorymath.SensorStuff;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.tragectorymath.RobotConstants;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.tragectorymath.SensorStuff.localization.*;
import org.firstinspires.ftc.teamcode.tragectorymath.maths.*;

public class Pose2dUpdater {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private IMU imu;
    private Localizer localizer;

    private Pose2d startingPose2d = new Pose2d(0,0,0);
    private Pose2d currentPose2d = startingPose2d;
    private Pose2d previousPose2d = startingPose2d;

    private Vector2d currentVelocity = new Vector2d();
    private Vector2d previousVelocity = new Vector2d();
    private Vector2d currentAcceleration = new Vector2d();

    private double xOffset = 0;
    private double yOffset = 0;
    private double headingOffset = 0;

    private long previousPose2dTime;
    private long currentPose2dTime;

    /**
     * Creates a new Pose2dUpdater from a HardwareMap and a Localizer.
     *
     * @param hardwareMap the HardwareMap
     * @param localizer the Localizer
     */
    public Pose2dUpdater(HardwareMap hardwareMap, Telemetry telemetry, Localizer localizer) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.localizer = localizer;
        imu = localizer.getIMU();
    }

    /**
     * Creates a new Pose2dUpdater from a HardwareMap.
     *
     * @param hardwareMap the HardwareMap
     */
    public Pose2dUpdater(HardwareMap hardwareMap, Telemetry telemetry) {
        // TODO: replace the second argument with your preferred localizer
        //this(hardwareMap, new ThreeWheelLocalizer(hardwareMap));
        this(hardwareMap, telemetry, new MultiSensorFusionLocalization(hardwareMap,telemetry));
    }

    /**
     * This updates the robot's pose, as well as updating the previous pose, velocity, and
     * acceleration. The cache for the current pose, velocity, and acceleration is cleared, and
     * the time stamps are updated as well.
     */
    public void update() {
        previousVelocity = getVelocity();
        previousPose2d = applyOffset(getRawPose2d());
        currentPose2d = null;
        currentVelocity = null;
        currentAcceleration = null;
        previousPose2dTime = currentPose2dTime;
        currentPose2dTime = System.nanoTime();
        localizer.update();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param set the Pose2d to set the starting pose to.
     */
    public void setStartingPose2d(Pose2d set) {
        startingPose2d = set;
        previousPose2d = startingPose2d;
        previousPose2dTime = System.nanoTime();
        currentPose2dTime = System.nanoTime();
        localizer.setStartPose2d(set);
    }

    /**
     * This sets the current pose, using offsets. Think of using offsets as setting trim in an
     * aircraft. This can be reset as well, so beware of using the resetOffset() method.
     *
     *
     * @param set The pose to set the current pose to.
     */
    public void setCurrentPose2dWithOffset(Pose2d set) {
        Pose2d currentPose2d = getRawPose2d();
        setXOffset(set.getX() - currentPose2d.getX());
        setYOffset(set.getY() - currentPose2d.getY());
        setHeadingOffset(Maths.getTurnDirection(currentPose2d.getHeading(), set.getHeading()) * Maths.getSmallestAngleDifference(currentPose2d.getHeading(), set.getHeading()));
    }

    /**
     * This sets the offset for only the x position.
     *
     * @param offset This sets the offset.
     */
    public void setXOffset(double offset) {
        xOffset = offset;
    }

    /**
     * This sets the offset for only the y position.
     *
     * @param offset This sets the offset.
     */
    public void setYOffset(double offset) {
        yOffset = offset;
    }

    /**
     * This sets the offset for only the heading.
     *
     * @param offset This sets the offset.
     */
    public void setHeadingOffset(double offset) {
        headingOffset = offset;
    }

    /**
     * This returns the x offset.
     *
     * @return returns the x offset.
     */
    public double getXOffset() {
        return xOffset;
    }

    /**
     * This returns the y offset.
     *
     * @return returns the y offset.
     */
    public double getYOffset() {
        return yOffset;
    }

    /**
     * This returns the heading offset.
     *
     * @return returns the heading offset.
     */
    public double getHeadingOffset() {
        return headingOffset;
    }

    /**
     * This applies the offset to a specified Pose2d.
     *
     * @param pose The pose to be offset.
     * @return This returns a new Pose2d with the offset applied.
     */
    public Pose2d applyOffset(Pose2d pose) {
        return new Pose2d(pose.getX()+xOffset, pose.getY()+yOffset, pose.getHeading()+headingOffset);
    }

    /**
     * This resets all offsets set to the Pose2dUpdater. If you have reset your pose using the
     * setCurrentPose2dUsingOffset(Pose2d set) method, then your pose will be returned to what the
     * Pose2dUpdater thinks your pose would be, not the pose you reset to.
     */
    public void resetOffset() {
        setXOffset(0);
        setYOffset(0);
        setHeadingOffset(0);
    }

    /**
     * This returns the current pose, with offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the current pose.
     */
    public Pose2d getPose2d() {
        if (currentPose2d == null) {
            currentPose2d = localizer.getPose2d();
            return applyOffset(currentPose2d);
        } else {
            return applyOffset(currentPose2d);
        }
    }

    /**
     * This returns the current raw pose, without any offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the raw pose.
     */
    public Pose2d getRawPose2d() {
        if (currentPose2d == null) {
            currentPose2d = localizer.getPose2d();
            return currentPose2d;
        } else {
            return currentPose2d;
        }
    }

    /**
     * This sets the current pose without using resettable offsets.
     *
     * @param set the pose to set the current pose to.
     */
    public void setPose2d(Pose2d set) {
        resetOffset();
        localizer.setPose2d(set);
    }

    /**
     * Returns the robot's pose from the previous update.
     *
     * @return returns the robot's previous pose.
     */
    public Pose2d getPreviousPose2d() {
        return previousPose2d;
    }

    /**
     * Returns the robot's change in pose from the previous update.
     *
     * @return returns the robot's delta pose.
     */
    public Pose2d getDeltaPose2d() {
        Pose2d returnPose2d = getPose2d();
        returnPose2d.subtract(previousPose2d);
        return returnPose2d;
    }

    /**
     * This returns the velocity of the robot as a Vector2d. If this is called multiple times in
     * a single update, the velocity Vector2d is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the velocity of the robot.
     */
    public Vector2d getVelocity() {
        if (currentVelocity == null) {
//            currentVelocity = new Vector2d();
//            currentVelocity.setOrthogonalComponents(getPose2d().getX() - previousPose2d.getX(), getPose2d().getY() - previousPose2d.getY());
//            currentVelocity.setMagnitude(Maths.distance(getPose2d(), previousPose2d) / ((currentPose2dTime - previousPose2dTime) / Math.pow(10.0, 9)));
            currentVelocity = localizer.getVelocity2dVector();
            //return Maths.copyVector2d(currentVelocity);
            return currentVelocity.copy();
        } else {
            //return Maths.copyVector2d(currentVelocity);
            return currentVelocity.copy();
        }
    }

    /**
     * This returns the angular velocity of the robot as a double.
     *
     * @return returns the angular velocity of the robot.
     */
    public double getAngularVelocity() {
        return Maths.getTurnDirection(previousPose2d.getHeading(), getPose2d().getHeading()) * Maths.getSmallestAngleDifference(getPose2d().getHeading(), previousPose2d.getHeading()) / ((currentPose2dTime-previousPose2dTime)/Math.pow(10.0, 9));
    }

    /**
     * This returns the acceleration of the robot as a Vector2d. If this is called multiple times in
     * a single update, the acceleration Vector2d is cached so that subsequent calls don't have to
     * repeat localizer calls or calculations.
     *
     * @return returns the acceleration of the robot.
     */
    public Vector2d getAcceleration() {
        if (currentAcceleration == null) {
            currentAcceleration = getVelocity().minus(previousVelocity);
            currentAcceleration.setMagnitude(currentAcceleration.getMagnitude() / ((currentPose2dTime - previousPose2dTime) / Math.pow(10.0, 9)));
            return currentAcceleration.copy();
        } else {
            return currentAcceleration.copy();
        }
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using Road Runner's pose reset.
     */
    public void resetHeadingToIMU() {
        if (imu != null) {
            localizer.setPose2d(new Pose2d(getPose2d().getX(), getPose2d().getY(), getNormalizedIMUHeading() + startingPose2d.getHeading()));
        }
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using offsets instead of Road
     * Runner's pose reset. This way, it's faster, but this can be wiped with the resetOffsets()
     * method.
     */
    public void resetHeadingToIMUWithOffsets() {
        if (imu != null) {
            setCurrentPose2dWithOffset(new Pose2d(getPose2d().getX(), getPose2d().getY(), getNormalizedIMUHeading() + startingPose2d.getHeading()));
        }
    }

    /**
     * This returns the IMU heading normalized to be between [0, 2 PI] radians.
     *
     * @return returns the normalized IMU heading.
     */
    public double getNormalizedIMUHeading() {
        if (imu != null) {
            return Maths.normalizeAngle(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }
        return 0;
    }

    /**
     * This returns the total number of radians the robot has turned.
     *
     * @return the total heading.
     */
    public double getTotalHeading() {
        return localizer.getTotalHeading();
    }

    /**
     * This returns the Localizer.
     *
     * @return the Localizer
     */
    public Localizer getLocalizer() {
        return localizer;
    }

    /**
     *
     */
    public void resetIMU() throws InterruptedException {
        localizer.resetIMU();
    }
}