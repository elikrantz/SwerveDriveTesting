package org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.krantzPathing.maths.*;


public abstract class Localizer {

    /**
     * This returns the current pose2d estimate from the Localizer.
     *
     * @return returns the pose2d as a Pose2d object.
     */
    public abstract Pose2d getPose2d();

    /**
     * This returns the current velocity estimate from the Localizer.
     *
     * @return returns the velocity as a Pose2d object.
     */
    public abstract Pose2d getVelocity2d();

    /**
     * This returns the current velocity estimate from the Localizer as a Vector.
     *
     * @return returns the velocity as a Vector2d.
     */
    public abstract Vector2d getVelocity2dVector();

    /**
     * This sets the start pose2d of the Localizer. Changing the start pose2d should move the robot as if
     * all its previous movements were displacing it from its new start pose2d.
     *
     * @param setStart the new start pose2d
     */
    public abstract void setStartPose2d(Pose2d setStart);

    /**
     * This sets the current pose2d estimate of the Localizer. Changing this should just change the
     * robot's current pose2d estimate, not anything to do with the start pose2d.
     *
     * @param setPose2d the new current pose2d estimate
     */
    public abstract void setPose2d(Pose2d setPose2d);

    /**
     * This calls an update to the Localizer, updating the current pose2d estimate and current velocity
     * estimate.
     */
    public abstract void update();

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public abstract double getTotalHeading();

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public abstract double getForwardMultiplier();

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public abstract double getLateralMultiplier();

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public abstract double getTurningMultiplier();

    /**
     * This resets the Encoders.
     */
    public abstract void resetEncoders();

    /**
     * This resets the IMU of the localizer, if applicable.
     */
    public abstract void resetIMU() throws InterruptedException;

    /**
     * This is overridden to return the IMU, if there is one.
     *
     * @return returns the IMU if it exists
     */
    public IMU getIMU() {
        return null;
    }
}