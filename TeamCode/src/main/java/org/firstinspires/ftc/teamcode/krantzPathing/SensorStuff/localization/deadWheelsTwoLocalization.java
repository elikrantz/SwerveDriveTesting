package org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.localization;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.krantzPathing.RobotConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.EncoderEx;
import org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.Localizer;
import org.firstinspires.ftc.teamcode.krantzPathing.maths.*;
import org.firstinspires.ftc.teamcode.krantzPathing.util.NanoTimer;

@Config
public class deadWheelsTwoLocalization extends Localizer {
    private HardwareMap hardwareMap;
    private Pose2d startPose2d;
    private Pose2d displacementPose2d;
    private Pose2d currentVelocity;
    private Matrix prevRotationMatrix;
    private NanoTimer timer;
    private long deltaTimeNano;
    private IMU imu;
    private double previousIMUOrientation;
    private EncoderEx forwardEncoder;
    private EncoderEx strafeEncoder;
    private EncoderEx[] deadWheelEncoders = new EncoderEx[RobotConstants.deadWheelEncoders.size()];
    private Pose2d forwardEncoderPose;
    private Pose2d strafeEncoderPose;
    private double totalHeading;
    private double deltaRadians;
    public static double FORWARD_TICKS_TO_INCHES = RobotConstants.STRAFE_TICKS_TO_INCHES;
    public static double STRAFE_TICKS_TO_INCHES = RobotConstants.STRAFE_TICKS_TO_INCHES;

    /**
     * This creates a new TwoWheelLocalizer from a HardwareMap, with a starting Pose2d at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public deadWheelsTwoLocalization(HardwareMap map) {
        this(map, new Pose2d());
    }

    /**
     * This creates a new TwoWheelLocalizer from a HardwareMap and a Pose2d, with the Pose2d
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose2d the Pose2d to start from
     */
    public deadWheelsTwoLocalization(HardwareMap map, Pose2d setStartPose2d) {
        // TODO: replace these with your encoder positions
        forwardEncoderPose = RobotConstants.deadWheelPositions[0];
        strafeEncoderPose = RobotConstants.deadWheelPositions[1];

        hardwareMap = map;

        imu = hardwareMap.get(IMU.class, "imu");

        // TODO: replace this with your IMU's orientation
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotConstants.LOGO_FACING_DIR, RobotConstants.USB_FACING_DIR)));

        // TODO: replace these with your encoder ports
        for (int encoderNum = 0; encoderNum < deadWheelEncoders.length; encoderNum++) {
            deadWheelEncoders[encoderNum] = new EncoderEx(hardwareMap,RobotConstants.deadWheelEncoderNames.get(encoderNum));
            if (RobotConstants.reversedEncoders.contains(deadWheelEncoders[encoderNum])) { deadWheelEncoders[encoderNum].setDirection(EncoderEx.Direction.REVERSE); }
        }

        forwardEncoder = RobotConstants.deadWheelForward;
        strafeEncoder = RobotConstants.deadWheelSide;

        // TODO: reverse any encoders necessary
        forwardEncoder.setDirection(EncoderEx.Direction.REVERSE);
        strafeEncoder.setDirection(EncoderEx.Direction.FORWARD);

        setStartPose2d(setStartPose2d);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose2d = new Pose2d();
        currentVelocity = new Pose2d();
        totalHeading = 0;
        deltaRadians = 0;

        resetEncoders();
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose2d
     */
    @Override
    public Pose2d getPose2d() {
        return startPose2d.add(displacementPose2d);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose2d
     */
    @Override
    public Pose2d getVelocity2d() {
        return currentVelocity.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector2d
     */
    @Override
    public Vector2d getVelocity2dVector() {
        return currentVelocity.getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose2d(Pose2d setStart) {
        startPose2d = setStart;
    }

    /**
     * This sets the Matrix that contains the previous pose's heading rotation.
     *
     * @param heading the rotation of the Matrix
     */
    public void setPrevRotationMatrix(double heading) {
        prevRotationMatrix = new Matrix(3,3);
        prevRotationMatrix.set(0, 0, Math.cos(heading));
        prevRotationMatrix.set(0, 1, -Math.sin(heading));
        prevRotationMatrix.set(1, 0, Math.sin(heading));
        prevRotationMatrix.set(1, 1, Math.cos(heading));
        prevRotationMatrix.set(2, 2, 1.0);
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose2d the new current pose estimate
     */
    @Override
    public void setPose2d(Pose2d setPose2d) {
        displacementPose2d = setPose2d.subtract(startPose2d);
        resetEncoders();
    }

     /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders. Then, the robot's global change in position is calculated
     * using the pose exponential method.
     */
    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas;
        setPrevRotationMatrix(getPose2d().getHeading());

        Matrix transformation = new Matrix(3,3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), robotDeltas);

        displacementPose2d.add(new Pose2d(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        currentVelocity = new Pose2d(globalDeltas.get(0, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano / Math.pow(10.0, 9)));

        totalHeading += globalDeltas.get(2, 0);
    }

    /**
     * This updates the Encoders.
     */
    public void updateEncoders() {
        forwardEncoder.update();
        strafeEncoder.update();

        double currentIMUOrientation = Maths.normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        deltaRadians = Maths.getTurnDirection(previousIMUOrientation, currentIMUOrientation) * Maths.getSmallestAngleDifference(currentIMUOrientation, previousIMUOrientation);
        previousIMUOrientation = currentIMUOrientation;
    }

    /**
     * This resets the Encoders.
     */
    public void resetEncoders() {
        forwardEncoder.reset();
        strafeEncoder.reset();
    }

    /**
     * This calculates the change in position from the perspective of the robot using information
     * from the Encoders and IMU.
     *
     * @return returns a Matrix containing the robot relative movement.
     */
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        returnMatrix.set(0,0, FORWARD_TICKS_TO_INCHES * (forwardEncoder.getDeltaPosition() - forwardEncoderPose.getY() * deltaRadians));
        //y/strafe movement
        returnMatrix.set(1,0, STRAFE_TICKS_TO_INCHES * (strafeEncoder.getDeltaPosition() - strafeEncoderPose.getX() * deltaRadians));
        // theta/turning
        returnMatrix.set(2,0, deltaRadians);
        return returnMatrix;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return FORWARD_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return STRAFE_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return 1;
    }

    /**
     * This does nothing since this localizer does not use the IMU.
     */
    public void resetIMU() {
    }
}