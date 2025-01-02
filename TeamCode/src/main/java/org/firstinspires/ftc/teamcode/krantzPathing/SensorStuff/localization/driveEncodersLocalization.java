package org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.localization;

import org.firstinspires.ftc.teamcode.krantzPathing.RobotConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.EncoderEx;
import org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.Localizer;
import org.firstinspires.ftc.teamcode.krantzPathing.maths.*;
import org.firstinspires.ftc.teamcode.krantzPathing.util.NanoTimer;

@Config
public class driveEncodersLocalization extends Localizer {
    private HardwareMap hardwareMap;
    private Pose2d startPose2d;
    private Pose2d displacementPose2d;
    private Pose2d currentVelocity;
    private Matrix prevRotationMatrix;
    private NanoTimer timer;
    private long deltaTimeNano;
    private EncoderEx frontLeftEncoder;
    private EncoderEx frontRightEncoder;
    private EncoderEx backLeftEncoder;
    private EncoderEx backRightEncoder;
    private EncoderEx[] driveWheelEncoders = new EncoderEx[RobotConstants.driveWheelEncoders.size()];
    private double totalHeading;
    public static double FORWARD_TICKS_TO_INCHES = 1;
    public static double STRAFE_TICKS_TO_INCHES = 1;
    public static double TURN_TICKS_TO_RADIANS = 1;

    /**
     * This creates a new driveEncodersLocalization from a HardwareMap, with a starting Pose2d at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public driveEncodersLocalization(HardwareMap map) {
        this(map, new Pose2d());
    }
    /**
     * This creates a new driveEncodersLocalization from a HardwareMap and a Pose2d, with the Pose2d
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose2d the Pose2d to start from
     */
    public driveEncodersLocalization(HardwareMap map, Pose2d setStartPose2d) {
        hardwareMap = map;

        // RobotConstants.frontLeft = new Encoder(hardwareMap.get(DcMotorEx.class, leftFrontMotorName));
        // RobotConstants.backLeft = new Encoder(hardwareMap.get(DcMotorEx.class, leftRearMotorName));
        // RobotConstants.backRight = new Encoder(hardwareMap.get(DcMotorEx.class, rightRearMotorName));
        // RobotConstants.frontRight = new Encoder(hardwareMap.get(DcMotorEx.class, rightFrontMotorName));

        for (int encoderNum = 0; encoderNum < driveWheelEncoders.length; encoderNum++) {
            driveWheelEncoders[encoderNum] = new EncoderEx(hardwareMap,RobotConstants.driveWheelEncoderNames.get(encoderNum));
            if (RobotConstants.reversedEncoders.contains(driveWheelEncoders[encoderNum])) { driveWheelEncoders[encoderNum].setDirection(EncoderEx.Direction.REVERSE); }
        }

        frontLeftEncoder = RobotConstants.frontLeft;
        frontRightEncoder = RobotConstants.frontRight;
        backLeftEncoder = RobotConstants.backLeft;
        backRightEncoder = RobotConstants.backRight;

        // TODO: reverse any encoders necessary
         RobotConstants.frontLeft.setDirection(EncoderEx.Direction.REVERSE);
         RobotConstants.backLeft.setDirection(EncoderEx.Direction.REVERSE);
         RobotConstants.frontRight.setDirection(EncoderEx.Direction.FORWARD);
         RobotConstants.backRight.setDirection(EncoderEx.Direction.FORWARD);

        setStartPose2d(setStartPose2d);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose2d = new Pose2d();
        currentVelocity = new Pose2d();
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
        //driveWheelEncoders.getEncoderDegrees();
        frontLeftEncoder.update();
        frontRightEncoder.update();
        backLeftEncoder.update();
        backRightEncoder.update();
    }

    /**
     * This resets the Encoders.
     */
    public void resetEncoders() {
        //driveWheelEncoders.reset();
        frontLeftEncoder.reset();
        frontRightEncoder.reset();
        backLeftEncoder.reset();
        backRightEncoder.reset();
    }

    /**
     * This calculates the change in position from the perspective of the robot using information
     * from the Encoders.
     *
     * @return returns a Matrix containing the robot relative movement.
     */
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        returnMatrix.set(0,0, FORWARD_TICKS_TO_INCHES * (RobotConstants.frontLeft.getDeltaPosition() + RobotConstants.frontRight.getDeltaPosition() + RobotConstants.backLeft.getDeltaPosition() + RobotConstants.backRight.getDeltaPosition()));
        //y/strafe movement
        returnMatrix.set(1,0, STRAFE_TICKS_TO_INCHES * (-RobotConstants.frontLeft.getDeltaPosition() + RobotConstants.frontRight.getDeltaPosition() + RobotConstants.backLeft.getDeltaPosition() - RobotConstants.backRight.getDeltaPosition()));
        // theta/turning
        returnMatrix.set(2,0, TURN_TICKS_TO_RADIANS * ((-RobotConstants.frontLeft.getDeltaPosition() + RobotConstants.frontRight.getDeltaPosition() - RobotConstants.backLeft.getDeltaPosition() + RobotConstants.backRight.getDeltaPosition()) / (RobotConstants.ROBOT_WIDTH + RobotConstants.ROBOT_LENGTH)));
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
        return TURN_TICKS_TO_RADIANS;
    }

    /**
     * This does nothing since this localizer does not use the IMU.
     */
    public void resetIMU() {
    }
}