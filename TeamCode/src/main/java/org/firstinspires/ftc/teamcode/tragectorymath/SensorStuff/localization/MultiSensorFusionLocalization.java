package org.firstinspires.ftc.teamcode.tragectorymath.SensorStuff.localization;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tragectorymath.RobotConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import org.firstinspires.ftc.teamcode.tragectorymath.SensorStuff.Filters.*;
import org.firstinspires.ftc.teamcode.tragectorymath.SensorStuff.Localizer;
import org.firstinspires.ftc.teamcode.tragectorymath.maths.*;
import org.firstinspires.ftc.teamcode.tragectorymath.util.NanoTimer;

@Config
public class MultiSensorFusionLocalization extends Localizer {
    private driveEncodersLocalization localizerDrive;
    private AprilTagLocalization localizerAprilTag;
    private deadWheelsThreeLocalization localizerDead3;
    private deadWheelsTwoLocalization localizerDead2;
    private imuLocalization localizerIMU;
    private Localizer[] localizers;
    //private Localizer localizer;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Pose2d startPose2d;
    private Pose2d displacementPose2d;
    private Pose2d currentPose2d;
    private Pose2d currentVelocity;
    //private Matrix prevRotationMatrix;
    //private double currentHeading;
    private double totalHeading;
    private NanoTimer timer;
    private long deltaTimeNano;

    public final IMU imu;

    public static double FORWARD_TICKS_TO_INCHES = RobotConstants.STRAFE_TICKS_TO_INCHES;
    public static double STRAFE_TICKS_TO_INCHES = RobotConstants.STRAFE_TICKS_TO_INCHES;
    public static double TURN_TICKS_TO_RADIANS = RobotConstants.TURN_TICKS_TO_RADIANS;

    private KalmanFilterMulti sensorFusionFilterX = new KalmanFilterMulti(RobotConstants.sensorFusionParametersX);
    private KalmanFilterMulti sensorFusionFilterY = new KalmanFilterMulti(RobotConstants.sensorFusionParametersY);
    private KalmanFilterMulti sensorFusionFilterRot = new KalmanFilterMulti(RobotConstants.sensorFusionParametersRot);
    private KalmanFilterMulti sensorFusionFilterVelocityX = new KalmanFilterMulti(RobotConstants.sensorFusionParametersVelocityX);
    private KalmanFilterMulti sensorFusionFilterVelocityY = new KalmanFilterMulti(RobotConstants.sensorFusionParametersVelocityY);
    private KalmanFilterMulti sensorFusionFilterVelocityRot = new KalmanFilterMulti(RobotConstants.sensorFusionParametersVelocityRot);
    
    KalmanFilterMulti imuSensorFilteringX, imuSensorFilteringY, imuSensorFilteringRot, imuSensorFilteringVelocityX, imuSensorFilteringVelocityY, imuSensorFilteringVelocityRot;
    private KalmanFilterMulti[] imuSensorFiltering = new KalmanFilterMulti[] {
            imuSensorFilteringX = new KalmanFilterMulti(new KalmanFilterMultiParameters(6,1)),
            imuSensorFilteringY = new KalmanFilterMulti(new KalmanFilterMultiParameters(6,1)),
            imuSensorFilteringRot = new KalmanFilterMulti(new KalmanFilterMultiParameters(6,1)),
            imuSensorFilteringVelocityX = new KalmanFilterMulti(new KalmanFilterMultiParameters(6,1)),
            imuSensorFilteringVelocityY = new KalmanFilterMulti(new KalmanFilterMultiParameters(6,1)),
            imuSensorFilteringVelocityRot = new KalmanFilterMulti(new KalmanFilterMultiParameters(6,1))
    };
    
    public MultiSensorFusionLocalization(HardwareMap hardwareMap, Telemetry telemetry, Pose2d setStartPose2d) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
            RobotConstants.LOGO_FACING_DIR, RobotConstants.USB_FACING_DIR)));

        this.startPose2d = setStartPose2d;
        this.localizers = new Localizer[] {
            localizerDrive = new driveEncodersLocalization(hardwareMap),
            localizerAprilTag = new AprilTagLocalization(hardwareMap,telemetry),
            localizerDead3 = new deadWheelsThreeLocalization(hardwareMap),
            //localizerDead2 = new deadWheelsTwoLocalization(hardwareMap),
            localizerIMU = new imuLocalization(hardwareMap) // Not currently functional?
        };
        setStartPose2d(setStartPose2d);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose2d = new Pose2d();
        currentPose2d = new Pose2d();
        currentVelocity = new Pose2d();
        totalHeading = 0;
    }

    public MultiSensorFusionLocalization(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, new Pose2d());
    }

    // private Object iterate(Object[] listObject) {
    //     for (Object object : listObject) {
    //         Object output = object;
    //         return output;
    //     }
    // }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose2d
     */
    @Override
    public Pose2d getPose2d() {
        //return startPose2d.add(displacementPose2d);
        return currentPose2d;
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
        // for (Localizer localizer : localizers) {
        //     localizer.setStartPose2d(setStart);
        // }
        //Arrays.asList(localizers).stream().map(a -> a.setStartPose2d(setStart));
        for (Localizer localizer : localizers) { localizer.setStartPose2d(setStart); }
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose2d the new current pose estimate
     */
    @Override
    public void setPose2d(Pose2d setPose2d) {
        //displacementPose2d = setPose2d.subtract(startPose2d);
        currentPose2d = setPose2d;
        // for (Localizer localizer : localizers) {
        //     localizer.setStartPose2d(setStart);
        // }
        //Arrays.asList(localizers).stream().map(a -> a.setPose2d(setPose2d));
        for (Localizer localizer : localizers) { localizer.setPose2d(setPose2d); }
        //localizers.resetEncoders();
        //Arrays.asList(localizers).stream().map(a -> a.resetEncoders());
        resetEncoders();
    }

    @Override
    public void resetEncoders() {
        for (Localizer localizer : localizers) { localizer.resetEncoders(); }
    }

    public void filterSensorData(Pose2d sensorDataPos, Pose2d sensorDataVelocity, KalmanFilterMulti[] filter, @NonNull Pose2d outputDataPos, @NonNull Pose2d outputDataVelocity) {
        // X Filtering
        filter[0].runLoop(new double[] {sensorDataPos.getX()});
        outputDataPos.setX(filter[0].getState());
        // Y Filtering
        filter[1].runLoop(new double[] {sensorDataPos.getY()});
        outputDataPos.setY(filter[1].getState());
        // Rotation Filtering
        filter[2].runLoop(new double[] {sensorDataPos.getHeading()});
        outputDataPos.setHeading(filter[2].getState());
        // Velocity X Filtering
        filter[3].runLoop(new double[] {sensorDataVelocity.getX()});
        outputDataVelocity.setX(filter[3].getState());
        // Velocity Y Filtering
        filter[4].runLoop(new double[] {sensorDataVelocity.getY()});
        outputDataVelocity.setY(filter[4].getState());
        // Velocity Rot Filtering
        filter[5].runLoop(new double[] {sensorDataVelocity.getHeading()});
        outputDataVelocity.setHeading(filter[5].getState());
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

        //Arrays.asList(localizers).stream().map(a -> a.update());
        for (Localizer localizer : localizers) { localizer.update(); }
        //localizers.update();
        
        //Pose2d sensorPos2dEstimates = localizers.getPose2d();
        Pose2d[] sensorPos2dEstimates = new Pose2d[localizers.length];
        for (int i = 0; i < localizers.length; i++) { sensorPos2dEstimates[i] = Arrays.asList(localizers).stream().map(a -> a.getPose2d()).collect(Collectors.toList()).get(i); }
        
        // Velocity filtering
        Pose2d[] sensorVelocityEstimates = new Pose2d[localizers.length];
        for (int i = 0; i < localizers.length; i++) { sensorVelocityEstimates[i] = Arrays.asList(localizers).stream().map(a -> a.getVelocity2d()).collect(Collectors.toList()).get(i); }
        
        // Individual Sensor Data Filtering
        filterSensorData(localizerIMU.getPose2d(), localizerIMU.getVelocity2d(), imuSensorFiltering, sensorPos2dEstimates[3], sensorVelocityEstimates[3]); // Comment out, THIS IS AN EXAMPLE

        // X direction filtering
        double[] xSensorVals = new double[localizers.length];
        for (int i = 0; i < localizers.length; i++) { xSensorVals[i] = Arrays.asList(sensorPos2dEstimates).stream().map(a -> a.getX()).collect(Collectors.toList()).get(i); }
        sensorFusionFilterX.runLoop(xSensorVals);
        currentPose2d.setX(sensorFusionFilterX.getState());

        // Y direction filtering
        double[] ySensorVals = new double[localizers.length];
        for (int i = 0; i < localizers.length; i++) { ySensorVals[i] = Arrays.asList(sensorPos2dEstimates).stream().map(a -> a.getY()).collect(Collectors.toList()).get(i); }
        sensorFusionFilterY.runLoop(ySensorVals);
        currentPose2d.setY(sensorFusionFilterY.getState());

        // Rotation filtering
        double[] rotSensorVals = new double[localizers.length];
        for (int i = 0; i < localizers.length; i++) { rotSensorVals[i] = Arrays.asList(sensorPos2dEstimates).stream().map(a -> a.getHeading()).collect(Collectors.toList()).get(i); }
        sensorFusionFilterRot.runLoop(rotSensorVals);
        currentPose2d.setHeading(sensorFusionFilterRot.getState());
        
        // Velocity X filtering
        double[] xVelocitySensorVals = new double[localizers.length];
        for (int i = 0; i < localizers.length; i++) { xVelocitySensorVals[i] = Arrays.asList(sensorVelocityEstimates).stream().map(a -> a.getX()).collect(Collectors.toList()).get(i); }
        sensorFusionFilterVelocityX.runLoop(xVelocitySensorVals);
        currentVelocity.setX(sensorFusionFilterVelocityX.getState());

        // Velocity Y Filtering
        double[] yVelocitySensorVals = new double[localizers.length];
        for (int i = 0; i < localizers.length; i++) { yVelocitySensorVals[i] = Arrays.asList(sensorVelocityEstimates).stream().map(a -> a.getY()).collect(Collectors.toList()).get(i); }
        sensorFusionFilterVelocityY.runLoop(yVelocitySensorVals);
        currentVelocity.setX(sensorFusionFilterVelocityY.getState());

        // Velocity Rot Filtering
        double[] rotVelocitySensorVals = new double[localizers.length];
        for (int i = 0; i < localizers.length; i++) { rotVelocitySensorVals[i] = Arrays.asList(sensorVelocityEstimates).stream().map(a -> a.getHeading()).collect(Collectors.toList()).get(i); }
        sensorFusionFilterVelocityRot.runLoop(rotVelocitySensorVals);
        currentVelocity.setHeading(sensorFusionFilterVelocityRot.getState());

        //totalHeading += currentPose2d.getHeading();
        totalHeading = currentPose2d.getHeading();
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
        imu.resetYaw();
    }

    /**
     * This is returns the IMU.
     *
     * @return returns the IMU
     */
    @Override
    public IMU getIMU() {
        return imu;
    }
}