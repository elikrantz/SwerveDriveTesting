package org.firstinspires.ftc.teamcode.krantzPathing;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.EncoderEx;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.krantzPathing.maths.*;
import org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.Filters.KalmanFilterMultiParameters;

@Config
public class RobotConstants {
    /*public static double motorLeftDir = -1;
    public static double motorRightDir = 1;*/
    public static double startMovingAngle = 5;

    public static boolean doSwerve = false;
    public static boolean doDiffySwerve = true;

    public static double ROBOT_WIDTH = 12;
    public static double ROBOT_LENGTH = 15;

    public static double FORWARD_TICKS_TO_INCHES = 8192 * 1.37795 * 2 * Math.PI * 0.5008239963;
    public static double STRAFE_TICKS_TO_INCHES = 8192 * 1.37795 * 2 * Math.PI * 0.5018874659;
    public static double TURN_TICKS_TO_RADIANS = 8192 * 1.37795 * 2 * Math.PI * 0.5;

    /** FOR DIFFY
     * Encoder is back of module, and then motor on left is Left motor, and same for right motor
     * Encoders are one the encoder port for the module's Left motor
     **/
    /** FOR COAX
     * No encoders, servo is encoder
     * Right is drive motor
     * Left is servoMotor
     * SERVO AND DRIVE MOTOR NEED TO BE IN SAME POSITION IN RESPECTIVE LISTS
     **/
    public static DcMotorEx module1Right, module2Right;
    public static List<DcMotorEx> motors = Arrays.asList(module1Right, module2Right);
    public static List<String> motorNames = Arrays.asList("module1Right", "module2Right");
    public static List<DcMotorEx> reversedMotors = Arrays.asList();

    public static Servo module1Left, module2Left;
    public static List<Servo> servos = Arrays.asList(module1Left, module2Left);
    public static List<String> servoNames = Arrays.asList("module1Left", "module2Left");
    public static List<DcMotorEx> reversedServos = Arrays.asList();

//    public static DcMotorEx module1Encoder, module2Encoder;
//    public static List<DcMotorEx> moduleEncoders = Arrays.asList(module1Encoder, module2Encoder);
//    /**
//     * module1Encoder = module1Left
//     * module2Encoder = module2Left
//     **/
//    public static List<String> moduleEncoderNames = Arrays.asList("module1Left", "module2Left");
    // public static EncodersEx module1Encoder, module2Encoder;
    // public static List<EncodersEx> moduleEncoders = Arrays.asList(module1Encoder, module2Encoder);
    // /**
    //  * module1Encoder = module1Left
    //  * module2Encoder = module2Left
    //  **/
    // public static List<String> moduleEncoderNames = Arrays.asList("module1Left", "module2Left");

    public static EncoderEx module1Encoder, module2Encoder;
    public static List<EncoderEx> moduleEncoders = Arrays.asList(module1Encoder, module2Encoder);
    public static List<String> moduleEncoderNames = Arrays.asList("module1Left", "module2Left");

    public static EncoderEx frontLeft, frontRight, backLeft, backRight;
    public static List<EncoderEx> driveWheelEncoders = Arrays.asList(frontLeft, frontRight, backLeft, backRight);
    public static List<String> driveWheelEncoderNames = Arrays.asList("frontLeft", "frontRight", "backLeft", "backRight");

    public static EncoderEx deadWheelLeft, deadWheelRight, deadWheelSide, deadWheelForward;
    public static List<EncoderEx> deadWheelEncoders = Arrays.asList(deadWheelLeft, deadWheelRight, deadWheelSide,deadWheelForward);
    public static List<String> deadWheelEncoderNames = Arrays.asList("deadWheelLeft", "deadWheelRight", "deadWheelSide","deadWheelForward");

    public static List<EncoderEx> reversedEncoders = Arrays.asList();


    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    public static Position cameraPos = new Position(DistanceUnit.INCH, 3.7, 3.7, 0, 0);
    public static YawPitchRollAngles cameraRotation = new YawPitchRollAngles(AngleUnit.DEGREES, 0,-90,0,0);

    public static double rotationTest = 0;

    public static double pulsesPerRevEncoder = 22.5;

    public static final int numberOfModules = 2;
    public static final double gearRatioLeftMotor2Wheel = 1;
    public static final double gearRatioRightMotor2Wheel = 1;
    public static final double gearRatio2Encoder = 0.5;

    public static double powerCutOff = 0.05;

    public static double[][] PIDvals = new double[][] {
            {0.01,0,0},
            {0.01,0,0}
    };

    public static KalmanFilterMultiParameters sensorFusionParametersX = new KalmanFilterMultiParameters(
        6,
        1,1,1,1
    );
    public static KalmanFilterMultiParameters sensorFusionParametersY = new KalmanFilterMultiParameters(
        6,
        1,1,1,1
    );
    public static KalmanFilterMultiParameters sensorFusionParametersRot = new KalmanFilterMultiParameters(
        6,
        1,1,1,1
    );

    public static KalmanFilterMultiParameters sensorFusionParametersVelocityX = new KalmanFilterMultiParameters(
        6,
        1,1,1,1
    );
    public static KalmanFilterMultiParameters sensorFusionParametersVelocityY = new KalmanFilterMultiParameters(
        6,
        1,1,1,1
    );
    public static KalmanFilterMultiParameters sensorFusionParametersVelocityRot = new KalmanFilterMultiParameters(
        6,
        1,1,1,1
    );

    public static Pose2d[] deadWheelPositions = new Pose2d[]{
        new Pose2d(0,0,Math.toRadians(0)), //left dead wheel
        new Pose2d(0,0,Math.toRadians(0)), //right dead wheel
        new Pose2d(0,0,Math.toRadians(90)) // turning dead wheel
    };

    public static final Vector2d[] driveWheelsPositions = new Vector2d[]{
        new Vector2d(0,0),
        new Vector2d(0,0),
        new Vector2d(0,0),
        new Vector2d(0,0)
    };

    public static final double[][] modulePositions = new double[][] { // x,y (unit: inches) (left module is first) (positive is right and up)
            {-4,0},
            {0,0}
    };
}