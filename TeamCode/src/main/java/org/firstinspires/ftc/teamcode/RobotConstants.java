package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.EncodersEx;

import java.util.Arrays;
import java.util.List;

@Config
public class RobotConstants {
    /*public static double motorLeftDir = -1;
    public static double motorRightDir = 1;*/
    /**
     * @param startMovingAngle is the distance away from the required module angle for the wheel to start moving
     *                         You don't want the wheels to move in the wrong direction
     *                         if module turning speed is fast enough you shouldn't notice this
     */
    public static double startMovingAngle = 5;

    public static boolean doDiffySwerve = false;

    public static boolean usingServoForEncoders = true;

    /** FOR DIFFY
     * Encoder is back of module, and then motor on left is Left motor, and same for right motor
     * Encoders are one the encoder port for the module's Left motor
     **/
    /** FOR COAX
     * No dedicated encoders, servos are encoders
     * Right is drive motor
     * Left is servoMotor
     * SERVO AND DRIVE MOTOR NEED TO BE IN SAME POSITION IN RESPECTIVE LISTS
     **/
    public static DcMotorEx module1Right, module2Right;
    public static DcMotorEx[] motorsArray = new DcMotorEx[] {module1Right,module2Right};
    public static final List<DcMotorEx> motors = Arrays.asList(motorsArray);
    public static String[] motorNamesArray = new String[] {"module1Right", "module2Right"}; //"module1Right", "module2Right"
    public static final List<String> motorNames = Arrays.asList(motorNamesArray);
    public static DcMotorEx[] reversedMotorsArray = new DcMotorEx[] {};
    public static final List<DcMotorEx> reversedMotors = Arrays.asList(reversedMotorsArray);

    public static Servo module1Left, module2Left;
    public static Servo[] servosArray = new Servo[] {module1Left,module2Left};
    public static final List<Servo> servos = Arrays.asList(servosArray);
    public static String[] servoNamesArray = new String[] {"module1Left", "module2Left"}; //"module1Left", "module2Left"
    public static final List<String> servoNames = Arrays.asList(servoNamesArray);
    public static Servo[] reversedServosArray = new Servo[] {};
    public static final List<Servo> reversedServos = Arrays.asList(reversedServosArray);

    /**
     * module1Encoder = module1Left
     * module2Encoder = module2Left
     **/
    /*
    public static DcMotorEx module1Encoder, module2Encoder;
    public static DcMotorEx[] moduleEncodersArray = new DcMotorEx[] {module1Encoder,module2Encoder};
    public static final List<DcMotorEx> moduleEncoders = Arrays.asList(moduleEncodersArray);
    public static String[] moduleEncoderNamesArray = new String[] {"module1Left", "module2Left"};
    public static final List<String> moduleEncoderNames = Arrays.asList(moduleEncoderNamesArray);
    public static DcMotorEx[] reversedEncodersArray = new DcMotorEx[] {};
    public static final List<DcMotorEx> reversedEncoders = Arrays.asList(reversedEncodersArray);
    */
    public static Servo module1Encoder, module2Encoder;
    public static Servo[] moduleEncodersArray = new Servo[] {module1Encoder,module2Encoder};
    public static final List<Servo> moduleEncoders = Arrays.asList(moduleEncodersArray);
    public static String[] moduleEncoderNamesArray = new String[] {"module1Left", "module2Left"}; //"module1Left", "module2Left"
    public static final List<String> moduleEncoderNames = Arrays.asList(moduleEncoderNamesArray);
    public static Servo[] reversedEncodersArray = new Servo[] {};
    public static final List<Servo> reversedEncoders = Arrays.asList(reversedEncodersArray);

    public static double[] encoderZeroValues = new double[] {0.03,0.07};

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

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

    /**
     * [n][x,y]
     * @param n: is the value for the first array, and is the module number (i.e. module 1, module 2, ect.)
     * @param x: is the first value in the nested array, and is the distance from the center of the module's wheel away from the centerline of the robot
     *         distance left(negative) or right(positive) if you are behind the robot and look in the direction the robot faces
     * @param y: is the second value in the nested array, and is the distance from the center of the module's wheel to the middle of the robot
     *         distance forward(positive) or backward(negative) if you are behind the robot and look in the direction the robot faces
     *         distance right(positive) or left(negative) if you are on the left side of the robot look at the right side of the robot
     */
    // LEFT MODULE IS FIRST
    public static final double[][] modulePositions = new double[][] { // x,y (unit: inches) (left module is first) (positive is right and up)
            {-4.5,0},
            {4.5,0}
    };
}
