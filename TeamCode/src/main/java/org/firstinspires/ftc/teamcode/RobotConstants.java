package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.systems.EncodersEx;

import java.util.Arrays;
import java.util.List;

@Config
public class RobotConstants {
    /*public static double motorLeftDir = -1;
    public static double motorRightDir = 1;*/
    public static double startMovingAngle = 5;

    /**
     * Encoder is back of module, and then motor on left is Left motor, and same for right motor
     * Encoders are one the encoder port for the module's Left motor
     **/
    public static DcMotorEx module1Left, module1Right, module2Right, module2Left;
    public static List<DcMotorEx> motors = Arrays.asList(module1Left, module1Right, module2Right, module2Left);
    public static List<String> motorNames = Arrays.asList("module1Left", "module1Right", "module2Right", "module2Left");
    public static List<DcMotorEx> reversedMotors = Arrays.asList();

    public static DcMotorEx module1Encoder, module2Encoder;
    public static List<DcMotorEx> moduleEncoders = Arrays.asList(module1Encoder, module2Encoder);
    /**
     * module1Encoder = module1Left
     * module2Encoder = module2Left
     **/
    public static List<String> moduleEncoderNames = Arrays.asList("module1Left", "module2Left");

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
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

    public static final double[][] modulePositions = new double[][] { // x,y (unit: inches) (left module is first) (positive is right and up)
            {-4,0},
            {0,0}
    };
}
