package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.systems.EncodersEx;

import java.util.Arrays;
import java.util.List;

@Config
public class RobotConstants {

    public static DcMotorEx module1Left, module1Right, module2Right, module2Left;
    public static List<DcMotorEx> motors = Arrays.asList(module1Left, module1Right);
    public static List<String> motorNames = Arrays.asList("module1Left", "module1Right");
    public static List<DcMotorEx> reversedMotors = Arrays.asList(module1Left);

    public static DcMotorEx module1Encoder, module2Encoder;
    public static List<DcMotorEx> moduleEncoders = Arrays.asList(module1Encoder);
    public static List<String> moduleEncoderNames = Arrays.asList("module1Encoder");

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    public static double rotationTest = 0;

    public static double pulsesPerRevEncoder = 22.5;

    public static final int numberOfModules = 1;
    public static final double gearRatioLeftMotor2Wheel = 1;
    public static final double gearRatioRightMotor2Wheel = 1;
    public static final double gearRatio2Encoder = 2;

    public static double powerCutOff = 0.05;

    public static double[][] PIDvals = new double[][] {
            {0.005},
            {0.005}
    };

    public static final double[][] modulePositions = new double[][] { // x,y (unit: inches) (left module is first) (positive is right and up)
            {-4,0},
            {0,0}
    };
}
