package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;


public class RobotConstants {

    public static DcMotorEx module1Left, module1Right, module2Right, module2Left;
    public static List<DcMotorEx> motors = Arrays.asList(module1Left, module1Right, module2Right, module2Left);

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    public static double pulsesPerRevEncoder = 1;

    public static final int numberOfModules = 2;
    public static final double gearRatio2LeftMotor = 1;
    public static final double gearRatio2RightMotor = 1;
    public static final double gearRatio2Encoder = 1;

    public static final double[][] modulePositions = new double[][] { // x,y (left module is first)
            {0,0},
            {0,0}
    };

    /*public RobotConstants() {
        motors = Arrays.asList(module1Left, module1Right, module2Right, module2Left);
    }*/
}
