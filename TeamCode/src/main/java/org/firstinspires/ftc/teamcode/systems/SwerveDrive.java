package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maths.MathEx;
import org.firstinspires.ftc.teamcode.maths.diffySwerveCalc;
import org.firstinspires.ftc.teamcode.maths.swerveCalc;

public class SwerveDrive {
    private final Telemetry telemetry;
    private final IMU imu;
    private final swerveCalc swerveMath = new swerveCalc();
    private final boolean optimumTurn;
    private EncodersEx EncodersExs;

    private double[] modulesTargetRot = new double[RobotConstants.numberOfModules];
    //private double[] modulesPower = new double[RobotConstants.numberOfModules];

    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap) {
        /*for (DcMotorEx motor : RobotConstants.motors) {
            motor = hardwareMap.get(DcMotorEx.class, motor.toString());
            if (RobotConstants.reversedMotors.contains(motor)) motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }*/
        DcMotorEx[] motors = new DcMotorEx[RobotConstants.motors.size()];
        motors = RobotConstants.motors.toArray(motors);
        for (int motorNum = 0; motorNum < motors.length; motorNum++) {
            motors[motorNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.motorNames.get(motorNum));
            if (RobotConstants.reversedMotors.contains(motors[motorNum])) motors[motorNum].setDirection(DcMotorSimple.Direction.REVERSE);
        }

        EncodersEx.InitializeEncoders(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotConstants.LOGO_FACING_DIR, RobotConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        this.telemetry = telemetry;
        this.optimumTurn = true;
    }
    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap, boolean optimumTurn) {
        for (DcMotorEx motor : RobotConstants.motors) {
            motor = hardwareMap.get(DcMotorEx.class, motor.toString());
            if (RobotConstants.reversedMotors.contains(motor)) motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        EncodersEx.InitializeEncoders(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotConstants.LOGO_FACING_DIR, RobotConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        this.telemetry = telemetry;
        this.optimumTurn = optimumTurn;
    }

    public void drive (double x, double y, double rx) {

        double heading = AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        double[][] output = swerveMath.calculate(x,y,rx,heading);

        for (int i = 0; i < RobotConstants.numberOfModules; i++) {
            double moduleRotEncoder = EncodersEx.getEncoderDegrees(i);

            double modulePower = output[0][i];

            if (y != 0 || x != 0 || rx != 0) {
                modulesTargetRot[i] = output[1][i];
            }

            moduleRotEncoder = AngleUnit.normalizeDegrees(moduleRotEncoder);
            modulesTargetRot[i] = AngleUnit.normalizeDegrees(modulesTargetRot[i]);

            double[] optimizedModuleVals = diffySwerveCalc.optimizedTurning(modulesTargetRot[i],moduleRotEncoder,modulePower);

            if (optimumTurn) {
                modulesTargetRot[i] = optimizedModuleVals[0];
                modulePower = optimizedModuleVals[1];
            }

            double[] motorVals = diffySwerveCalc.convert2Diffy(modulePower,AngleUnit.normalizeDegrees(modulesTargetRot[i] - moduleRotEncoder));
            /*for (DcMotorEx motor: RobotConstants.motors) {
                char[] chars = motor.toString().toCharArray();
                for (char c : chars) {
                    if (Character.isDigit(c)) {
                        if (Character.getNumericValue(c) == (i+1)) {
                            if (motor.toString().toLowerCase().contains("right")) {
                                motor.setPower(motorVals[0]);
                            }
                            if (motor.toString().toLowerCase().contains("left")) {
                                motor.setPower(motorVals[1]);
                            }
                        }
                    }
                }
            }*/
            for (int motorNum = 0; motorNum < RobotConstants.motors.size(); motorNum++) {
                char[] chars = RobotConstants.motorNames.get(motorNum).toCharArray();
                for (char c : chars) {
                    if (Character.isDigit(c)) {
                        if (Character.getNumericValue(c) == (i+1)) {
                            if (RobotConstants.motorNames.get(motorNum).toLowerCase().contains("right")) {
                                RobotConstants.motors.get(motorNum).setPower(motorVals[0]);
                            }
                            if (RobotConstants.motorNames.get(motorNum).toLowerCase().contains("left")) {
                                RobotConstants.motors.get(motorNum).setPower(motorVals[1]);
                            }
                        }
                    }
                }
            }
        }
    }
}
