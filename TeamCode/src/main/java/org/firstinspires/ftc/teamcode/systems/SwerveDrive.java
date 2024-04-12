package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.diffySwerveCalc;
import org.firstinspires.ftc.teamcode.maths.swerveCalc;

import java.util.Arrays;
import java.util.List;

public class SwerveDrive {
    private final Telemetry telemetry;
    private final IMU imu;
    private final swerveCalc swerveMath = new swerveCalc();
    private final boolean optimumTurn;
    private final boolean testing;

    private double[] modulesTargetRot = new double[RobotConstants.numberOfModules];
    //private double[] modulesPower = new double[RobotConstants.numberOfModules];

    DcMotorEx[] motors = new DcMotorEx[RobotConstants.motors.size()];
    DcMotorEx[] encoders = new DcMotorEx[RobotConstants.moduleEncoders.size()];
    private final PIDcontroller[] modulesPID = new PIDcontroller[RobotConstants.PIDvals.length];
    //private final PIDcontroller[] modulesPID = new PIDcontroller[RobotConstants.numberOfModules];

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap) {
        for (int moduleNum = 0; moduleNum < modulesPID.length; moduleNum++) {
            double[] vals = new double[4];
            Arrays.fill(vals, 0);
            System.arraycopy(RobotConstants.PIDvals[moduleNum], 0, vals, 0, RobotConstants.PIDvals[moduleNum].length);
            modulesPID[moduleNum] = new PIDcontroller(vals[0],vals[1],vals[2],vals[3]);
        }

        motors = RobotConstants.motors.toArray(motors);
        for (int motorNum = 0; motorNum < motors.length; motorNum++) {
            motors[motorNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.motorNames.get(motorNum));
            if (RobotConstants.reversedMotors.contains(motors[motorNum])) motors[motorNum].setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //EncodersEx.InitializeEncoders(hardwareMap);
        /*EncodersEx[] encoders = new EncodersEx[RobotConstants.moduleEncoders.size()];
        encoders = RobotConstants.moduleEncoders.toArray(encoders);
        for (int motorNum = 0; motorNum < encoders.length; motorNum++) {
            encoders[motorNum] = new EncodersEx(hardwareMap.get(DcMotorEx.class, RobotConstants.moduleEncoderNames.get(motorNum)));
        }*/
        //DcMotorEx[] encoders = new DcMotorEx[RobotConstants.moduleEncoders.size()];
        encoders = RobotConstants.moduleEncoders.toArray(encoders);
        for (int encoderNum = 0; encoderNum < encoders.length; encoderNum++) {
            encoders[encoderNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.moduleEncoderNames.get(encoderNum));
        }

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotConstants.LOGO_FACING_DIR, RobotConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        this.telemetry = telemetry;
        this.optimumTurn = true;
        this.testing = false;
    }
    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap, boolean optimumTurn, boolean testing) {
        for (int moduleNum = 0; moduleNum < modulesPID.length; moduleNum++) {
            double[] vals = new double[4];
            Arrays.fill(vals, 0);
            System.arraycopy(RobotConstants.PIDvals[moduleNum], 0, vals, 0, RobotConstants.PIDvals[moduleNum].length);
            modulesPID[moduleNum] = new PIDcontroller(vals[0],vals[1],vals[2],vals[3]);
        }

        motors = RobotConstants.motors.toArray(motors);
        for (int motorNum = 0; motorNum < motors.length; motorNum++) {
            motors[motorNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.motorNames.get(motorNum));
            if (RobotConstants.reversedMotors.contains(motors[motorNum])) motors[motorNum].setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //EncodersEx.InitializeEncoders(hardwareMap);
        /*EncodersEx[] encoders = new EncodersEx[RobotConstants.moduleEncoders.size()];
        encoders = RobotConstants.moduleEncoders.toArray(encoders);
        for (int motorNum = 0; motorNum < encoders.length; motorNum++) {
            encoders[motorNum] = new EncodersEx(hardwareMap.get(DcMotorEx.class, RobotConstants.moduleEncoderNames.get(motorNum)));
        }*/
        //DcMotorEx[] encoders = new DcMotorEx[RobotConstants.moduleEncoders.size()];
        encoders = RobotConstants.moduleEncoders.toArray(encoders);
        for (int encoderNum = 0; encoderNum < encoders.length; encoderNum++) {
            encoders[encoderNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.moduleEncoderNames.get(encoderNum));
        }

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotConstants.LOGO_FACING_DIR, RobotConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        this.telemetry = telemetry;
        this.optimumTurn = optimumTurn;
        this.testing = testing;
    }

    public void drive (double x, double y, double rx) {
        TelemetryPacket packet = new TelemetryPacket();

        double heading = AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        double[][] output = swerveMath.calculate(x,y,rx,heading);
        //packet.put("output", output);

        for (int i = 0; i < RobotConstants.numberOfModules; i++) {
            telemetry.addData("Power", output[0][i]);
            telemetry.addData("Rotation", output[1][i]);
            //double moduleRotEncoder = RobotConstants.moduleEncoders.get(i).getEncoderDegrees();
            double moduleRotEncoder = MathEx.EncoderTicks2Degrees(encoders[i].getCurrentPosition());
            telemetry.addData("degrees", moduleRotEncoder);
            //double moduleRotEncoder = RobotConstants.rotationTest;

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
            telemetry.addData("modTargetRot"+i, modulesTargetRot[i]);
            telemetry.addData("modPower"+i, modulePower);

            double[] motorVals = diffySwerveCalc.convert2Diffy(modulePower,((modulesTargetRot[i] - moduleRotEncoder) / 360));
            //double[] motorVals = diffySwerveCalc.convert2Diffy(modulePower,modulesPID[i].controller(AngleUnit.normalizeDegrees(modulesTargetRot[i] - moduleRotEncoder)));
            motorVals[0] = MathEx.clip(motorVals[0],RobotConstants.powerCutOff);
            motorVals[1] = MathEx.clip(motorVals[1],RobotConstants.powerCutOff);
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
                                if (!testing) {
                                    motors[motorNum].setPower(motorVals[0]);
                                }
                                telemetry.addData(c+"Right",motorVals[0]);
                            }
                            if (RobotConstants.motorNames.get(motorNum).toLowerCase().contains("left")) {
                                if (!testing) {
                                    motors[motorNum].setPower(motorVals[1]);
                                }
                                telemetry.addData(c+"Left",motorVals[0]);
                            }
                        }
                    }
                }
            }
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
