package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotConstants;

@TeleOp
public class motorTesting extends LinearOpMode {
    DcMotorEx[] motors = new DcMotorEx[RobotConstants.motors.size()];
    private int moduleNum = 1;
    public void runOpMode() throws InterruptedException {
        motors = RobotConstants.motors.toArray(motors);
        for (int motorNum = 0; motorNum < motors.length; motorNum++) {
            motors[motorNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.motorNames.get(motorNum));
            if (RobotConstants.reversedMotors.contains(motors[motorNum])) motors[motorNum].setDirection(DcMotorSimple.Direction.REVERSE);
        }
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.share && (moduleNum + 1) <= RobotConstants.numberOfModules) {
                moduleNum += 1;
            }
            for (int motorNum = 0; motorNum < RobotConstants.motors.size(); motorNum++) {
                char[] chars = RobotConstants.motorNames.get(motorNum).toCharArray();
                for (char c : chars) {
                    if (Character.isDigit(c)) {
                        if (Character.getNumericValue(c) == (moduleNum)) {
                            if (RobotConstants.motorNames.get(motorNum).toLowerCase().contains("right")) {
                                double powerValR = gamepad1.right_stick_x;
                                motors[motorNum].setPower(powerValR);
                                telemetry.addData(c + "Right", powerValR);
                            }
                            if (RobotConstants.motorNames.get(motorNum).toLowerCase().contains("left")) {
                                double powerValL = -gamepad1.left_stick_x;
                                motors[motorNum].setPower(powerValL);
                                telemetry.addData(c + "Left", powerValL);
                            }
                        }
                    }
                }
            }
        }
    }
}
