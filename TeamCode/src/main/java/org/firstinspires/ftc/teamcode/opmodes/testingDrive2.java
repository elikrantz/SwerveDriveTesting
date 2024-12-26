package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants;

@Config
@TeleOp
public class testingDrive2 extends LinearOpMode {
    DcMotorEx[] motors = new DcMotorEx[RobotConstants.motors.size()];
    public static DcMotorEx motorTest;
    public static String motorTestingName = "Motor0";
    public static double power = 0.0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        motorTest = hardwareMap.get(DcMotorEx.class, motorTestingName);
        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("testing","testing1");

            motorTest.setPower(power);

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
