package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class testingEncoderReadingAxon extends LinearOpMode {
    public static Servo servoTest;
    public static String servoTestingName = "Servo1";
    public static double position = 0.0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        servoTest = hardwareMap.get(Servo.class, servoTestingName);
        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("testing","testing1");

            double encoderVal = servoTest.getPosition();
            packet.put("encoderVal",encoderVal);
            servoTest.setPosition(position);

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
