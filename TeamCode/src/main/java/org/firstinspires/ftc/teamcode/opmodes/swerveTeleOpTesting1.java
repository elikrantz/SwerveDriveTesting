package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

@Config
@TeleOp
public class swerveTeleOpTesting1 extends LinearOpMode {
    SwerveDrive swerveDrive;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double yVal = 0;
    public static double xVal = 0;
    public static double rxVal = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
//            double rx = gamepad1.right_stick_x;

            double y = -yVal;
            double x = xVal;
            double rx = rxVal;

            //y = MathEx.clip(y,0.05);
            //x = MathEx.clip(x,0.05);
            //rx = MathEx.clip(rx,0.05);

            swerveDrive.drive(x,y,rx,false);

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private void Initialization() {
        swerveDrive = new SwerveDrive(telemetry,hardwareMap,true, false, true);
    }
}
