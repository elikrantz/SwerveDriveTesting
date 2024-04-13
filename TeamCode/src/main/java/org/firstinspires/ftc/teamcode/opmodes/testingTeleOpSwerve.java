package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.maths.MathEx;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

@TeleOp
public class testingTeleOpSwerve extends LinearOpMode {
    SwerveDrive swerveDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            //y = MathEx.clip(y,0.05);
            //x = MathEx.clip(x,0.05);
            //rx = MathEx.clip(rx,0.05);

            swerveDrive.drive(x,y,rx);
        }
    }

    private void Initialization() {
        swerveDrive = new SwerveDrive(telemetry,hardwareMap,true, false);
    }
}
