package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class SwerveDrive {
    private final Telemetry telemetry;
    private final IMU imu;

    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap) {
        for (DcMotorEx motor: RobotConstants.motors) {
            motor = hardwareMap.get(DcMotorEx.class, motor.toString());
        }
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotConstants.LOGO_FACING_DIR, RobotConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        this.telemetry = telemetry;
    }

    public void drive (double x, double y, double rx) {
        //
    }
}
