package org.firstinspires.ftc.teamcode.tragectorymath.follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;
import org.firstinspires.ftc.teamcode.tragectorymath.RobotConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tragectorymath.SensorStuff.Pose2dUpdater;
import org.firstinspires.ftc.teamcode.tragectorymath.maths.*;
import org.firstinspires.ftc.teamcode.tragectorymath.util.*;

public class DriveController {
    HardwareMap hardwareMap;
    Pose2dUpdater pose2dUpdater;
    Follower follower;
    DriveType drive;
    Telemetry telemetry;

    private Pose2d startPose2d;
    private Pose2d currentPose2d;
    private Vector2d robotVectors;
    private double headingTarget;
    private double robotAngleTarget;

    private double Kn = 0.7, Kf = 15, Ks = 0.7;

    public DriveController(HardwareMap hardwareMap, Telemetry telemetry, DriveType drive, Pose2d startPose2d, CubicPath startPath) {
        this.hardwareMap = hardwareMap;
        this.drive = drive;
        this.startPose2d = startPose2d;
        this.telemetry = telemetry;
        //drive.initialize();
        pose2dUpdater = new Pose2dUpdater(hardwareMap,telemetry);
        pose2dUpdater.setStartingPose2d(startPose2d);
        follower = new Follower(startPath, Kn, Kf, Ks, telemetry);
    }

    public DriveController(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startPose2d, CubicPath startPath) {
        this(hardwareMap, telemetry, new SwerveDrive(telemetry,hardwareMap,true,false,false), startPose2d, startPath);
    }

    public void update() {
        //Pose2d pose = swerve.getPose();
        pose2dUpdater.update();
        currentPose2d = pose2dUpdater.getPose2d();
        robotVectors = follower.output(new Vector2d(currentPose2d.getX(), currentPose2d.getY()));
        robotAngleTarget = follower.headingOut(headingTarget,pose2dUpdater.getTotalHeading(), false, false);

        drive.drive(robotVectors.getX(), robotVectors.getY(), robotAngleTarget, true);
    }

    public void setPath(CubicPath newPath, double Kn, double Kf, double Ks) {
        follower.setPath(newPath, Kn, Kf, Ks);
    }
    public void setPath(CubicPath newPath) {
        this.setPath(newPath, Kn, Kf, Ks);
    }

    public boolean isPathDone(double positionTolerance, double headingTolerance) { return follower.isDone(positionTolerance, headingTolerance); }

}