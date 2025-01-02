package org.firstinspires.ftc.teamcode.tragectorymath.Examples;

import org.firstinspires.ftc.teamcode.tragectorymath.RobotConstants;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tragectorymath.follower.*;
import org.firstinspires.ftc.teamcode.tragectorymath.util.*;
import org.firstinspires.ftc.teamcode.tragectorymath.maths.*;

import java.util.List;


@Autonomous
public class ExampleAuto extends LinearOpMode {
    
    private double headingTarget = 0;
    private int taskNum = 0;

    public void runOpMode() {
        //

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        DriveController driveController = new DriveController(hardwareMap, telemetry, new Pose2d(0,0,0), PathList.EXAMPLE_CUBIC_PATH_DOUBLE);

        waitForStart();

        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            driveController.update();

            if (taskNum == 0) {
                telemetry.addData("taskNum: ", taskNum);
                headingTarget = -120;
                taskNum++;
            }
            if (taskNum == 1 && driveController.isPathDone(5,7)) {
                telemetry.addData("taskNum: ", taskNum);
                headingTarget = 0;
                //driveController.setPath(PathList.EXAMPLE_CUBIC_PATH_REVERSED);
                driveController.setPath(PathList.EXAMPLE_CUBIC_PATH_REVERSED, 0.7, 10, 0.75);
                taskNum++;
            }
            if (taskNum == 2 && driveController.isPathDone(4,7)) {
                telemetry.addData("taskNum: ", taskNum);
                telemetry.addData("completed task","");
                taskNum++;
            }
            telemetry.update();
            if (taskNum == 3) {
                break;
            }
        }
    }
}