package org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.localization;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.krantzPathing.RobotConstants;
import org.firstinspires.ftc.teamcode.krantzPathing.maths.Vector2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;

import org.firstinspires.ftc.teamcode.krantzPathing.maths.Pose2d;
import org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.Localizer;

@Config
public class AprilTagLocalization extends Localizer {
    //public LinearOpMode opmode = null;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public HardwareMap hardwareMap;
    private Telemetry telemetry;

    public static double FORWARD_TICKS_TO_INCHES = 1;
    public static double STRAFE_TICKS_TO_INCHES = 1;
    public static double TURN_TICKS_TO_RADIANS = 1;


    public enum aprilTags {
        RedAllianceLeft,
        RedAllianceCenter,
        RedAllianceRight,
        BlueAllianceLeft,
        BlueAllianceCenter,
        BlueAllianceRight,
        RedAudienceWallSmall,
        RedAudienceWallLarge,
        BlueAudienceWallSmall,
        BlueAudienceWallLarge
    }
    Dictionary<aprilTags, Integer> aprilTagsDict = new Hashtable<>();
    Dictionary<aprilTags, double[]> aprilTagsPosDict = new Hashtable<>();
    public final static double[] fieldSize = new double[] {144,144};
    public final static double[] redAprilTagSmallPos = new double[] {34.5,144.5,4};
    //public final static double[] redAprilTagBigPos = new double[] {-5.5,0,1.5};
    public final static double[] redAprilTagBigPos = new double[] {29,144.5,5.5};
    public final static double[] blueAprilTagSmallPos = new double[] {109.5,144.5,4};
    //public final static double[] blueAprilTagBigPos = new double[] {5.5,0,1.5};
    public final static double[] blueAprilTagBigPos = new double[] {115,144.5,5.5};
    //public final static double[] cameraOffset = new double[] {3.5,5.5}; // x offset (left: positive, right: negative), y(distance) offset; (units: inches from center)
    //double[] robotDistanceToAprilTag = new double[] {0,0};
    //double[] robotFieldPos = new double[] {0,0};
    Pose2d robotFieldPos = new Pose2d();
    double[][] robotDistancesToAprilTags = new double[][] {{0,0},{0,0},{0,0},{0,0}}; // [0][n]: RedAudienceWallLarge, [1][n]: BlueAudienceWallLarge, [2][n]: RedAudienceWallSmall, [3][n]: BlueAudienceWallSmall
    double[][] calculationAprilTagsDistances = new double[][] {{0,0},{0,0},{0,0},{0,0}}; // [0][n]: RedAudienceWallLarge, [1][n]: BlueAudienceWallLarge, [2][n]: RedAudienceWallSmall, [3][n]: BlueAudienceWallSmall
    // goes Yaw, Pitch, Roll
    double[][] robotAnglesFromAprilTags = new double[][] {{0,0,0},{0,0,0},{0,0,0},{0,0,0}}; // [0][n]: RedAudienceWallLarge, [1][n]: BlueAudienceWallLarge, [2][n]: RedAudienceWallSmall, [3][n]: BlueAudienceWallSmall

    public AprilTagLocalization(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap,telemetry,new Pose2d());
    }

    public AprilTagLocalization(HardwareMap hardwareMap, Telemetry telemetry, Pose2d setStartPose2d) {
        this.hardwareMap = hardwareMap;

        setStartPose2d(setStartPose2d);

        initCam();
    }

    @Override
    public void setStartPose2d(Pose2d setStart) {
        robotFieldPos = setStart;
    }

    @Override
    public Pose2d getVelocity2d() {
        return null;
    }

    @Override
    public Vector2d getVelocity2dVector() {
        return null;
    }

    @Override
    public void setPose2d(Pose2d setPose2d) {
        robotFieldPos = setPose2d;
    }

    @Override
    public double getTotalHeading() {
        return robotFieldPos.getHeading();
    }

    public double getForwardMultiplier() {
        return FORWARD_TICKS_TO_INCHES;
    }

    public double getLateralMultiplier() {
        return STRAFE_TICKS_TO_INCHES;
    }

    public double getTurningMultiplier() {
        return TURN_TICKS_TO_RADIANS;
    }

    public void resetIMU() {
    }

    public void resetEncoders() {}

    @Override
    //public void runOpMode(){
    public void update() {
        //initCam();
        // aprilTagsDict.put(aprilTags.RedAudienceWallLarge,0);
        // aprilTagsDict.put(aprilTags.BlueAudienceWallLarge,1);
        // aprilTagsDict.put(aprilTags.RedAudienceWallSmall,2);
        // aprilTagsDict.put(aprilTags.BlueAudienceWallSmall,3);
        // aprilTagsPosDict.put(aprilTags.RedAudienceWallLarge,redAprilTagBigPos);
        // aprilTagsPosDict.put(aprilTags.BlueAudienceWallLarge,blueAprilTagBigPos);
        // aprilTagsPosDict.put(aprilTags.RedAudienceWallSmall,redAprilTagSmallPos);
        // aprilTagsPosDict.put(aprilTags.BlueAudienceWallSmall,blueAprilTagSmallPos);
        // while (!isStarted()) {
        //     List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        //     telemetry.addData("%f",currentDetections.size());
        //     telemetry.update();
        //     //wait(1);
        // }
        //waitForStart();

        // if (opModeIsActive()) {
        //     while (opModeIsActive()) {

        //         //telemetryAprilTag();
        //         aprilTags[] aprilTagsLooking = new aprilTags[] {aprilTags.BlueAudienceWallLarge,aprilTags.RedAudienceWallLarge};
        //         getRobotPosAprilTag(aprilTagsLooking);

        //         // Push telemetry to the Driver Station.
        //         telemetry.update();

        //         // Save CPU resources; can resume streaming when needed.
//                 if (gamepad1.dpad_down) {
//                     visionPortal.stopStreaming();
//                 } else if (gamepad1.dpad_up) {
//                     visionPortal.resumeStreaming();
//                 }

        //         // Share the CPU.
        //         sleep(20);
        //     }
        // }
        //telemetryAprilTag();
        aprilTags[] aprilTagsLooking = new aprilTags[] {aprilTags.BlueAudienceWallLarge,aprilTags.RedAudienceWallLarge};
        getRobotPosAprilTag(aprilTagsLooking);



        // Push telemetry to the Driver Station.
        //telemetry.update();

        // Save CPU resources; can resume streaming when needed.
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }

        // Share the CPU.
        //sleep(20);
    }

    @Override
    public Pose2d getPose2d() {
        return robotFieldPos.copy();
    }

    public void initCam(){
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(RobotConstants.cameraPos, RobotConstants.cameraRotation)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        aprilTagsDict.put(aprilTags.RedAudienceWallLarge,0);
        aprilTagsDict.put(aprilTags.BlueAudienceWallLarge,1);
        aprilTagsDict.put(aprilTags.RedAudienceWallSmall,2);
        aprilTagsDict.put(aprilTags.BlueAudienceWallSmall,3);
        aprilTagsPosDict.put(aprilTags.RedAudienceWallLarge,redAprilTagBigPos);
        aprilTagsPosDict.put(aprilTags.BlueAudienceWallLarge,blueAprilTagBigPos);
        aprilTagsPosDict.put(aprilTags.RedAudienceWallSmall,redAprilTagSmallPos);
        aprilTagsPosDict.put(aprilTags.BlueAudienceWallSmall,blueAprilTagSmallPos);
    }
    public void getRobotPosAprilTag(aprilTags[] tagNames) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<aprilTags> foundAprilTags = new ArrayList<aprilTags>();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                for (aprilTags tagName : tagNames) {
                    if (detection.metadata.name.equals(tagName.toString())) {
                        int arrayNum = aprilTagsDict.get(tagName);
                        //robotDistancesToAprilTags[arrayNum][0] = detection.ftcPose.x + cameraOffset[0];
                        robotDistancesToAprilTags[arrayNum][0] = detection.robotPose.getPosition().x;
                        //robotDistancesToAprilTags[arrayNum][1] = detection.ftcPose.y + cameraOffset[1];
                        robotDistancesToAprilTags[arrayNum][1] = detection.robotPose.getPosition().y;
                        robotAnglesFromAprilTags[arrayNum][0] = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                        //robotAnglesFromAprilTags[arrayNum][1] = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                        //robotAnglesFromAprilTags[arrayNum][2] = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                        foundAprilTags.add(tagName);
                    }
                }
            } else { //add to move on to next step or align to a position
                break;
            }
        }
        //telemetry.addLine(String.format("XY %6.1f %6.1f  (inch)",robotDistancesToAprilTags[0],robotDistancesToAprilTags[1]));
        //int foundTagsLength = (int)foundAprilTags.size();
        if (foundAprilTags.size() > 0) {
            for (aprilTags tagName : foundAprilTags) {
                int arrayNum1 = aprilTagsDict.get(tagName);
                calculationAprilTagsDistances[arrayNum1][0] = aprilTagsPosDict.get(tagName)[0] - robotDistancesToAprilTags[arrayNum1][0];
                calculationAprilTagsDistances[arrayNum1][1] = aprilTagsPosDict.get(tagName)[1] - robotDistancesToAprilTags[arrayNum1][1];
            }
            double xPosSum = 0;
            double yPosSum = 0;
            double robotYawSum = 0;
            for (aprilTags tagName : foundAprilTags) {
                int arrayNum1 = aprilTagsDict.get(tagName);
                xPosSum += calculationAprilTagsDistances[arrayNum1][0];
                yPosSum += calculationAprilTagsDistances[arrayNum1][1];
                robotYawSum += robotAnglesFromAprilTags[arrayNum1][0]; // Not sure if this outputs the right thing more testing needed
            }
            //robotFieldPos[0] = xPosSum / foundAprilTags.size();
            robotFieldPos.setX(xPosSum / foundAprilTags.size());
            //robotFieldPos[1] = yPosSum / foundAprilTags.size();
            robotFieldPos.setY(yPosSum / foundAprilTags.size());
            robotFieldPos.setHeading(robotYawSum / foundAprilTags.size());
            //telemetry.addLine(String.format("XY %6.1f %6.1f  (inch)",robotFieldPos[0],robotFieldPos[1]));
            telemetry.addLine(String.format("XY %6.1f %6.1f  (inch)",robotFieldPos.getX(),robotFieldPos.getY()));
        }
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}