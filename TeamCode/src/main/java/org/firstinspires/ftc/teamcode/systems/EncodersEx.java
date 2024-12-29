package org.firstinspires.ftc.teamcode.systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maths.MathEx;

public class EncodersEx {
    HardwareMap hardwareMap;
    DcMotorEx encoder;
    Servo encoderServo;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private double positionAngle = 0;
    private double zeroOffset;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    private Direction direction;

    public EncodersEx(HardwareMap hardwareMap, DcMotorEx encoder, String encoderName) {
        if (!RobotConstants.usingServoForEncoders) {
            this.encoder = encoder;
            this.hardwareMap = hardwareMap;
            this.zeroOffset = 0;
            this.direction = Direction.FORWARD;
            this.encoder = hardwareMap.get(DcMotorEx.class, encoderName);
        }
    }
    public EncodersEx(HardwareMap hardwareMap, Servo encoderServo, String encoderName) {
        if (RobotConstants.usingServoForEncoders) {
            this.encoderServo = encoderServo;
            this.hardwareMap = hardwareMap;
            this.zeroOffset = 0;
            this.direction = Direction.FORWARD;
            //this.encoderServo = hardwareMap.get(Servo.class, encoderName);
        }
    }
    public EncodersEx(HardwareMap hardwareMap, DcMotorEx encoder, String encoderName, double zeroOffset) {
        if (!RobotConstants.usingServoForEncoders) {
            this.encoder = encoder;
            this.hardwareMap = hardwareMap;
            this.zeroOffset = zeroOffset;
            this.direction = Direction.FORWARD;
            this.encoder = hardwareMap.get(DcMotorEx.class, encoderName);
        }
    }
    public EncodersEx(HardwareMap hardwareMap, Servo encoderServo, String encoderName, double zeroOffset) {
        if (RobotConstants.usingServoForEncoders) {
            this.encoderServo = encoderServo;
            this.hardwareMap = hardwareMap;
            this.zeroOffset = zeroOffset;
            this.direction = Direction.FORWARD;
            //this.encoderServo = hardwareMap.get(Servo.class, encoderName);
        }
    }

//    public static void InitializeEncoders(HardwareMap hardwareMap) {
//        DcMotorEx[] encoders = new DcMotorEx[RobotConstants.moduleEncoders.size()];
//        encoders = RobotConstants.moduleEncoders.toArray(encoders);
//        for (int motorNum = 0; motorNum < encoders.length; motorNum++) {
//            encoders[motorNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.moduleEncoderNames.get(motorNum));
//            //if (RobotConstants.reversedMotors.contains(encoders[motorNum])) encoders[motorNum].setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public double getEncoderDegrees() {
        TelemetryPacket packet = new TelemetryPacket();
        if (RobotConstants.usingServoForEncoders) {
            double input = encoderServo.getPosition() - zeroOffset;
            positionAngle = MathEx.scale(input, new double[]{0,1}, new double[]{0,180});
        } else {
            double degrees = MathEx.EncoderTicks2Degrees(encoder.getCurrentPosition());
            positionAngle = degrees - zeroOffset;
        }

        if (direction == Direction.REVERSE) {
            positionAngle = 360 - positionAngle;
        }
        positionAngle = MathEx.modulusDegrees(positionAngle);
        packet.put("positionAngle",positionAngle);
        packet.put("encoderVal",encoderServo.getPosition());
        dashboard.sendTelemetryPacket(packet);
        return positionAngle;
    }
}
