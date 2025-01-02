package org.firstinspires.ftc.teamcode.tragectorymath.SensorStuff;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tragectorymath.maths.*;

public class EncoderEx {
    HardwareMap hardwareMap;
    DcMotorEx encoder;
    public enum Direction {
        FORWARD,
        REVERSE
    };
    // DcMotorEx[] encoders = new DcMotorEx[RobotConstants.moduleEncoders.size()];
    // encoders = RobotConstants.moduleEncoders.toArray(encoders);

    private double currentVal;
    private double previousVal;
    private Direction direction;

    public EncoderEx(HardwareMap hardwareMap, String encoderName) {
        //this.encoder = encoder;
        this.hardwareMap = hardwareMap;
        //this.encoder = encoder;
        this.encoder = hardwareMap.get(DcMotorEx.class, encoderName);
    }

    public void setDirection(Direction direction) {
        switch (direction) {
            case FORWARD:
                encoder.setDirection(DcMotorEx.Direction.FORWARD);
                this.direction = Direction.FORWARD;
                break;
            case REVERSE:
                encoder.setDirection(DcMotorEx.Direction.REVERSE);
                this.direction = Direction.REVERSE;
                break;
        }
        //encoder.setDirection(DcMotorEx.direction);
    }

    // public static void InitializeEncoder(HardwareMap hardwareMap) {
    //     DcMotorEx[] encoders = new DcMotorEx[RobotConstants.moduleEncoders.size()];
    //     encoders = RobotConstants.moduleEncoders.toArray(encoders);
    //     // for (int motorNum = 0; motorNum < encoders.length; motorNum++) {
    //     //     encoders[motorNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.moduleEncoderNames.get(motorNum));
    //     //     //if (RobotConstants.reversedMotors.contains(encoders[motorNum])) encoders[motorNum].setDirection(DcMotorSimple.Direction.REVERSE);
    //     // }
    //     //encoders[encoderNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.moduleEncoderNames.get(encoderNum));
    //     for (int encoderNum = 0; encoderNum < encoders.length; encoderNum++) {
    //         encoders[encoderNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.moduleEncoderNames.get(encoderNum));
    //     }
    // }
    public Direction getDirection() {
        return direction;
    }

    public double getDirectionVal() {
        return (getDirection() == Direction.FORWARD) ? 1 : -1;
    }

    public double getEncoderDegrees() {
        double degrees = Maths.EncoderTicks2Degrees(encoder.getCurrentPosition());
        return degrees;
    }

    public void update() {
        previousVal = currentVal;
        currentVal = encoder.getCurrentPosition();
    }

    public double getDeltaPosition() {
        //previousVal = currentVal;
        //currentVal = encoder.getCurrentPosition();
        return getDirectionVal() * (currentVal - previousVal);
    }

    public void reset() {
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}