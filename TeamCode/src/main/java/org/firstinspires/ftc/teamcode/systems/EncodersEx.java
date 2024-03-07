package org.firstinspires.ftc.teamcode.systems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maths.MathEx;

public class EncodersEx {
    //HardwareMap hardwareMap;
    DcMotorEx encoder;

    public EncodersEx(DcMotorEx encoder) {
        this.encoder = encoder;
    }

    public static void InitializeEncoders(HardwareMap hardwareMap) {
        DcMotorEx[] encoders = new DcMotorEx[RobotConstants.moduleEncoders.size()];
        encoders = RobotConstants.moduleEncoders.toArray(encoders);
        for (int motorNum = 0; motorNum < encoders.length; motorNum++) {
            encoders[motorNum] = hardwareMap.get(DcMotorEx.class, RobotConstants.moduleEncoderNames.get(motorNum));
            //if (RobotConstants.reversedMotors.contains(encoders[motorNum])) encoders[motorNum].setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public int getEncoderDegrees() {
        int degrees = (int)MathEx.EncoderTicks2Degrees(encoder.getCurrentPosition());
        return degrees;
    }
}
