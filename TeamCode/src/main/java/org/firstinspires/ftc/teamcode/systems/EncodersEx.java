package org.firstinspires.ftc.teamcode.systems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maths.MathEx;

public class EncodersEx {

    public EncodersEx(HardwareMap hardwareMap) {
        for (DcMotorEx encoder : RobotConstants.moduleEncoders) {
            encoder = hardwareMap.get(DcMotorEx.class, encoder.toString());
        }
    }

    public static double getEncoderDegrees(int indexNum) {
        double degrees = MathEx.EncoderTicks2Degrees(RobotConstants.moduleEncoders.get(indexNum).getCurrentPosition());
        return degrees;
    }
}
