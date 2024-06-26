package org.firstinspires.ftc.teamcode.maths;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class MathEx {

    public static int max(@NonNull int... values) {
        int maxVal = 0;
        for (int i = 0; i < values.length; i++) {
            if (values[i] > maxVal) maxVal = values[i];
        }
        return maxVal;
    }
    public static long max(@NonNull long... values) {
        long maxVal = 0;
        for (int i = 0; i < values.length; i++) {
            if (values[i] > maxVal) maxVal = values[i];
        }
        return maxVal;
    }
    public static float max(@NonNull float... values) {
        float maxVal = 0;
        for (int i = 0; i < values.length; i++) {
            if (values[i] > maxVal) maxVal = values[i];
        }
        return maxVal;
    }
    public static double max(@NonNull double... values) {
        double maxVal = 0;
        for (int i = 0; i < values.length; i++) {
            if (values[i] > maxVal) maxVal = values[i];
        }
        return maxVal;
    }

    public static double clip(double value, double min) {
        //value = (Math.abs(value) <= min) ? 0 : value;
        if (Math.abs(value) <= Math.abs(min)) {
            value = 0;
        }
        return value;
    }

    public static double[] clipMulti(double min, @NonNull double... values) {
        double[] output = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            output[i] = (Math.abs(values[i]) <= min) ? 0 : values[i];
        }
        return output;
    }

    public static double modulusDegrees(double degrees) {
        return degrees % 360;
    }

    public static double EncoderTicks2Degrees(int ticks) {
        return modulusDegrees(RobotConstants.gearRatio2Encoder * ticks / RobotConstants.pulsesPerRevEncoder);
        //return RobotConstants.gearRatio2Encoder * ticks / RobotConstants.pulsesPerRevEncoder;
    }
}
