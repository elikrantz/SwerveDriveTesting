package org.firstinspires.ftc.teamcode.maths;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

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
        double output = degrees % 360;
        if (output < 0) { output = 360 + output; }
        return output;
    }

    public static double EncoderTicks2Degrees(int ticks) {
        return modulusDegrees(RobotConstants.gearRatio2Encoder * ticks / RobotConstants.pulsesPerRevEncoder);
        //return RobotConstants.gearRatio2Encoder * ticks / RobotConstants.pulsesPerRevEncoder;
    }

    static class double2 {
        double[] range = new double[2];
        double min, max;
        double2(double[] range) {
            if (range.length != 2) { throw new IndexOutOfBoundsException("Array must be length 2"); }
            this.range = range;
            this.min = range[0];
            this.max = range[1];
        }
    }
    private static double scaleMath(double input, double2 inputRange, double2 outputRange) {
        double inputPercent = (input - inputRange.min) / (inputRange.max - inputRange.min);
        double output = inputPercent * (outputRange.max + outputRange.min);
        return output;
    }
    public static double scale(double input, double[] inputRange, double[] outputRange) {
        return scaleMath(input, new double2(inputRange), new double2(outputRange));
    }

    public static double getServoPositionDegrees(Servo encoderServo, double zeroOffset) {
        double input = encoderServo.getPosition() - zeroOffset;
        double positionAngle = MathEx.scale(input, new double[]{0,1}, new double[]{0,355}); //Remember to change output double to {0,360} when servos are fixed
        return positionAngle;
    }

    public static boolean between(double input, double min, double max) {
        return (input < max && input > min) ? (true) : (false);
    }

//    public static double range(double val ,double min, double max) {
//        if (val)
//    }
}
