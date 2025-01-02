package org.firstinspires.ftc.teamcode.krantzPathing.maths;
//package tragectorymath.maths;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.krantzPathing.RobotConstants;

public class Maths {

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians;
        while (angle < 0) angle += 2 * Math.PI;
        while (angle > 2 * Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    public static Vector2d interpolateBetweenVectors(Vector2d start, Vector2d end, double interpolator) {
        double m = (start.getY() - end.getY()) / (start.getX() - end.getX());
        double b = -(m * start.getX()) + start.getY();

        return new Vector2d((interpolator - b) / m, interpolator);
    }

    static class MissingAPointComponent extends Exception {
        public MissingAPointComponent() {
            super();
        }

        public MissingAPointComponent(String message) {
            super(message);
        }
    }

    public static Vector2d[] pointListToVectorList(double[] coordinateList) { // there should be 16 components making 8 vectors
        if (coordinateList.length != 15) {
            //throw new MissingAPointComponent("(probably missing a component) there needs to be 16 point components (x,y) to make the 8 vectors");
            throw new IndexOutOfBoundsException("(probably missing a component) there needs to be 16 point components (x,y) to make the 8 vectors");
        }
        Vector2d[] vectorList = new Vector2d[coordinateList.length];
        for (int i = 0; i < coordinateList.length; i += 2) {
            vectorList[i] = new Vector2d(coordinateList[i], coordinateList[i + 1]);
        }
        return vectorList;
    }

    public static double tanhErrorMap(double x) {
        double a = 1;
        double b = 1.1;
        return a * Math.tanh(b * x);
    }

    public static double modulusDegrees(double degrees) {
        return degrees % 360;
    }

    public static double EncoderTicks2Degrees(int ticks) {
        //return modulusDegrees(RobotConstants.gearRatio2Encoder * ticks / RobotConstants.pulsesPerRevEncoder);
        return RobotConstants.gearRatio2Encoder * ticks / RobotConstants.pulsesPerRevEncoder;
    }

    public static double getSmallestAngleDifference(double one, double two) {
        return Math.min(Maths.normalizeAngle(one - two), Maths.normalizeAngle(two - one));
    }

    public static double getTurnDirection(double startHeading, double endHeading) {
        if (Maths.normalizeAngle(endHeading - startHeading) >= 0 && Maths.normalizeAngle(endHeading - startHeading) <= Math.PI) {
            return 1; // counter clock wise
        }
        return -1; // clock wise
    }

    public static boolean epsilonEquals(double state, double equals, double thresh) {
        return Math.abs(state - equals) < thresh;
    }

    public static boolean epsilonEquals(double state, double equals) {
        return Math.abs(state - equals) < 1e-6;
    }

    public static double angleOf(Vector2d vec) {
        return AngleUnit.normalizeRadians(Math.atan2(vec.getY(), vec.getX()));
    }

    public static double magnitudeOf(Vector2d vec) {
        return Math.sqrt(Math.pow(vec.getX(), 2) + Math.pow(vec.getY(), 2));
    }

    public static double distanceBetween(Vector2d a, Vector2d b) {
        return magnitudeOf(a.minus(b));
    }

    public static boolean roughlyEquals(double one, double two, double accuracy) {
        return (one < two + accuracy && one > two - accuracy);
    }

    public static boolean roughlyEquals(double one, double two) {
        return roughlyEquals(one, two, 0.0001);
    }
}