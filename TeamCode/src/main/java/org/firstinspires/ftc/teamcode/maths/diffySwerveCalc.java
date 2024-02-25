package org.firstinspires.ftc.teamcode.maths;

public class diffySwerveCalc {

    public static double[] convert2Diffy(double wheelVel, double moduleAngle) {
        double motor1 = moduleAngle + wheelVel;
        double motor2 = moduleAngle - wheelVel;
        double maxVal = MathEx.max(Math.abs(motor1), Math.abs(motor2), 1);
        if (maxVal > 1) {
            motor1 /= Math.abs(maxVal);
            motor2 /= Math.abs(maxVal);
        }
        return new double[] {motor1,motor2};
    }

    public static double[] optimizedTurning(double target, double current, double power) {
        double error = target - current;

        while (error > 90) {
            power *= -1;
            target -= 180;
            error = target - current;
        }
        while (error < -90) {
            power *= -1;
            target += 180;
            error = target - current;
        }

        return new double[] {target,power};
    }
}
