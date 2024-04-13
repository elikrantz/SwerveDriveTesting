package org.firstinspires.ftc.teamcode.maths;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class diffySwerveCalc {

    public static double[] convert2Diffy(double wheelVel, double moduleAngle) {
        double angleDiv = MathEx.max(Math.abs(moduleAngle/90),1);
        moduleAngle = (moduleAngle/90)/angleDiv;
        /*double motorRight = moduleAngle + RobotConstants.motorRightDir * wheelVel;
        double motorLeft = moduleAngle + RobotConstants.motorLeftDir * wheelVel;*/
        double motorRight = moduleAngle + wheelVel;
        double motorLeft = moduleAngle - wheelVel;
        double maxVal = MathEx.max(Math.abs(motorRight), Math.abs(motorLeft), 1);
        if (maxVal > 1) {
            motorRight /= Math.abs(maxVal);
            motorLeft /= Math.abs(maxVal);
        }
        return new double[] {motorRight,motorLeft};
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
