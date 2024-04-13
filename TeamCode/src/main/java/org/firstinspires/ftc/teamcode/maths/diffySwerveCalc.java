package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class diffySwerveCalc {

    public static double[] convert2Diffy(double wheelVel, double moduleAngle, Telemetry telemetry) {
    //public static double[] convert2Diffy(double wheelVel, double moduleAngle) {
        double maxTurnVal = 30;
        telemetry.addData("moduleAngleBegin",moduleAngle);
        double angleDiv = MathEx.max(Math.abs(moduleAngle/maxTurnVal),1);
        moduleAngle = (moduleAngle/maxTurnVal)/angleDiv;
        telemetry.addData("moduleAngle",moduleAngle);
        telemetry.addData("wheelVel", wheelVel);
        /*double angleDiv = MathEx.max(Math.abs(moduleAngle),1);
        moduleAngle = (moduleAngle)/angleDiv;*/
        /*double motorRight = moduleAngle + RobotConstants.motorRightDir * wheelVel;
        double motorLeft = moduleAngle + RobotConstants.motorLeftDir * wheelVel;*/
        double motorRight = moduleAngle + wheelVel;
        double motorLeft = moduleAngle - wheelVel;
        //telemetry.addData("motorRight", motorRight);
        //telemetry.addData("motorLeft", motorLeft);
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
