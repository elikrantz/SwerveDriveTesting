package org.firstinspires.ftc.teamcode.krantzPathing.util;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.krantzPathing.maths.*;

public class PID {
    //PID controller class

    private ConstantsForPID constants;
    //private double integralSum, out, previousOut, previousError, lastReference;
    private double error, previousError;
    //private double position, targetPosition, previousPosition;
    private double out, previousOut;
    private double integralSum;
    private double lastReference;
    private long previousUpdateTimeNano;
    private long deltaTimeNano;
    //private final ElapsedTime timer = new ElapsedTime();

    public PID(ConstantsForPID constants) {
        this.constants = constants;
    }

    public PID(double Kp, double Kd, double Ki, double Kf, double Kl, double pointTunedAt) {
        constants = new ConstantsForPID(Kp, Kd, Ki, Kf, Kl, pointTunedAt);
    }

    public PID(double Kp, double Kd, double Ki, double Kf, double Kl) {
        constants = new ConstantsForPID(Kp, Kd, Ki, Kf, Kl, 0);
    }

    //calculate
    public double pidOut(double reference, double state) {
        //position = state;
        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        if (!Maths.epsilonEquals(lastReference, reference, 0.01)) integralSum = 0;

        double error = reference - state;

        //integral and derivative values
        double derivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
        integralSum += error * (deltaTimeNano / Math.pow(10.0, 9));
        integralSum = Range.clip(integralSum, -constants.Kl(), constants.Kl());
        //weight each term so that tuning makes a difference
        out = (constants.Kp() * error) + (constants.Kd() * derivative) + (constants.Ki() * integralSum) + (constants.Kf() * Math.signum(error));
        out /= 10;
        previousError = error;
        lastReference = reference;
        //timer.reset();

        return out;
    }

    public double pidAngleOut(double reference, double state) {
        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        if (!Maths.epsilonEquals(lastReference, reference, 0.01)) integralSum = 0;

        double error = AngleUnit.normalizeDegrees(reference - state);
        //integral and derivative values
        double derivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
        integralSum += error * (deltaTimeNano / Math.pow(10.0, 9));
        integralSum = Range.clip(integralSum, -constants.Kl(), constants.Kl());
        //weight each term so that tuning makes a difference
        out = (constants.Kp() * error) + (constants.Kd() * derivative) + (constants.Ki() * integralSum) + (constants.Kf() * Math.signum(error));
        out /= 10;

        previousError = error;
        previousOut = out;
        lastReference = reference;
        //timer.reset();

        return out;
    }

    public double pidPivotOut(double reference, double state) {
        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        if (!Maths.epsilonEquals(lastReference, reference, 0.01)) integralSum = 0;

        double error = AngleUnit.normalizeDegrees(reference - state);
        //integral and derivative values
        double derivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
        integralSum += error * (deltaTimeNano / Math.pow(10.0, 9));
        integralSum = Range.clip(integralSum, -constants.Kl(), constants.Kl());
        //weight each term so that tuning makes a difference
        out = (constants.Kp() * error) + (constants.Kd() * derivative) + (constants.Ki() * integralSum) + (constants.Kf() * Math.signum(error) * Math.sin(Math.toRadians(state)));
        out /= 10;

        previousError = error;
        previousOut = out;
        lastReference = reference;
        //timer.reset();

        return out;
    }

    public double[] inDepthOutput(double reference, double state) {
        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        double error = AngleUnit.normalizeDegrees(reference - state);
        //integral and derivative values
        double derivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));

        return new double[] {out, constants.Kp() * error, constants.Kd() * derivative, constants.Ki() * integralSum, constants.Kf() * Math.signum(error), (deltaTimeNano / Math.pow(10.0, 9)), (out - previousOut), error};

    }
}