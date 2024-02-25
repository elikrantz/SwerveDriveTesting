package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDcontroller {
    // Parameters you define when creating the object, DONT'T TOUCH "dT" UNLESS YOU KNOW WHAT YOU ARE DOING
    public double kP, kI, kD, kF;
    public final double kPS, kIS, kDS, kFS;
    /** For fine tuning P, I, and D, read link that is below **/
    // Eli needs to add link, so yell at him
    //private final long defaultDT = 20;
    private final long dT = 20; //(Units: milliSeconds) time between cycles
    private final ElapsedTime timer = new ElapsedTime();

    // Predefined constants, also don't touch
    public double maxIntegralVal = 50000000;
    public final double maxIntegralFinal = maxIntegralVal;
    private double maxErrorReset = 10000;
    private double minErrorReset = 100;

    // Changing variables
    private double error = 0;
    private double prevError = 0;
    private double integral = 0;
    private double derivative = 0;
    private double power = 0;

    public PIDcontroller(double kP, double kI, double kD, double kF) {
        //targetPoint = targetVal;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        //this.dT = dT;
        kPS = kP;
        kIS = kI;
        kDS = kD;
        kFS = kF;
    }
    public PIDcontroller(double kP) {
        this.kP = kP;
        this.kI = 0;
        this.kD = 0;
        this.kF = 0;
        kPS = kP;
        kIS = 0;
        kDS = 0;
        kFS = 0;
    }
    public PIDcontroller(double kP, double kD) {
        this.kP = kP;
        this.kI = 0;
        this.kD = kD;
        this.kF = 0;
        kPS = kP;
        kIS = 0;
        kDS = kD;
        kFS = 0;
    }
    public PIDcontroller(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
        kPS = kP;
        kIS = kI;
        kDS = kD;
        kFS = 0;
    }

    public void setLimits(double maxIntegral) {
        this.maxIntegralVal = maxIntegral;
    }


    public double controller(double error, double powerMax) {
        timer.reset();
        integral += error * timer.seconds();
        integral = Range.clip(integral,-maxIntegralFinal,maxIntegralFinal);
        derivative = (error - prevError) / timer.seconds();
        prevError = error;
        power = error*kP + integral*kI + derivative*kD + Math.signum(error)*kF;
        power = Range.clip(power,-powerMax,powerMax);
        timer.reset();
        return power;
    }
}
