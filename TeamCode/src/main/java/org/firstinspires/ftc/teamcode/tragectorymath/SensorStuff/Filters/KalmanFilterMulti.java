package org.firstinspires.ftc.teamcode.tragectorymath.SensorStuff.Filters;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KalmanFilterMulti {
    private KalmanFilterMultiParameters parameters;
    private double state;
    private double variance;
    private double kalmanGain;
    private double previousState;
    private double previousVariance;

    /**
     * need as many data variance as data sources = different R
     * need 1 proccess noise variance = Q
     * 
     */

    public KalmanFilterMulti(KalmanFilterMultiParameters parameters) {
        this.parameters = parameters;
        //reset();
        initKalman(0,1);
    }

    public KalmanFilterMulti(KalmanFilterMultiParameters parameters, double startState, double startVariance) {
        this.parameters = parameters;
        //reset(startState, startVariance, startGain);
        initKalman(startState,startVariance);
    }
    
    public void initKalman(double startState, double startVariance) {
        this.state = startState;
        //this.previousState = startState;
        this.variance = startVariance;
        //this.previousVariance = startVariance;
    }

    public void predict() {
        variance += parameters.modelCovariance;
    }

    public void update(double measurement, double dataCovariance) {
        double kalmanGain = variance / (variance + dataCovariance);
        state += kalmanGain * (measurement - state);
        variance *= (1.0 - kalmanGain);
    }

    public void runLoop(double[] measurements) {
        predict();
        for (int i=0; i < parameters.dataCovariance.length; i++) {
            update(measurements[i], parameters.dataCovariance[i]);
        }
    }

    public double getState() {
        return state;
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("state", state);
        telemetry.addData("variance", variance);
        telemetry.addData("Kalman gain", kalmanGain);
    }
}