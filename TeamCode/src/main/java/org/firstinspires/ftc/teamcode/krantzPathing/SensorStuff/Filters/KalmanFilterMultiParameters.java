package org.firstinspires.ftc.teamcode.krantzPathing.SensorStuff.Filters;

import kotlin.jvm.JvmField;

public class KalmanFilterMultiParameters {
    @JvmField public double modelCovariance;
    @JvmField public double[] dataCovariance;

    /**
     * This creates a new KalmanFilterParameters with a specified model and data covariance.
     *
     * @param modelCovariance the covariance of the model.
     * @param dataCovariance the covariance of the data.
     */
    public KalmanFilterMultiParameters(double modelCovariance, double... dataCovariance) {
        this.modelCovariance = modelCovariance;
        if (dataCovariance.length < 1) { throw new ArrayIndexOutOfBoundsException("need at least 1 data source"); }
        for (int i=0; i < dataCovariance.length; i++) {
            this.dataCovariance[i] = dataCovariance[i];
        }
        //this.dataCovariance = dataCovariance;
    }

    //KalmanFilterMultiParameters test = new KalmanFilterMultiParameters(0,0,0);
}