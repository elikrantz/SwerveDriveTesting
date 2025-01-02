package org.firstinspires.ftc.teamcode.tragectorymath.util;

public class NanoTimer {
    private long startTime;

    /**
     * This creates a new NanoTimer with the start time set to its creation time.
     */
    public NanoTimer() {
        resetTimer();
    }

    /**
     * This resets the NanoTimer's start time to the current time using System.nanoTime().
     */
    public void resetTimer() {
        startTime = System.nanoTime();
    }

    /**
     * This returns the elapsed time in nanoseconds since the start time of the NanoTimer.
     *
     * @return this returns the elapsed time in nanoseconds.
     */
    public long getElapsedTime() {
        return System.nanoTime() - startTime;
    }

    /**
     * This returns the elapsed time in seconds since the start time of the NanoTimer.
     *
     * @return this returns the elapsed time in seconds.
     */
    public double getElapsedTimeSeconds() {
        return (getElapsedTime() / Math.pow(10.0,9));
    }
}