package org.firstinspires.ftc.teamcode.tragectorymath.follower;

import org.firstinspires.ftc.teamcode.tragectorymath.maths.*;

public abstract class DriveType {
    
    /**
     * This is the method used to power the motors / drive the robot
     *
     * @param x the X component vector for the robots velocity
     * @param y the R component vector for the robots velocity
     * @param rot the angle you want the robot to be at
     * 
     * @param fieldOriented a bool for if field oriented drive (needs to be set to true for auto)
     */
    public abstract void drive(double x, double y, double rot, boolean fieldOriented);
}