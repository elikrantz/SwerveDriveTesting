package org.firstinspires.ftc.teamcode.maths;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class swerveCalc {
    public double[][] calculate(double velRobotX,double velRobotY,double velRobotRot, double imu, boolean... extraOptions) {
        //first extraOptions is normalize, second extraOptions is fieldOriented
        boolean normalize = (extraOptions.length >= 1) ? extraOptions[0] : true;
        boolean fieldOperated = (extraOptions.length >= 2) ? extraOptions[1] : false;
        /**
         * @param output[0][]: this is the velocity of the module's wheel
         * @param output[1][]: this is the angle of the module's wheel
         * @param output[][n]: "n" is the number of the module you want the information from
         */
        ArrayList<Double> moduleVelocities = new ArrayList<>();
        ArrayList<Double> moduleAngles = new ArrayList<>();

        double maxVel = 1;
        
        double strafe = velRobotX;
        double forward = velRobotY;

        double botHeading = Math.toRadians(imu);
        if (fieldOperated) {
            forward = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            strafe = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        }

        for (int i = 0; i < RobotConstants.numberOfModules; i++) {
            double velModuleX;
            double velModuleY;
            double velWheel;
            double moduleAngle;

            velModuleX = (1 * strafe) + (0 * forward) + (-RobotConstants.modulePositions[i][1] * velRobotRot);
            velModuleY = (0 * strafe) + (1 * forward) + (RobotConstants.modulePositions[i][0] * velRobotRot);

            velWheel = Math.sqrt(Math.pow(velModuleX,2) + Math.pow(velModuleY,2));
            moduleAngle = Math.atan2(velModuleX,velModuleY) * 180 / Math.PI;

            if (normalize && Math.abs(velWheel) >= Math.abs(maxVel)) maxVel = Math.abs(velWheel);

            moduleVelocities.add(i,velWheel);
            moduleAngles.add(i,moduleAngle);
        }

        if (normalize) {
            for (int i = 0; i < moduleVelocities.size(); i++) {
                moduleVelocities.set(i, (moduleVelocities.get(i) / Math.abs(maxVel)));
            }
        }

        double[][] output = new double[2][moduleVelocities.size()]; // this makes a 2D array size of 2 by (length/size of the moduleVelocities list)
        for (int i = 0; i < moduleVelocities.size(); i++) output[0][i] = moduleVelocities.get(i);
        for (int i = 0; i < moduleAngles.size(); i++) output[1][i] = moduleAngles.get(i);

        return output;
    }
}
