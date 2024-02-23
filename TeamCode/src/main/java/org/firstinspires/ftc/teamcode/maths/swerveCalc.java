package org.firstinspires.ftc.teamcode.maths;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.Arrays;

public class swerveCalc {
    public double[] calculate(int moduleNum,double velRobotX,double velRobotY,double velRobotRot) {
        double velModuleX;
        double velModuleY;

        double velWheel;
        double moduleAngle;

        velModuleX = (1 * velRobotX) + (0 * velRobotY) + (-RobotConstants.modulePositions[moduleNum][1] * velRobotRot);
        velModuleY = (0 * velRobotX) + (1 * velRobotY) + (-RobotConstants.modulePositions[moduleNum][0] * velRobotRot);

        velWheel = Math.sqrt(Math.pow(velModuleX,2) + Math.pow(velModuleY,2));
        moduleAngle = Math.atan2(velModuleY,velModuleX);

        double[] output = new double[] {velWheel, moduleAngle};
        return output;
    }
}
