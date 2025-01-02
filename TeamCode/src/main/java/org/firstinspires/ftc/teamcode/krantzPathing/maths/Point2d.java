package org.firstinspires.ftc.teamcode.krantzPathing.maths;

public class Point2d {
    private double x;
    private double y;
    private double r;
    private double angle;

    public Point2d() {
        this.x = 0;
        this.y = 0;
    }

    public Point2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double distanceFrom(Point2d otherPoint) {
        return Math.sqrt(Math.pow(otherPoint.getX() - x, 2) + Math.pow(otherPoint.getY() - y, 2));
    }
}