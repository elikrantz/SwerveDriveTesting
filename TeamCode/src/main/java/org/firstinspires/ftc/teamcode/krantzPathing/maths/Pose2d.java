package org.firstinspires.ftc.teamcode.krantzPathing.maths;
//package tragectorymath.maths;

import androidx.annotation.NonNull;

public class Pose2d {
    
    private double x;
    private double y;
    private double heading;

    /**
     * This creates a new Pose2d from a x, y, and heading inputs.
     *
     * @param setX the initial x value
     * @param setY the initial y value
     * @param setHeading the initial heading value
     */
    public Pose2d(double setX, double setY, double setHeading) {
        setX(setX);
        setY(setY);
        setHeading(setHeading);
    }

    /**
     * This creates a new Pose2d from x and y inputs. The heading is set to 0.
     *
     * @param setX the initial x value
     * @param setY the initial y value
     */
    public Pose2d(double setX, double setY) {
        this(setX, setY, 0);
    }

    /**
     * This creates a new Pose2d with no inputs and 0 for all values.
     */
    public Pose2d() {
        this(0, 0, 0);
    }

    /**
     * This sets the x value.
     *
     * @param set the x value
     */
    public void setX(double set) {
        x = set;
    }

    /**
     * This sets the y value.
     *
     * @param set the y value
     */
    public void setY(double set) {
        y = set;
    }

    /**
     * This sets the heading value.
     *
     * @param set the heading value
     */
    public void setHeading(double set) {
        heading = Maths.normalizeAngle(set);
    }

    /**
     * This returns the x value.
     *
     * @return returns the x value
     */
    public double getX() {
        return x;
    }

    /**
     * This returns the y value.
     *
     * @return returns the y value
     */
    public double getY() {
        return y;
    }

    /**
     * This returns the heading value.
     *
     * @return returns the heading value
     */
    public double getHeading() {
        return heading;
    }

    /**
     * This returns the Pose2d as a Vector2d. Naturally, the heading data in the Pose2d cannot be included
     * in the Vector2d.
     *
     * @return returns the Pose2d as a Vector2d
     */
    public Vector2d getVector() {
        Vector2d returnVector = new Vector2d();
        returnVector.setComponents(x, y);
        return returnVector;
    }

    /**
     * This returns a new Vector2d with magnitude 1 pointing in the direction of the heading.
     *
     * @return returns a unit Vector2d in the direction of the heading
     */
    public Vector2d getHeadingVector() {
        return new Vector2d(1, heading);
    }

    /**
     * This adds all the values of an input Pose2d to this Pose2d. The input Pose2d's data will not be
     * changed.
     *
     * @param pose the input Pose2d
     */
    public Pose2d add(Pose2d pose) {
        setX(x + pose.getX());
        setY(y + pose.getY());
        setHeading(heading + pose.getHeading());
        return this;
    }

    /**
     * This subtracts all the values of an input Pose2d from this Pose2d. The input Pose2d's data will not
     * be changed.
     *
     * @param pose the input Pose2d
     */
    public Pose2d subtract(Pose2d pose) {
        setX(x - pose.getX());
        setY(y - pose.getY());
        setHeading(heading - pose.getHeading());
        return this;
    }

    /**
     * This multiplies all the values of this Pose2d by a scalar.
     *
     * @param scalar the scalar
     */
    public Pose2d scalarMultiply(double scalar) {
        setX(x * scalar);
        setY(y * scalar);
        setHeading(heading * scalar);
        return this;
    }

    /**
     * This divides all the values of this Pose2d by a scalar.
     *
     * @param scalar the scalar
     */
    public Pose2d scalarDivide(double scalar) {
        setX(x / scalar);
        setY(y / scalar);
        setHeading(heading / scalar);
        return this;
    }

    /**
     * This flips the signs of all values in this Pose2d by multiplying them by -1. Heading values are
     * still normalized to be between 0 and 2 * pi in value.
     */
    public Pose2d flipSigns() {
        setX(-x);
        setY(-y);
        setHeading(-heading);
        return this;
    }

    /**
     * This returns if a Pose2d is within a specified accuracy of this Pose2d in terms of x position,
     * y position, and heading.
     *
     * @param pose the input Pose2d to check
     * @param accuracy the specified accuracy necessary to return true
     * @return returns if the input Pose2d is within the specified accuracy of this Pose2d
     */
    public boolean roughlyEquals(Pose2d pose, double accuracy) {
        return Maths.roughlyEquals(x, pose.getX(), accuracy) && Maths.roughlyEquals(y, pose.getY(), accuracy) && Maths.roughlyEquals(Maths.getSmallestAngleDifference(heading, pose.getHeading()), 0, accuracy);
    }

    /**
     * This checks if the input Pose2d is within 0.0001 in all values to this Pose2d.
     *
     * @param pose the input Pose2d
     * @return returns if the input Pose2d is within 0.0001 of this Pose2d
     */
    public boolean roughlyEquals(Pose2d pose) {
        return roughlyEquals(pose, 0.0001);
    }

    /**
     * This creates a copy of this Pose2d that points to a new memory location.
     *
     * @return returns a deep copy of this Pose2d
     */
    public Pose2d copy() {
        return new Pose2d(getX(), getY(), getHeading());
    }

    @NonNull
    @Override
    public String toString() {
        return "(" + getX() + ", " + getY() + ", " + Math.toDegrees(getHeading()) + ")";
    }
}