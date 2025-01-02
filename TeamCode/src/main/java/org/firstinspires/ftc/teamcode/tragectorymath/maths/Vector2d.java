package org.firstinspires.ftc.teamcode.tragectorymath.maths;

public class Vector2d {

    // IMPORTANT NOTE: angle is defined in radians.
    // These are the values of the coordinate defined by this Point, in both polar and
    // Cartesian systems.
    private double x;
    private double y;
    private double magnitude;
    private double angle;

    /**
     * This creates a new zero Vector (zero magnitude and direction).
     */
    public Vector2d() {
        setComponents(0,0);
    }

    /**
     * This creates a new Vector with specified components.
     *
     * @param x x component of the Vector.
     * @param y y compenent of the Vector.
     */
    public Vector2d(double x, double y) {
        setComponents(x,y);
    }

    /**
     * This takes in an r and angle value and converts them to Cartesian coordinates.
     *
     * @param r this is the r value of the Point being converted.
     * @param angle this is the angle value of the Point being converted.
     * @return this returns the x and y values, in that order, in an Array of doubles.
     */
    private static double[] polarToCartesian(double r, double angle) {
        return new double[]{r * Math.cos(angle), r * Math.sin(angle)};
    }

    /**
     * This takes in an x and y value and converts them to polar coordinates.
     *
     * @param x this is the x components of the Vector being converted.
     * @param y this is the y components of the Vector being converted.
     * @return this returns the r and angle values, in that order, in an Array of doubles.
     */
    private static double[] cartesianToPolar(double x, double y) {
        double r = Math.sqrt(x * x + y * y);
        return new double[]{r, Math.atan2(y,x)};
    }

    /**
     * This sets the orthogonal components of the Vector. These orthogonal components are assumed
     * to be in the direction of the x-axis and y-axis. In other words, this is setting the
     * coordinates of the Vector using x and y coordinates instead of a direction and magnitude.
     *
     * @param x sets the x component of this Vector.
     * @param y sets the y component of this Vector.
     */
    public void setComponents(double x, double y) {
        double[] polarComponents;
        this.x = x;
        this.y = y;
        polarComponents = cartesianToPolar(x, y);
        magnitude = polarComponents[0];
        angle = polarComponents[1];
    }
    
    /**
     * This sets the components of the Vector in regular vector coordinates.
     *
     * @param magnitude sets the magnitude of this Vector.
     * @param angle sets the angle value of this Vector.
     */
    public void setPolarComponents(double magnitude, double angle) {
        double[] orthogonalComponents;
        if (magnitude<0) {
            this.magnitude = -magnitude;
            this.angle = Maths.normalizeAngle(angle+Math.PI);
        } else {
            this.magnitude = magnitude;
            this.angle = Maths.normalizeAngle(angle);
        }
        orthogonalComponents = polarToCartesian(magnitude, angle);
        x = orthogonalComponents[0];
        y = orthogonalComponents[1];
    }

    /**
     * This sets only the magnitude of the Vector.
     *
     * @param magnitude sets the magnitude of this Vector.
     */
    public void setMagnitude(double magnitude) {
        setPolarComponents(magnitude, angle);
    }

    /**
     * This sets only the angle, angle, of the Vector.
     *
     * @param angle sets the angle, or angle value, of this Vector.
     */
    public void setAngle(double angle) {
        setPolarComponents(magnitude, angle);
    }

    /**
     * This rotates the Vector by an angle, angle.
     *
     * @param angle2 the angle to be added.
     */
    public void rotateVectorBy(double angle2) {
        setAngle(angle+angle2);
    }

    /**
     * This divides the Vector by a scalar value, val
     * 
     * @param val the value of the divisor
     * @return returns the result of the division
     */
    public Vector2d divide(double val) {
        return new Vector2d(x/val,y/val);
    }

    /**
     * This multiplies the Vector by a scalar value, val
     * 
     * @param val the value the vector is getting scaled/multiplied
     * @return returns the result of the multiplication
     */
    public Vector2d times(double val) {
        return new Vector2d(x*val,y*val);
    }

    /**
     * This subtracts the other Vector from the Vector
     * 
     * @param otherVector the vector being subtracted
     * @return returns the result of the substraction
     */
    public Vector2d minus(Vector2d otherVector) {
        return new Vector2d(x-otherVector.getX(),y-otherVector.getY());
    }

    /**
     * This adds the other Vector to the Vector
     * 
     * @param otherVector the vector being added
     * @return returns the result of the addition
     */
    public Vector2d add(Vector2d otherVector) {
        return new Vector2d(x+otherVector.getX(),y+otherVector.getY());
    }

    /**
     * This take the dot product of the Vector and other Vector
     * 
     * @param otherVector the vector being dotted into
     * @return returns the result of the dot product
     */
    public double dot(Vector2d otherVector) {
        return x*otherVector.getX() + y*otherVector.getY();
    }

    /**
     * This take the cross product of the Vector and other Vector
     * 
     * @param otherVector the vector being crossed into (Vector X otherVector)
     * @return returns the result of the cross product vector
     */
    public double cross(Vector2d otherVector) {
        //double determinant1 = y * 0 - 0 * otherVector.getY();
        //double determinant2 = x * 0 - 0 * otherVector.getX();
        double determinant3 = x * otherVector.getY() - y * otherVector.getX();
        return determinant3;
    }

    public double distTo(Vector2d b) {
        return Maths.magnitudeOf(this.minus(b));
    }

    /**
     * Returns the magnitude of this Vector.
     *
     * @return returns the magnitude.
     */
    public double getMagnitude() {
        return magnitude;
    }

    /**
     * Returns the angle value, or angle, of this Vector.
     *
     * @return returns the angle value.
     */
    public double getAngle() {
        return angle;
    }

    /**
     * Returns the x component of this Vector.
     *
     * @return returns the x component.
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y component of this Vector.
     *
     * @return returns the y component.
     */
    public double getY() {
        return y;
    }

    public Vector2d copy() {
        return new Vector2d(this.magnitude, this.angle);
    }
}