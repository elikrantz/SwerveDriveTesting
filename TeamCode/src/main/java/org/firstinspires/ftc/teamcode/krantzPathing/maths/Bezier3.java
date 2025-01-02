package org.firstinspires.ftc.teamcode.krantzPathing.maths;

public class Bezier3 {
    private Vector2d P0,P1,P2,P3;
    private double totalArcLength;
    private final int accuracy = 50; // Number of points the find along curve, the higher the number the more points and the more accurate
    private Vector2d[] points = new Vector2d[accuracy + 1];

    /**
     * A bezier curve must pass through all 4 control points. When created, total arc length of the curve as well as a points table of distance -> T
     */
    public Bezier3(Vector2d P0, Vector2d P1, Vector2d P2, Vector2d P3) {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;

        generatePoints();
    }

    public void setControlPoints(Vector2d P0, Vector2d P1, Vector2d P2, Vector2d P3) {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;

        generatePoints();
    }

    public Vector2d getP0() { return this.P0; }
    public Vector2d getP1() { return this.P1; }
    public Vector2d getP2() { return this.P2; }
    public Vector2d getP3() { return this.P3; }

    public void generatePoints() {
        double arc = 0;
        for (int i = 0; i < accuracy; i++) {
            points[i] = new Vector2d(i / (double) accuracy, arc);
            arc += Maths.distanceBetween(getPoint(i / (double) accuracy), getPoint((i + 1) / (double) accuracy));
        }
        //by the end, the arc variable has computed the arc length of the entire curve.
        totalArcLength = arc;
        points[accuracy] = new Vector2d(1, totalArcLength);
    }

    public double getTotalArcLength() {
        return totalArcLength;
    }

    /**
     * Finds the arc length from 0 to T along the curve using parametrization.
     */
    public double getArcLength(double T) {
        double total = 0;
        for (int i = 0; i < accuracy; i++) {
            if (i / (double) accuracy < T) {
                total += Maths.distanceBetween(getPoint(i / (double) accuracy), getPoint((i + 1) / (double) accuracy));
            }
            else break;
        }
        return total;
    }

    /**
     * Returns a point in cartesian coordinates along the curve
     * @param T value which desired point is at
     * @return (x,y) point on the curve at specified T value
     */
    public Vector2d getPoint(double T) {
        double weight1 = (1 - Math.pow((T),3) + 3 * Math.pow((T),2) - 3 * T);
        double weight2 = (3  * Math.pow((T),3) - 6 * Math.pow((T),2) + 3 * T);
        double weight3 = (-3 * Math.pow((T),3) + 3 * Math.pow((T),2));
        double weight4 = Math.pow((T),3);

        return P0.times(weight1).add(P1.times(weight2)).add(P2.times(weight3)).add(P3.times(weight4));
    }

    /**
     * Returns a point in cartesian coordinates along this bezier curve's 1st derivative, which is a bezier curve with 3 control points.
     * @param T value which desired point is at
     * @return (x,y) point on the curve's first derivative at specified T value
     */
    public Vector2d firstDerivative(double T) {
        double weight1 = (-3 * Math.pow((T),2) + 6  * T - 3);
        double weight2 = (9 *  Math.pow((T),2) - 12 * T + 3);
        double weight3 = (-9 * Math.pow((T),2) + 6  * T);
        double weight4 = (3 *  Math.pow((T),2));

        return P0.times(weight1).add(P1.times(weight2)).add(P2.times(weight3)).add(P3.times(weight4));
    }

    /**
     * Returns the normalized tangent vector to the curve at specified point
     * @param T value at which to calculate tangent vector
     * @return tangent to the curve at specified point
     */
    public Vector2d getNormalizedTangent(double T) {
        Vector2d firstDerivative = firstDerivative(T);
        return firstDerivative.divide(firstDerivative.getMagnitude());
    }

    /**
     * Returns the normalized normal vector to the curve at specified point
     * @param T value at which to calculate tangent vector
     * @return normal to the curve at specified point
     */
    public Vector2d getNormalizedNormal(double T) {
        Vector2d normal = getNormalizedTangent(T);
        normal.rotateVectorBy(Math.PI / 2);
        return normal;
    }

    /**
     * Uses a pre generated points table to estimate what the T value is for a specific arc length.
     * @param distance arc length along the curve at which we wish to find the T value
     * @return the estimated T value at which this arc length is located
     */
    public double distanceToT(double distance) {
        //throws error if point is not found
        int index = -(int)distance;
        for (int i = 0; i < accuracy; i++) {
            if (points[i].getY() <= distance && points[i + 1].getY() >= distance) {
                index = i;
                break;
            }
        }
        return Maths.interpolateBetweenVectors(points[index], points[index + 1], distance).getX();
    }
}