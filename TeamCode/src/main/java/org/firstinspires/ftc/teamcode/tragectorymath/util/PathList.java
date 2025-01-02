package org.firstinspires.ftc.teamcode.tragectorymath.util;

import org.firstinspires.ftc.teamcode.tragectorymath.maths.*;

import java.util.Arrays;
import java.util.Collections;

public class PathList {

    public static CubicPath reversePath(CubicPath path) {
        Vector2d[] pointList = path.getControlPointList();
        Collections.reverse(Arrays.asList(pointList));
        return new CubicPath(pointList);
    }

    public static CubicPath toRed(CubicPath path) {
        Vector2d[] pointList = path.getControlPointList();
        int i = 0;
        for (Vector2d vec : pointList) {
            pointList[i] = new Vector2d(-vec.getX(), -vec.getY());
            i++;
        }
        return new CubicPath(pointList);
    }

    public static final CubicPath EXAMPLE_CUBIC_PATH_DOUBLE = new CubicPath(new double[] { // length of double[] need to be a length of 16, since this gets turned into 8 points
        20.3,-50, 30,-40, 25.4,-30, 40,-10, 30,10, 50,30, 40,30, 10,-20
    });

    public static final CubicPath EXAMPLE_CUBIC_PATH_VECTOR = new CubicPath(new Vector2d[] { // length of Vector[] need to be a length of 8, since this gets turned into 8 points
        new Vector2d(20.3,-50),
        new Vector2d(30,-40),
        new Vector2d(25.4,-30),
        new Vector2d(40,-10),
        new Vector2d(30,10),
        new Vector2d(50,30),
        new Vector2d(40,30),
        new Vector2d(10,-20)
    });

    public static final CubicPath EXAMPLE_CUBIC_PATH_REVERSED = reversePath(EXAMPLE_CUBIC_PATH_DOUBLE);

    public static final CubicPath EXAMPLE_CUBIC_PATH_SWAP_SIDES = toRed(EXAMPLE_CUBIC_PATH_VECTOR);
}