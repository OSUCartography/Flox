/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.oregonstate.cartography.flox.model;

/**
 *
 * @author Maccabee
 */
public class RangeboxEnforcer {

    /**
     * Copied from here:
     * http://www.java-gaming.org/index.php?topic=22590.0
     * 
     * The coordinates below are the endpoints of two line segments.
     * The method returns true if the line segments intersect.
     * Points 1 & 2 are a line segment
     * Points 3 & 4 are a line segment
     * 
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @param x3
     * @param y3
     * @param x4
     * @param y4
     * @return 
     */
    public static boolean linesIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        // Return false if either of the lines have zero length
        if (x1 == x2 && y1 == y2
                || x3 == x4 && y3 == y4) {
            return false;
        }
        // Fastest method, based on Franklin Antonio's "Faster Line Segment Intersection" topic "in Graphics Gems III" book (http://www.graphicsgems.org/)
        double ax = x2 - x1;
        double ay = y2 - y1;
        double bx = x3 - x4;
        double by = y3 - y4;
        double cx = x1 - x3;
        double cy = y1 - y3;

        double alphaNumerator = by * cx - bx * cy;
        double commonDenominator = ay * bx - ax * by;
        if (commonDenominator > 0) {
            if (alphaNumerator < 0 || alphaNumerator > commonDenominator) {
                return false;
            }
        } else if (commonDenominator < 0) {
            if (alphaNumerator > 0 || alphaNumerator < commonDenominator) {
                return false;
            }
        }
        double betaNumerator = ax * cy - ay * cx;
        if (commonDenominator > 0) {
            if (betaNumerator < 0 || betaNumerator > commonDenominator) {
                return false;
            }
        } else if (commonDenominator < 0) {
            if (betaNumerator > 0 || betaNumerator < commonDenominator) {
                return false;
            }
        }
        if (commonDenominator == 0) {
            // This code wasn't in Franklin Antonio's method. It was added by Keith Woodward.
            // The lines are parallel.
            // Check if they're collinear.
            double y3LessY1 = y3 - y1;
            double collinearityTestForP3 = x1 * (y2 - y3) + x2 * (y3LessY1) + x3 * (y1 - y2);   // see http://mathworld.wolfram.com/Collinear.html
            // If p3 is collinear with p1 and p2 then p4 will also be collinear, since p1-p2 is parallel with p3-p4
            if (collinearityTestForP3 == 0) {
                // The lines are collinear. Now check if they overlap.
                if (x1 >= x3 && x1 <= x4 || x1 <= x3 && x1 >= x4
                        || x2 >= x3 && x2 <= x4 || x2 <= x3 && x2 >= x4
                        || x3 >= x1 && x3 <= x2 || x3 <= x1 && x3 >= x2) {
                    if (y1 >= y3 && y1 <= y4 || y1 <= y3 && y1 >= y4
                            || y2 >= y3 && y2 <= y4 || y2 <= y3 && y2 >= y4
                            || y3 >= y1 && y3 <= y2 || y3 <= y1 && y3 >= y2) {
                        return true;
                    }
                }
            }
            return false;
        }
        return true;
    }

    /**
     * Copied from here:
     * http://www.java-gaming.org/index.php?topic=22590.0
     * 
     * Finds the intersection between two infinite lines.
     * Returns null if they are parallel.
     * The coordinates below define two infinite lines.
     * Points 1 & 2 are along one line
     * Points 3 & 4 are along a second line
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @param x3
     * @param y3
     * @param x4
     * @param y4
     * @return 
     */
    public static Point getLineLineIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det1And2 = det(x1, y1, x2, y2);
        double det3And4 = det(x3, y3, x4, y4);
        double x1LessX2 = x1 - x2;
        double y1LessY2 = y1 - y2;
        double x3LessX4 = x3 - x4;
        double y3LessY4 = y3 - y4;
        double det1Less2And3Less4 = det(x1LessX2, y1LessY2, x3LessX4, y3LessY4);
        if (det1Less2And3Less4 == 0) {
            // the denominator is zero so the lines are parallel and there's either no solution (or multiple solutions if the lines overlap) so return null.
            return null;
        }
        double x = (det(det1And2, x1LessX2,
                det3And4, x3LessX4)
                / det1Less2And3Less4);
        double y = (det(det1And2, y1LessY2,
                det3And4, y3LessY4)
                / det1Less2And3Less4);
        return new Point(x, y);
    }

    protected static double det(double a, double b, double c, double d) {
        return a * d - b * c;
    }

    /**
     * If the control point of a flow falls outside of the flow's range box, 
     * this returns the intersection between a line connecting the control point
     * to the midpoint of the baseline, and the location along the rangebox's
     * border where the line crosses.
     * Checks each side of the range rectangle one at a time.
     * @param flow A QuadraticBezierFlow
     * @return 
     */
    public static Point enforceRange(QuadraticBezierFlow flow) {

        Point cPt = flow.getCtrlPt();
        Point refPt = flow.getBaseLineMidPoint();

        if (linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                flow.b1.x, flow.b1.y,
                flow.b2.x, flow.b2.y)) {
            return getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    flow.b1.x, flow.b1.y,
                    flow.b2.x, flow.b2.y);
        }

        if (linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                flow.b3.x, flow.b3.y,
                flow.b4.x, flow.b4.y)) {
            return getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    flow.b3.x, flow.b3.y,
                    flow.b4.x, flow.b4.y);
        }
        
        if (linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                flow.b1.x, flow.b1.y,
                flow.b3.x, flow.b3.y)) {
            return getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    flow.b1.x, flow.b1.y,
                    flow.b3.x, flow.b3.y);
        }
        
        if (linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                flow.b2.x, flow.b2.y,
                flow.b4.x, flow.b4.y)) {
            return getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    flow.b2.x, flow.b2.y,
                    flow.b4.x, flow.b4.y);
        }
        
        return cPt;
    }

}
