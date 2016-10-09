package edu.oregonstate.cartography.utils;

import edu.oregonstate.cartography.flox.model.Point;
import java.awt.geom.Rectangle2D;

public class GeometryUtils {

    /**
     *
     * This method returns true if 2 line segments intersect. The coordinates
     * below are the endpoints of two line segments. Points 1 & 2 are a line
     * segment Points 3 & 4 are a line segment
     *
     * Copied from here: http://www.java-gaming.org/index.php?topic=22590.0
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
    public static boolean linesIntersect(double x1, double y1,
            double x2, double y2,
            double x3, double y3,
            double x4, double y4) {
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
     * Copied from here: http://www.java-gaming.org/index.php?topic=22590.0
     *
     * Finds the intersection between two infinite lines. Returns null if they
     * are parallel. The coordinates below define two infinite lines. Points 1 &
     * 2 are along one line Points 3 & 4 are along a second line
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
    public static Point getLineLineIntersection(double x1, double y1,
            double x2, double y2,
            double x3, double y3,
            double x4, double y4) {
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

    /**
     * Copied from here: http://www.java-gaming.org/index.php?topic=22590.0
     *
     * Finds the intersection between two infinite lines. The found intersection
     * is stored in destination Point. The destination Point is not changed if
     * the lines are parallel. The parameters define two infinite lines. Points
     * 1 & 2 are along one line Points 3 & 4 are along a second line
     *
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @param x3
     * @param y3
     * @param x4
     * @param y4
     * @param destination
     * @return true if an intersection was found, that is, the lines are not
     * parallel.
     */
    public static boolean getLineLineIntersection(double x1, double y1,
            double x2, double y2,
            double x3, double y3,
            double x4, double y4,
            Point destination) {
        double det1And2 = det(x1, y1, x2, y2);
        double det3And4 = det(x3, y3, x4, y4);
        double x1LessX2 = x1 - x2;
        double y1LessY2 = y1 - y2;
        double x3LessX4 = x3 - x4;
        double y3LessY4 = y3 - y4;
        double det1Less2And3Less4 = det(x1LessX2, y1LessY2, x3LessX4, y3LessY4);
        if (det1Less2And3Less4 == 0) {
            // the denominator is zero the lines are parallel and there is 
            // either no solution or multiple solutions if the lines overlap.
            return false;
        }
        destination.x = (det(det1And2, x1LessX2,
                det3And4, x3LessX4)
                / det1Less2And3Less4);
        destination.y = (det(det1And2, y1LessY2,
                det3And4, y3LessY4)
                / det1Less2And3Less4);
        return true;
    }

    protected static double det(double a, double b, double c, double d) {
        return a * d - b * c;
    }

    /**
     * Get the orientation angle (in radians) of a line connecting two points.
     *
     * @param startPt The start point of the line
     * @param endPt The end point of the line
     * @return angle relative to horizontal x axis in counter-clockwise
     * direction
     */
    public static double orientation(Point startPt, Point endPt) {
        final double dx = endPt.x - startPt.x;
        final double dy = endPt.y - startPt.y;
        return Math.atan2(dy, dx);
    }

    /**
     * Compute the shortest distance between a point and a line. Assumes a line
     * of infinite length on which the 2nd and 3rd set of coordinates lie.
     *
     * @param x Point who's distance is being measured x
     * @param y Point who's distance is being measured y
     * @param x0 Line start point x
     * @param y0 Line start point y
     * @param x1 Line end point x
     * @param y1 Line end point y
     * @return
     */
    public static double getDistanceToLine(double x, double y,
            double x0, double y0, double x1, double y1) {
        double distToLine = (Math.abs((y0 - y1) * x + (x1 - x0) * y + (x0 * y1 - x1 * y0))
                / (Math.sqrt(((x1 - x0) * (x1 - x0)) + ((y1 - y0) * (y1 - y0)))));
        return Double.isNaN(distToLine) ? 0 : distToLine;
    }

    /**
     * Calculates the shortest distance from a point to a finite line. Copied
     * from a stackoverflow forum post.
     * http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * segment.
     *
     * @param x X coordinate of point.
     * @param y Y coordinate of point.
     * @param x1 X coordinate of start point of line segment.
     * @param y1 Y coordinate of start point of line segment.
     * @param x2 X coordinate of end point of line segment.
     * @param y2 Y coordinate of end point of line segment.
     * @return Distance between point and line segment.
     */
    public static double getDistanceToLineSegment(double x, double y,
            double x1, double y1, double x2, double y2) {
        double A = x - x1;
        double B = y - y1;
        double C = x2 - x1;
        double D = y2 - y1;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = -1;
        if (len_sq != 0) {
            param = dot / len_sq;
        }

        double xx, yy;

        if (param < 0) {
            xx = x1;
            yy = y1;
        } else if (param > 1) {
            xx = x2;
            yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        double dx = x - xx;
        double dy = y - yy;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Gets the squared distance to a line segment from a point.
     *
     * @param p Point to get the distance to the segment from
     * @param sp Start point of line segment.
     * @param ep End point of line segment.
     * @return
     */
    public static double getDistanceToLineSegementSquare(Point p, Point sp, Point ep) {
        return getDistanceToLineSegmentSquare(p.x, p.y, sp.x, sp.y, ep.x, ep.y);
    }

    /**
     * Calculates the square value of the shortest distance from a point to a
     * finite line. Copied from a stackoverflow forum post.
     * http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * segment.
     *
     * @param x X coordinate of point.
     * @param y Y coordinate of point.
     * @param x1 X coordinate of start point of line segment.
     * @param y1 Y coordinate of start point of line segment.
     * @param x2 X coordinate of end point of line segment.
     * @param y2 Y coordinate of end point of line segment.
     * @return Distance between point and line segment.
     */
    public static double getDistanceToLineSegmentSquare(double x, double y,
            double x1, double y1, double x2, double y2) {
        double A = x - x1;
        double B = y - y1;
        double C = x2 - x1;
        double D = y2 - y1;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = -1;
        if (len_sq != 0) {
            param = dot / len_sq;
        }

        double xx, yy;

        if (param < 0) {
            xx = x1;
            yy = y1;
        } else if (param > 1) {
            xx = x2;
            yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        double dx = x - xx;
        double dy = y - yy;
        return dx * dx + dy * dy;
    }

    /**
     * Compute the difference between two angles. The resulting angle is in the
     * range of -pi..+pi if the input angle are also in this range.
     */
    public static double angleDif(double a1, double a2) {
        double val = a1 - a2;
        if (val > Math.PI) {
            val -= 2. * Math.PI;
        }
        if (val < -Math.PI) {
            val += 2. * Math.PI;
        }
        return val;
    }

    /**
     * Sum of two angles. The resulting angle is in the range of -pi..+pi if the
     * input angle are also in this range.
     */
    public static float angleSum(float a1, float a2) {
        double val = a1 + a2;
        if (val > Math.PI) {
            val -= 2. * Math.PI;
        }
        if (val < -Math.PI) {
            val += 2. * Math.PI;
        }
        return (float) val;
    }

    public static double trimAngle(double angle) {
        if (angle > Math.PI) {
            return angle - 2. * Math.PI;
        }
        if (angle < -Math.PI) {
            return angle + 2. * Math.PI;
        }
        return angle;
    }

    /**
     * Test whether three points align.
     *
     * @param x1 point 1 x
     * @param y1 point 1 y
     * @param x2 point 2 x
     * @param y2 point 2 y
     * @param x3 point 3 x
     * @param y3 point 3 y
     * @param tol tolerance the points are collinear if the deviation is shorter
     * than tolerance.
     * @return
     */
    public static boolean collinear(double x1, double y1, double x2, double y2, double x3, double y3, double tol) {
        return Math.abs((y1 - y2) * (x1 - x3) - (y1 - y3) * (x1 - x2)) < tol;
    }

    private static double cuberoot(double x) {
        if (x < 0.0f) {
            return -Math.pow(-x, 1.0 / 3.0);
        }
        return Math.pow(x, 1.0 / 3.0);
    }

    /**
     * Find roots in cubic equation of the form x^3 + a·x^2 + b·x + c = 0 From
     * http://www.pouet.net/topic.php?which=9119&page=1
     *
     * @param a
     * @param b
     * @param c
     * @param r Array that will receive solutions.
     * @return The number of solutions.
     */
    private static int solveCubic(double a, double b, double c, double[] r) {
        double p = b - a * a / 3.0;
        double q = a * (2.0 * a * a - 9.0 * b) / 27.0 + c;
        double p3 = p * p * p;
        double d = q * q + 4.0 * p3 / 27.0;
        double offset = -a / 3.0;
        if (d >= 0) { // Single solution
            double z = Math.sqrt(d);
            double u = (-q + z) / 2.0;
            double v = (-q - z) / 2.0;
            u = cuberoot(u);
            v = cuberoot(v);
            r[0] = offset + u + v;
            return 1;
        }
        double u = Math.sqrt(-p / 3);
        double v = Math.acos(-Math.sqrt(-27.0 / p3) * q / 2.0) / 3.0;
        double m = Math.cos(v), n = Math.sin(v) * Math.sqrt(3.0);
        r[0] = offset + u * (m + m);
        r[1] = offset - u * (n + m);
        r[2] = offset + u * (n - m);
        return 3;
    }

    /**
     * Computes the square of the shortest distance between a point and any
     * point on a quadratic Bézier curve. Also computes the location of the
     * point on the flow with the shortest distance.
     *
     * Attention: xy parameter is changed. 
     * 
     * Based on
     * http://blog.gludion.com/2009/08/distance-to-quadratic-bezier-curve.html
     * and http://www.pouet.net/topic.php?which=9119&page=2
     *
     * @param p0x Start point x
     * @param p0y Start point y
     * @param p1x Control point x
     * @param p1y Control point y
     * @param p2x End point x
     * @param p2y End point x
     * @param tol Tolerance to test whether points are collinear.
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @return The square distance between the point x/y and the quadratic
     * Bezier curve.
     */
    public static double getDistanceToQuadraticBezierCurveSq(double p0x, double p0y,
            double p1x, double p1y,
            double p2x, double p2y,
            double tol,
            double[] xy) {

        double x = xy[0];
        double y = xy[1];

        if (collinear(p0x, p0y, p1x, p1y, p2x, p2y, tol)) {
            return getDistanceToLineSegmentSquare(x, y, p0x, p0y, p2x, p2y);
        }

        double dx1 = p0x - x;
        double dy1 = p0y - y;
        double d0sq = dx1 * dx1 + dy1 * dy1;
        double dx2 = p2x - x;
        double dy2 = p2y - y;
        double d2sq = dx2 * dx2 + dy2 * dy2;
        double minDistSq = Math.min(d0sq, d2sq);

        double ax = p0x - 2.0 * p1x + p2x;
        double ay = p0y - 2.0 * p1y + p2y;
        double bx = 2.0 * (p1x - p0x);
        double by = 2.0 * (p1y - p0y);
        double cx = p0x;
        double cy = p0y;

        double k3 = 2.0 * (ax * ax + ay * ay);
        double k2 = 3.0 * (ax * bx + ay * by);
        double k1 = bx * bx + by * by + 2.0 * ((cx - x) * ax + (cy - y) * ay);
        double k0 = (cx - x) * bx + (cy - y) * by;

        // FIXME allocating this array each time might not be efficient
        double res[] = new double[3];
        int n = solveCubic(k2 / k3, k1 / k3, k0 / k3, res);
        for (int i = 0; i < n; i++) {
            double t = res[i];
            if (t >= 0.0 && t <= 1.0) {
                double _1_t = 1.0 - t;
                double w0 = _1_t * _1_t;
                double w1 = 2.0 * t * _1_t;
                double w2 = t * t;
                // point on Bézier curve
                double posx = w0 * p0x + w1 * p1x + w2 * p2x;
                double posy = w0 * p0y + w1 * p1y + w2 * p2y;

                double dx = posx - x;
                double dy = posy - y;
                double distSq = dx * dx + dy * dy;
                if (distSq < minDistSq) {
                    minDistSq = distSq;
                    xy[0] = posx;
                    xy[1] = posy;
                }
            }
        }

        return minDistSq;
    }

    /**
     * Computes the square of the shortest distance between a point and any
     * point on a quadratic Bézier curve. Based on
     * http://blog.gludion.com/2009/08/distance-to-quadratic-bezier-curve.html
     * and http://www.pouet.net/topic.php?which=9119&page=2
     *
     * @param p0x Start point x
     * @param p0y Start point y
     * @param p1x Control point x
     * @param p1y Control point y
     * @param p2x End point x
     * @param p2y End point x
     * @param tol Tolerance to test whether points are collinear.
     * @param x Point x
     * @param y point y
     * @return The square distance between the point x/y and the quadratic
     * Bezier curve.
     */
    public static double getDistanceToQuadraticBezierCurveSq(double p0x, double p0y,
            double p1x, double p1y,
            double p2x, double p2y,
            double tol,
            double x, double y) {

        if (collinear(p0x, p0y, p1x, p1y, p2x, p2y, tol)) {
            return getDistanceToLineSegmentSquare(x, y, p0x, p0y, p2x, p2y);
        }

        double dx1 = p0x - x;
        double dy1 = p0y - y;
        double d0sq = dx1 * dx1 + dy1 * dy1;
        double dx2 = p2x - x;
        double dy2 = p2y - y;
        double d2sq = dx2 * dx2 + dy2 * dy2;
        double minDistSq = Math.min(d0sq, d2sq);

        double ax = p0x - 2.0 * p1x + p2x;
        double ay = p0y - 2.0 * p1y + p2y;
        double bx = 2.0 * (p1x - p0x);
        double by = 2.0 * (p1y - p0y);
        double cx = p0x;
        double cy = p0y;

        double k3 = 2.0 * (ax * ax + ay * ay);
        double k2 = 3.0 * (ax * bx + ay * by);
        double k1 = bx * bx + by * by + 2.0 * ((cx - x) * ax + (cy - y) * ay);
        double k0 = (cx - x) * bx + (cy - y) * by;

        // FIXME allocating this array each time might not be efficient
        double res[] = new double[3];
        int n = solveCubic(k2 / k3, k1 / k3, k0 / k3, res);
        for (int i = 0; i < n; i++) {
            double t = res[i];
            if (t >= 0.0 && t <= 1.0) {
                double _1_t = 1.0 - t;
                double w0 = _1_t * _1_t;
                double w1 = 2.0 * t * _1_t;
                double w2 = t * t;
                // point on Bézier curve
                double posx = w0 * p0x + w1 * p1x + w2 * p2x;
                double posy = w0 * p0y + w1 * p1y + w2 * p2y;

                double dx = posx - x;
                double dy = posy - y;
                double distSq = dx * dx + dy * dy;
                if (distSq < minDistSq) {
                    minDistSq = distSq;
                }
            }
        }

        return minDistSq;
    }

    /**
     * Computes the shortest distance between a point and any point on a
     * quadratic Bézier curve. Attention: xy parameter is changed.
     *
     * @param p0x Start point x
     * @param p0y Start point y
     * @param p1x Control point x
     * @param p1y Control point y
     * @param p2x End point x
     * @param p2y End point x
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @return Distance between the point x/y and the quadratic Bezier curve.
     */
    public static double getDistanceToQuadraticBezierCurve(double p0x, double p0y,
            double p1x, double p1y,
            double p2x, double p2y,
            double tol,
            double[] xy) {
        double dSq = getDistanceToQuadraticBezierCurveSq(p0x, p0y, p1x, p1y, p2x, p2y, tol, xy);
        return Math.sqrt(dSq);
    }

    /**
     * S-shaped smooth function using cubic Hermite interpolation
     * http://en.wikipedia.org/wiki/Smoothstep
     *
     * @param edge0 interpolated values for x below edge0 will be 0.
     * @param edge1 interpolated values for x above edge1 will be 1.
     * @param x The x value to interpolate a value for.
     * @return
     */
    public static double smoothstep(double edge0, double edge1, double x) {
        // scale, bias and saturate x to 0..1 range
        x = Math.max(0, Math.min(1, (x - edge0) / (edge1 - edge0)));
        // evaluate polynomial
        return x * x * (3 - 2 * x);

        // alternative smootherstep
        // return x * x * x * (x * (x * 6 - 15) + 10);
    }

    /**
     * Returns the squared distance between two points.
     *
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    public static double distSq(double x1, double y1, double x2, double y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return dx * dx + dy * dy;
    }

    /**
     * Returns the shortest distance between two rectangles. Returns 0 if the
     * rectangles intersect or touch. See
     * http://stackoverflow.com/questions/4978323/how-to-calculate-distance-between-two-rectangles-context-a-game-in-lua?rq=1
     *
     * @param x1
     * @param y1
     * @param x1b
     * @param y1b
     * @param x2
     * @param y2
     * @param x2b
     * @param y2b
     * @return
     */
    public static double rectDistSq(double x1, double y1, double x1b, double y1b,
            double x2, double y2, double x2b, double y2b) {
        boolean left = x2b < x1;
        boolean right = x1b < x2;
        boolean bottom = y2b < y1;
        boolean top = y1b < y2;
        if (top && left) {
            return distSq(x1, y1b, x2b, y2);
        }
        if (left && bottom) {
            return distSq(x1, y1, x2b, y2b);
        }
        if (bottom && right) {
            return distSq(x1b, y1, x2, y2b);
        }
        if (right && top) {
            return distSq(x1b, y1b, x2, y2);
        }
        if (left) {
            return (x1 - x2b) * (x1 - x2b);
        }
        if (right) {
            return (x2 - x1b) * (x2 - x1b);
        }
        if (bottom) {
            return (y1 - y2b) * (y1 - y2b);
        }
        if (top) {
            return (y2 - y1b) * (y2 - y1b);
        }
        return 0;
    }

    /**
     * Returns the shortest distance between two rectangles. Returns 0 if the
     * rectangles intersect or touch.
     *
     * @param r1
     * @param r2
     * @return
     */
    public static double rectDistSq(Rectangle2D r1, Rectangle2D r2) {
        return rectDistSq(r1.getMinX(), r1.getMinY(), r1.getMaxX(), r1.getMaxY(),
                r2.getMinX(), r2.getMinY(), r2.getMaxX(), r2.getMaxY());
    }

}
