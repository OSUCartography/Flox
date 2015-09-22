package edu.oregonstate.cartography.utils;

import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import static java.lang.Double.isNaN;
import static java.lang.Math.abs;
import java.util.ArrayList;
import java.util.Iterator;

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

    protected static double det(double a, double b, double c, double d) {
        return a * d - b * c;
    }

    /**
     * Get the azimuth (in radians) of a line connecting two points
     *
     * @param startPt The start point of the line
     * @param endPt The end point of the line
     * @return
     */
    public static double computeAzimuth(Point startPt, Point endPt) {
        final double dx = endPt.x - startPt.x;
        final double dy = endPt.y - startPt.y;
        return Math.atan2(dy, dx);
    }

    public static boolean detectFlowCollisionWithRectangle(Flow flow, Rectangle2D rect, double pixelTolerance) {
        return false;
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
        double distToLine = (abs((y0 - y1) * x + (x1 - x0) * y + (x0 * y1 - x1 * y0))
                / (Math.sqrt(((x1 - x0) * (x1 - x0)) + ((y1 - y0) * (y1 - y0)))));
        return isNaN(distToLine) ? 0 : distToLine;
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
     * Returns the closest Point on a line segment to a point.
     *
     * @param p point to find closest point on segment to
     * @param sp segment start point
     * @param ep segment end point
     * @return closest point on segment to p
     */
    public static Point getClosestPointOnSegment(Point p, Point sp, Point ep) {
        return getClosestPointOnSegment(p.x, p.y, sp.x, sp.y, ep.x, ep.y);
    }

    /**
     * Returns the closest Point on a line segment to a point.
     *
     * @param px point x coordinate
     * @param py point y coordinate
     * @param spx segment start point x coordinate
     * @param spy segment start point y coordinate
     * @param epx segment end point x coordinate
     * @param epy segment end point y coordinate
     * @return closets point on segment to point
     */
    public static Point getClosestPointOnSegment(double px, double py, double spx, double spy,
            double epx, double epy) {

        double xDelta = epx - spx;
        double yDelta = epy - spy;

        if ((xDelta == 0) && (yDelta == 0)) {
            throw new IllegalArgumentException("Segment start equals segment end");
        }

        double u = ((px - spx) * xDelta + (py - spy) * yDelta) / (xDelta * xDelta + yDelta * yDelta);

        final Point closestPoint;
        if (u < 0) {
            closestPoint = new Point(spx, spy);
        } else if (u > 1) {
            closestPoint = new Point(epx, epy);
        } else {
            closestPoint = new Point((spx + u * xDelta), (spy + u * yDelta));
        }

        return closestPoint;
    }

    /**
     * Returns the closest point on a flow to a point. Actually finds the
     * nearest point on the nearest line segment along the flow to a point.
     *
     * @param flow The flow to find the closest point on.
     * @param pt The point to find the point on the flow closest to.
     * @param deCasteljauTol Determines how many segments are along the flow,
     * value should be passed from the model.
     * @return The Point along the flow closest to p.
     */
    public static Point getClosestPointOnFlow(Flow flow, Point pt, double deCasteljauTol) {

        ArrayList<Point> flowPts = flow.toStraightLineSegments(deCasteljauTol);

        // Find the segment closest to pt
        double shortestDistSquare = Double.POSITIVE_INFINITY;

        // FIXME
        // This is redundant, this assignment is made in the for loop. But the
        // points need to be initialized with something for NetBeans to be ok
        // with the return statement at the end. There's probably a nicer way 
        // to do this.
        Point pt1 = flowPts.get(0);
        Point pt2 = flowPts.get(1);
        for (int i = 0; i < (flowPts.size() - 1); i++) {
            pt1 = flowPts.get(i);
            pt2 = flowPts.get(i + 1);

            double distSquare = GeometryUtils.getDistanceToLineSegmentSquare(
                    pt.x, pt.y, pt1.x, pt1.y, pt2.x, pt2.y);

            if (distSquare < shortestDistSquare) {
                shortestDistSquare = distSquare;
                // save the endpoints of the nearest segment
            } else {
                pt2 = flowPts.get(i - 1);
                break;
            }

        }

        return getClosestPointOnSegment(pt, pt1, pt2);

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
     * Returns an ArrayList of all flows that intersect nodes in the provided
     * data model.
     *
     * @param model The data model containing flows and nodes.
     * @param scale The scale of the mapComponent.
     * @return
     */
    public static ArrayList<QuadraticBezierFlow> getFlowsThatIntersectNodes(Model model, double scale) throws IOException {
        ArrayList<QuadraticBezierFlow> flowsArray = new ArrayList();
        Iterator<Flow> flowIterator = model.flowIterator();
        while (flowIterator.hasNext()) {
            QuadraticBezierFlow flow = (QuadraticBezierFlow) flowIterator.next();
            Iterator<Point> nodeIterator = model.nodeIterator();
            while (nodeIterator.hasNext()) {
                Point node = nodeIterator.next();
                if (node != flow.getStartPt() && node != flow.getEndPt()) {
                    if (GeometryUtils.flowIntersectsNode(flow, node, model, scale)) {
                        flowsArray.add(flow);
                        break;
                    }
                }
            }
        }
        return flowsArray;
    }

    /**
     * Returns true if the provided flow intersects the provided node.
     *
     * @param flow A Flow.
     * @param node A Node.
     * @param model The current data model
     * @param mapScale The current scale of the mapComponent
     * @return
     */
    public static boolean flowIntersectsNode(QuadraticBezierFlow flow, Point node,
            Model model, double mapScale) throws IOException {

        final double NODE_TOLERANCE_PX = 4;

        // Get the pixel width of the flow
        // Get the locked scale factor needed to calculate flow widths
        double lockedScaleFactor;
        if (!model.isScaleLocked()) {
            lockedScaleFactor = 1;
        } else {
            // compare the locked scale to the current scale
            double lockedMapScale = model.getLockedMapScale();
            lockedScaleFactor = mapScale / lockedMapScale;
        }

        double deCasteljauTol = model.getDeCasteljauTolerance();

        // Get the current stroke width of the flow in pixels
        double flowStrokeWidthPx = Math.abs(flow.getValue()) * model.getFlowWidthScaleFactor()
                * lockedScaleFactor;

        // Find out what that width is in world coordinates
        double worldStrokeWidth = (flowStrokeWidthPx) / mapScale;

        // Get the current pixel radius of the node
        double nodeArea = Math.abs(node.getValue() * model.getNodeSizeScaleFactor());

        double nodeRadiusPx = (Math.sqrt(nodeArea / Math.PI)) * lockedScaleFactor;

        // Find out what that radius is in world coordinates
        // Add a bit to the pixel radius in order to make the radius a few pixels 
        // wider than the actual node and to account for the node's stroke width. 
        double worldNodeRadius = (nodeRadiusPx + NODE_TOLERANCE_PX) / mapScale;

        // Add the worldNodeRadius to half the worldFlowWidth
        double threshDist = (worldStrokeWidth / 2) + worldNodeRadius;

        // Get the flow's bounding box, and add a padding to it of threshDist.
        Rectangle2D flowBB = flow.getBoundingBox();
        flowBB.add((flowBB.getMinX() - threshDist), (flowBB.getMinY() - threshDist));
        flowBB.add((flowBB.getMaxX() + threshDist), (flowBB.getMaxY() + threshDist));

        // If flowBB contains the node's coordinates, then check the shortest
        // distance between the node and the flow. If it's less than the 
        // threshold, then it intersects. 
        double shortestDistSquare = Double.POSITIVE_INFINITY;

        if (flowBB.contains(node.x, node.y)) {
            double[] xy = {node.x, node.y};
            shortestDistSquare = flow.distanceSq(xy);
            
            /*
             // FIXME Could we use flow.getDistanceToQuadraticBezierCurveSq instead here
             // This would not require the conversion to line segments and therefore could be faster.
             ArrayList<Point> pts = flow.toStraightLineSegments(deCasteljauTol);
             for (int i = 0; i < (pts.size() - 1); i++) {

             Point pt1 = pts.get(i);
             Point pt2 = pts.get(i + 1);

             double distSquare = getDistanceToLineSegmentSquare(node.x, node.y,
             pt1.x, pt1.y, pt2.x, pt2.y);

             if (distSquare < shortestDistSquare) {
             shortestDistSquare = distSquare;
             } else {
             break;
             }
             }*/
        } else {
            return false;
        }

        // FIXME 
        // Comment by Bernie: The method name and the JavaDoc imply that the method is 
        // performing a flow/node overlap test. The following code however seems 
        // to be testing whether the flow can be moved, which I find confusing.
        // Should this test be moved to a separate method?
        if (shortestDistSquare < threshDist * threshDist) {

            // It intersects the flow. Check to see if it's possible to move
            // the flow off it. If it isn't, ERROR. If it is, return True.
            Point sPt = flow.getStartPt();
            Point ePt = flow.getEndPt();

            // world width of the flow being checked is worldStrokeWidth
            // world radius of the node it crosses is worldNodeRadius
            // FIXME comment by Bernie: could you explain how this test works?
            if (sPt.distance(node) - (worldNodeRadius + worldStrokeWidth / 2) < 0) {

                // FIXME System.out should not be used in productive code
                System.out.println("Impossible!");
                // FIXME comment by Bernie: IOException should be used when data
                // cannot be read or written.
                // This is also a misuse of exceptions. Exceptions should not be 
                // used to indicate software states that commonly occur (they are 
                // slow and it is considered poor style to use exceptions in this way).
                // If this test ("isFlowMoveable") can be moved to a separate 
                // method, the method can simply return true or false, and no 
                // exception needs to be used.
                throw new IOException();
                //return false;
            }

            if (ePt.distance(node) - (worldNodeRadius + worldStrokeWidth / 2) < 0) {
                System.out.println("Impossible!");
                throw new IOException();
                //return false;
            }
            return true;

        } else {
            return false;
        }

        //return (shortestDistSquare < threshDist * threshDist);
    }

    /**
     * Test whether three points align.
     *
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @param x3
     * @param y3
     * @return
     */
    public static boolean collinear(double x1, double y1, double x2, double y2, double x3, double y3) {
        return Math.abs((y1 - y2) * (x1 - x3) - (y1 - y3) * (x1 - x2)) <= 1e-9;
    }

    private static double cuberoot(double x) {
        if (x < 0.0f) {
            return -Math.pow(-x, 1.0 / 3.0);
        }
        return Math.pow(x, 1.0 / 3.0);
    }

    /**
     * Find roots in cubic equation of the form x^3 + aáx^2 + báx + c = 0 From
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
        double m = Math.cos(v), n = Math.sin(v) * 1.732050808;
        r[0] = offset + u * (m + m);
        r[1] = offset - u * (n + m);
        r[2] = offset + u * (n - m);
        return 3;
    }

    /**
     * Computes the square of the shortest distance between a point and any
     * point on a quadratic BŽzier curve. Attention: xy parameter is changed.
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
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @return The square distance between the point x/y and the quadratic
     * Bezier curve.
     */
    public static double getDistanceToQuadraticBezierCurveSq(double p0x, double p0y,
            double p1x, double p1y,
            double p2x, double p2y,
            double[] xy) {

        if (collinear(p0x, p0y, p1x, p1y, p2x, p2y)) {
            return getDistanceToLineSegmentSquare(xy[0], xy[1], p0x, p0y, p2x, p2y);
        }

        double dx1 = p0x - xy[0];
        double dy1 = p0y - xy[1];
        double d0sq = dx1 * dx1 + dy1 * dy1;
        double dx2 = p2x - xy[0];
        double dy2 = p2y - xy[1];
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
        double k1 = bx * bx + by * by + 2.0 * ((cx - xy[0]) * ax + (cy - xy[1]) * ay);
        double k0 = (cx - xy[0]) * bx + (cy - xy[1]) * by;

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
                // point on BŽzier curve
                double posx = w0 * p0x + w1 * p1x + w2 * p2x;
                double posy = w0 * p0y + w1 * p1y + w2 * p2y;

                double dx = posx - xy[0];
                double dy = posy - xy[1];
                double distSq = dx * dx + dy * dy;
                if (distSq < minDistSq) {
                    minDistSq = dx * dx + dy * dy;
                    xy[0] = posx;
                    xy[1] = posy;
                }
            }
        }

        return minDistSq;
    }

    /**
     * Computes the shortest distance between a point and any point on a
     * quadratic BŽzier curve. Attention: xy parameter is changed.
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
            double[] xy) {
        double dSq = getDistanceToQuadraticBezierCurveSq(p0x, p0y, p1x, p1y, p2x, p2y, xy);
        return Math.sqrt(dSq);
    }

    /**
     * Moves the control point of a flow perpendicularly to the baseline by one
     * pixel.
     *
     * FIXME The statement above is probably not correct, as coordinates of
     * flows are not in pixels. See FIXME comment below.
     *
     * @param flows
     */
    public static void moveFlowsThatCrossNodes(ArrayList<QuadraticBezierFlow> flows, double scale) {

        for (QuadraticBezierFlow flow : flows) {

            // Collect needed points from the flow
            Point cPt = flow.getCtrlPt();
            Point sPt = flow.getStartPt();
            Point ePt = flow.getEndPt();

            // Get the distance of startPt to endPt
            double dx = ePt.x - sPt.x;
            double dy = ePt.y - sPt.y;
            double dist = Math.sqrt(dx * dx + dy * dy);

            // Create a point known to be on the right side of the line.
            Point rightPt;
            // FIXME +1 and -1 seem to be in ground coordinates of the flow, not in pixels.
            if (dy > 0) {
                rightPt = new Point(sPt.x + 1, sPt.y);
            } else if (dy < 0) {
                rightPt = new Point(sPt.x - 1, sPt.y);
            } else {
                // dy is 0
                if (dx > 0) {
                    rightPt = new Point(sPt.x, sPt.y - 1);
                } else {
                    rightPt = new Point(sPt.x, sPt.y + 1);
                }
            }

            // Get the d value of rightPt. The d value will be positive if it's
            // on one side of the flow's baseline, and negative if it's on the 
            // other, but we don't know if the right side is positive or
            // negative. This will allow us to find out.
            double rightPtD = (rightPt.x - sPt.x) * (ePt.y - sPt.y) - (rightPt.y - sPt.y) * (ePt.x - sPt.x);

            // Get the d value of the flow's control point.
            double pt0D = (cPt.x - sPt.x) * (ePt.y - sPt.y) - (cPt.y - sPt.y) * (ePt.x - sPt.x);

            // Initiallize the perpendicular unitVector of the flow's baseline.
            // The values assigned to these will depend on whether the control
            // point is on the right or left side of the baseline.
            double unitVectorX;
            double unitVectorY;

            // if pt0D and rightPtD have the same polarity, than the conrol point
            // is on the right side! Set the unitVector accordingly.
            // If either d value is 0 (the point lies directly on top of the 
            // baseline) move the control point to the left arbitrarily.  
            if ((pt0D > 0 && rightPtD > 0) || (pt0D < 0 && rightPtD < 0)) {
                unitVectorX = dy / dist;
                unitVectorY = -dx / dist;
            } else if (pt0D == 0 || rightPtD == 0) {
                unitVectorX = -dy / dist;
                unitVectorY = dx / dist;
            } else {
                unitVectorX = -dy / dist;
                unitVectorY = dx / dist;
            }

            // If the distance from the control point to the baseline is more 
            // than twice the length of the baseline
            // move the control point to the baseline centerpoint,
            // and reverse the vectorUnits' polarity. This amounts to flipping
            // the flow to the other side.
            double distFromBaseline = GeometryUtils.getDistanceToLine(cPt.x, cPt.y, sPt.x, sPt.y, ePt.x, ePt.y);

            if (distFromBaseline > flow.getBaselineLength() * 2) {
                cPt.x = flow.getBaseLineMidPoint().x;
                cPt.y = flow.getBaseLineMidPoint().y;
                unitVectorX *= -1;
                unitVectorY *= -1;
            }

            // Add the unitVectors to the control point. Also, multiply the
            // unitVectors by 2. This will cut the iterations in half without
            // losing significant fidelity. 
            cPt.x += (unitVectorX * 2 / scale);
            cPt.y += (unitVectorY * 2 / scale);

            // Lock the flow. This is to prevent it from moving when forces
            // are reapplied to the layout.
            flow.setLocked(true);
        }
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

}
