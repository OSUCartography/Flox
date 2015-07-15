package edu.oregonstate.cartography.utils;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.linearref.LinearGeometryBuilder;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import java.awt.geom.Rectangle2D;
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
     * Get the bounding box for an ArrayList of Points.
     *
     * @param points An ArrayList of Points
     * @return A Rectangle2D object that contains all the Points.
     */
    public static Rectangle2D getBoundingBoxOfPoints(ArrayList<Point> points) {

        Rectangle2D.Double bb = new Rectangle2D.Double(points.get(0).x, points.get(0).y, 0, 0);
        for (int i = 1; i < points.size(); i++) {
            Point pt = points.get(i);
            bb.add(pt.x, pt.y);
        }

        return bb;
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
     * Returns an ArrayList of all flows that intersect nodes in the provided
     * data model.
     * @param model The data model containing flows and nodes.
     * @param scale The scale of the mapComponent.
     * @return 
     */
    public static ArrayList<QuadraticBezierFlow> getFlowsThatIntersectNodes(Model model, double scale) {
        
        ArrayList<QuadraticBezierFlow> flowsArray = new ArrayList();
        
        Iterator flows = model.flowIterator();
        ArrayList<Point> nodes = model.getNodes();
        while (flows.hasNext()) {
            QuadraticBezierFlow flow = (QuadraticBezierFlow) flows.next();
            for (Point node : nodes) {
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
     * @param flow A Flow.
     * @param node A Node.
     * @param model The current data model
     * @param mapScale The current scale of the mapComponent
     * @return 
     */
    public static boolean flowIntersectsNode(Flow flow, Point node,
            Model model, double mapScale) {

        // Get the pixel width of the flow
        // Get the locked scale factor needed to calculate flow widths
        double lockedScaleFactor;
        if (!model.isFlowWidthLocked()) {
            lockedScaleFactor = 1;
        } else {
            // compare the locked scale to the current scale
            double lockedMapScale = model.getLockedMapScale();
            lockedScaleFactor = mapScale / lockedMapScale;
        }

        double deCasteljauTol = model.getDeCasteljauTolerance();

        // Get the current stroke width of the flow in pixels
        double flowStrokeWidth = Math.abs(flow.getValue()) * model.getFlowWidthScaleFactor()
                * lockedScaleFactor;

        // Find out what that width is in world coordinates
        double worldStrokeWidth = (flowStrokeWidth) / mapScale;

        // Get the current pixel radius of the node
        double nodeArea = Math.abs(node.getValue()
                * model.getNodeSizeScaleFactor());

        double nodeRadius = (Math.sqrt(nodeArea / Math.PI)) * lockedScaleFactor;

        // Find out what that radius is in world coordinates
        // Add a bit to the pixel radius in order tomake the radius a few pixels 
        // wider than the actual node and to account for the node's stroke width. 
        double worldNodeRadius = (nodeRadius + 4) / mapScale;

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

            ArrayList<Point> pts = flow.toStraightLineSegments(deCasteljauTol);
            for (int i = 0; i < (pts.size() - 1); i++) {

                Point pt1 = pts.get(i);
                Point pt2 = pts.get(i + 1);

                double distSquare = getDistanceToLineSegmentSquare(node.x, node.y,
                        pt1.x, pt1.y, pt2.x, pt2.y);
                
                if(distSquare < shortestDistSquare) {
                    shortestDistSquare = distSquare;
                } else {
                    break;
                }
            }
        }

        return (shortestDistSquare < threshDist * threshDist);
    }

}
