package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryCollectionIterator;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;
import java.awt.geom.GeneralPath;
import java.awt.geom.PathIterator;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class QuadraticBezierFlow extends Flow {

    /**
     * control point.
     */
    private Point cPt;

    /**
     * Construct a QuadraticBezierFlow from 2 irregularPoints
     *
     * @param startPt Start point
     * @param ctrlPt Control point
     * @param endPt End point
     */
    public QuadraticBezierFlow(Point startPt, Point ctrlPt, Point endPt) {
        this.startPt = startPt;
        this.cPt = ctrlPt;
        this.endPt = endPt;
    }

    /**
     * Construct a QuadraticBezierFlow from 2 irregularPoints
     *
     * @param startPt Start point
     * @param endPt End point
     */
    public QuadraticBezierFlow(Point startPt, Point endPt) {

        this.startPt = startPt;
        this.endPt = endPt;

        // Angle between the straight line connecting start and end point and 
        // the line connecting the start/end point with the corresponding Bezier 
        // control point.
        double alpha = .5;

        // Distance between startPt and endPt
        double dist = getBaselineLength();
        double tangentLength = dist * .5;
        computeCtrlPt(alpha, tangentLength);

    }

    /**
     * Construct a QuadraticBezierFlow
     *
     * @param startPt Start point
     * @param endPt End point
     * @param alpha Angle around the point between the start point and the end
     * point, relative to the normal on the line connecting start point and end
     * point. 0 is perpendicular to this line. +/-PI/2 or is on the line.
     * @param distPerc The distance of the control point to the point between
     * the start and the end point (percentage).
     * @param value Value for line width.
     */
    public QuadraticBezierFlow(Point startPt, Point endPt, double alpha, int distPerc, double value) {
        this.startPt = startPt;
        this.endPt = endPt;
        this.value = value;
        cPt = new Point(0, 0);
        bend(alpha, distPerc);

    }

    /**
     * Bend flow
     *
     * @param alpha Angle relative to perpendicular line on base line.
     * @param distPerc Distance from base point.
     */
    @Override
    public final void bend(double alpha, int distPerc) {
        double dist = getBaselineLength() * distPerc / 100d;
        double beta = getBaselineAzimuth();
        double dx = dist * Math.cos(Math.PI / 2 - alpha + beta);
        double dy = dist * Math.sin(Math.PI / 2 - alpha + beta);
        cPt.x = (startPt.x + endPt.x) / 2 + dx;
        cPt.y = (startPt.y + endPt.y) / 2 + dy;
    }

    /**
     * Compute first control point from orientation of base line
     *
     * @param alpha angle between the base line and the line connecting the
     * start point with the first control point.
     * @param dist Distance between start point and first control point.
     */
    private void computeCtrlPt(double alpha, double dist) {
        final double lineOrientation = getBaselineAzimuth();
        final double azimuth = lineOrientation + alpha;
        final double dx1 = Math.sin(azimuth) * dist;
        final double dy1 = Math.cos(azimuth) * dist;
        double cPt1X = startPt.x + dx1;
        double cPt1Y = startPt.y + dy1;
        cPt = new Point(cPt1X, cPt1Y);
    }

    /**
     * Returns a bounding box, which is usually larger than the actual curve.
     * Does not take the line width into account.
     *
     * @return Bounding box.
     */
    @Override
    public Rectangle2D.Double getBoundingBox() {
        // Bezier curve is guaranteed to be within the convex hull defined by 
        // the four irregularPoints.
        Rectangle2D.Double bb = new Rectangle2D.Double(startPt.x, startPt.y, 0, 0);
        bb.add(endPt.x, endPt.y);
        bb.add(cPt.x, cPt.y);
        return bb;
    }

    /**
     * Returns the control point.
     *
     * @return the control point
     */
    public Point getCtrlPt() {
        return cPt;
    }

    /**
     * Sets the control point.
     *
     * @param cPt the control point to set
     */
    public void setcPt(Point cPt) {
        this.cPt = cPt;
    }

    public ArrayList<Point> toStraightLineSegmentsWithIrregularLength(double flatness) {
        assert (flatness > 0);

        // FIXME d should be a parameter
        double d = flatness * 100;

        ArrayList<Point> irregularPoints = new ArrayList<>();
        GeneralPath path = new GeneralPath();
        path.moveTo(startPt.x, startPt.y);
        path.quadTo(cPt.x, cPt.y, endPt.x, endPt.y);
        PathIterator iter = path.getPathIterator(null, flatness);
        double[] coords = new double[6];
        while (!iter.isDone()) {
            iter.currentSegment(coords);
            irregularPoints.add(new Point(coords[0], coords[1]));
            iter.next();
        }

        return irregularPoints;
    }

    /**
     * Converts this Bezier curve to straight line segments.
     *
     * @param flatness The maximum distance between the curve and the straight
     * line segments.
     * @return An list of irregularPoints, including copies of the start point
     * and the end point.
     */
    @Override
    public ArrayList<Point> toStraightLineSegments(double flatness) {
        assert (flatness > 0);

        // FIXME d should be a parameter
        double d = flatness * 100;

        ArrayList<Point> regularPoints = new ArrayList<>();
        ArrayList<Point> irregularPoints = toStraightLineSegmentsWithIrregularLength(flatness);

        // create new point set with regularly distributed irregularPoints
        double startX = irregularPoints.get(0).x;
        double startY = irregularPoints.get(0).y;

        // add start point
        regularPoints.add(new Point(startX, startY));

        double length = 0;
        int nPoints = irregularPoints.size();
        for (int i = 0; i < nPoints; i++) {
            Point inputPt = irregularPoints.get(i);
            double endX = inputPt.x;
            double endY = inputPt.y;

            // normalized direction dx and dy
            double dx = endX - startX;
            double dy = endY - startY;
            final double l = Math.sqrt(dx * dx + dy * dy);
            dx /= l;
            dy /= l;

            double rest = length;
            length += l;
            while (length >= d) {
                // compute new point
                length -= d;
                startX += dx * (d - rest);
                startY += dy * (d - rest);
                rest = 0;
                regularPoints.add(new Point(startX, startY));
            }
            startX = endX;
            startY = endY;
        }

        return regularPoints;
    }

    // FIXME
    public static QuadraticBezierFlow bendQuadraticFlow(Flow flow, int angleDeg, int distPerc) {

        // Convert angleDeg into radians
        double radians = angleDeg * (Math.PI / 180);

        //get the start and end irregularPoints
        Point startPt = flow.getStartPt();
        Point endPt = flow.getEndPt();
        double value = flow.getValue();

        return new QuadraticBezierFlow(startPt, endPt, radians, distPerc, value);
    }

    /**
     * Returns
     *
     * @param t
     * @return
     */
    public Point pointOnCurve(double t) {
        assert (t >= 0d && t <= 1d);

        double t2 = t * t;
        double mt = 1 - t;
        double mt2 = mt * mt;
        double x = startPt.x * mt2 + cPt.x * 2 * mt * t + endPt.x * t2;
        double y = startPt.y * mt2 + cPt.y * 2 * mt * t + endPt.y * t2;
        return new Point(x, y);
    }

    /**
     * Split a flow into two new flows.
     * http://pomax.github.io/bezierinfo/#matrixsplit
     *
     * @param t Parametric position [0..1]
     * @return
     */
    public QuadraticBezierFlow[] split(double t) {

        double startX1 = startPt.x;
        double startY1 = startPt.y;
        double ctrlX1 = t * cPt.x - (t - 1) * startPt.x;
        double ctrlY1 = t * cPt.y - (t - 1) * startPt.y;
        double endX1 = t * t * endPt.x - 2 * t * (t - 1) * cPt.x + (t - 1) * (t - 1) * startPt.x;
        double endY1 = t * t * endPt.y - 2 * t * (t - 1) * cPt.y + (t - 1) * (t - 1) * startPt.y;

        Point start1 = new Point(startX1, startY1);
        Point ctrl1 = new Point(ctrlX1, ctrlY1);
        Point end1 = new Point(endX1, endY1);

        double startX2 = t * t * endPt.x - 2 * t * (t - 1) * cPt.x + (t - 1) * (t - 1) * startPt.x;
        double startY2 = t * t * endPt.y - 2 * t * (t - 1) * cPt.y + (t - 1) * (t - 1) * startPt.y;
        double ctrlX2 = t * endPt.x - (t - 1) * cPt.x;
        double ctrlY2 = t * endPt.y - (t - 1) * cPt.y;
        double endX2 = endPt.x;
        double endY2 = endPt.y;

        Point start2 = new Point(startX2, startY2);
        Point ctrl2 = new Point(ctrlX2, ctrlY2);
        Point end2 = new Point(endX2, endY2);

        QuadraticBezierFlow flow1 = new QuadraticBezierFlow(start1, ctrl1, end1);
        flow1.value = value;

        QuadraticBezierFlow flow2 = new QuadraticBezierFlow(start2, ctrl2, end2);
        flow2.value = value;

        return new QuadraticBezierFlow[]{flow1, flow2};
    }

    /**
     * Returns the curve parameter where a circle with radius r around the end
     * point intersects the BŽzier curve.
     *
     * @param r Radius of circle
     * @return t parameter where the circle intersects the flow.
     */
    public double getIntersectionTWithCircleAroundEndPoint(double r) {
        if (r <= 0) {
            return 1;   // t = 1: end of curve
        }
        double t = 0.5;
        double t_step = 0.25;
        for (int i = 0; i < 20; i++) {
            Point pt = pointOnCurve(t);
            final double dx = endPt.x - pt.x;
            final double dy = endPt.y - pt.y;
            final double d = Math.sqrt(dx * dx + dy * dy);
            if (d < r) {
                t -= t_step;
            } else {
                t += t_step;
            }
            t_step /= 2;
        }

        return t;
    }

    /**
     * Returns the curve parameter where a circle with radius r around the start
     * point intersects the BŽzier curve.
     *
     * @param r Radius of circle
     * @return t parameter where the circle intersects the flow.
     */
    public double getIntersectionTWithCircleAroundStartPoint(double r) {
        if (r <= 0) {
            return 0;   // t = 0: start of curve
        }
        double t = 0.5;
        double t_step = 0.25;
        for (int i = 0; i < 20; i++) {
            Point pt = pointOnCurve(t);
            final double dx = startPt.x - pt.x;
            final double dy = startPt.y - pt.y;
            final double d = Math.sqrt(dx * dx + dy * dy);
            if (d < r) {
                t += t_step;
            } else {
                t -= t_step;
            }
            t_step /= 2;
        }

        return t;
    }

    /**
     * Returns a flow with the the start or end masking areas
     * removed.

     * @return A new flow object (if something was clipped), or this object.
     */
    public QuadraticBezierFlow getClippedFlow() {

        boolean clipWithStartArea = getStartClipArea() != null;
        boolean clipWithEndArea = getEndClipArea() != null;

        if (clipWithStartArea == false && clipWithEndArea == false) {
            return this;
        }
        
        // construct LineString from the current Bezier flow geometry
        // FIXME adapt de Casteljau tolerance
        ArrayList<Point> points = toStraightLineSegments(0.01);
        GeometryFactory geometryFactory = new GeometryFactory();
        int numPoints = points.size();
        Coordinate[] xy = new Coordinate[numPoints];
        for (int i = 0; i < numPoints; i++) {
            Point point = points.get(i);
            xy[i] = new Coordinate(point.x, point.y);
        }
        LineString lineString = geometryFactory.createLineString(xy);

        QuadraticBezierFlow splitFlow = this;

        // clip start
        if (clipWithStartArea && getStartClipArea() != null) {
            double d = 0;
            Geometry clippedGeometry = lineString.difference(getStartClipArea());
            Iterator geomi = new GeometryCollectionIterator(clippedGeometry);
            while (geomi.hasNext()) {
                Geometry geometry = (Geometry) geomi.next();
                if (geometry instanceof LineString) {
                    lineString = (LineString) geometry;
                    com.vividsolutions.jts.geom.Point pt = lineString.getStartPoint();
                    if (lineString.getNumPoints() >= 2) {
                        double dx = startPt.x - pt.getX();
                        double dy = startPt.y - pt.getY();
                        double dist = Math.sqrt(dx * dx + dy * dy);
                        if (dist > d) {
                            d = dist;
                        }
                    }
                }
            }
            double t = splitFlow.getIntersectionTWithCircleAroundStartPoint(d);
            splitFlow = split(t)[1];
        }
        
        // clip end
        if (clipWithEndArea && getEndClipArea() != null) {
            double d = 0;
            Geometry clippedGeometry = lineString.difference(getEndClipArea());
            Iterator geomi = new GeometryCollectionIterator(clippedGeometry);
            while (geomi.hasNext()) {
                Geometry geometry = (Geometry) geomi.next();
                if (geometry instanceof LineString) {
                    lineString = (LineString) geometry;
                    com.vividsolutions.jts.geom.Point pt = lineString.getEndPoint();
                    if (lineString.getNumPoints() >= 2) {
                        double dx = endPt.x - pt.getX();
                        double dy = endPt.y - pt.getY();
                        double endDistance = Math.sqrt(dx * dx + dy * dy);
                        if (endDistance > d) {
                            d = endDistance;
                        }
                    }
                }
            }
            double t = splitFlow.getIntersectionTWithCircleAroundEndPoint(d);
            splitFlow = split(t)[0];
        }

        return splitFlow;
    }
}
