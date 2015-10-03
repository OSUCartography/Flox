package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollectionIterator;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import edu.oregonstate.cartography.utils.GeometryUtils;
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
        double tangentLength = dist * 0.5;
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
        this.setValue(value);
        cPt = new Point(0, 0);
        bend(alpha, distPerc);
    }

    public void straighten() {
        bend(0, 0);
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
     * Returns a bounding box containing the curve. The control point can
     * be outside of the bounding box returned by this method.
     * Does not take any line width into account.
     * Based on http://pomax.github.io/bezierinfo/#boundingbox
     *
     * @return Bounding box.
     */
    @Override
    public Rectangle2D.Double getBoundingBox() {
        // initialize bounding box with start and end points
        double xmin, xmax, ymin, ymax;
        if (startPt.x > endPt.x) {
            xmin = endPt.x;
            xmax = startPt.x;
        } else {
            xmin = startPt.x;
            xmax = endPt.x;
        }
        if (startPt.y > endPt.y) {
            ymin = endPt.y;
            ymax = startPt.y;
        } else {
            ymin = startPt.y;
            ymax = endPt.y;
        }
        
        // Compute parameter t for the root of the first derivative of the x 
        // position. This is the t parameter for the extremum in x of the curve, 
        // as the first derivative is 0 at the extremum.
        double tx = (startPt.x - cPt.x) / (startPt.x - 2 * cPt.x + endPt.x);
        // t must be in [0,1]
        if (Double.isFinite(tx) && tx >= 0 && tx <= 1) {
            double one_minus_tx = 1d - tx;
            // compute x position of extrema
            double x = one_minus_tx * one_minus_tx * startPt.x
                    + 2 * one_minus_tx * tx * cPt.x + tx * tx * endPt.x;
            // extend bounding box
            xmin = Math.min(xmin, x);
            xmax = Math.max(xmax, x);
        }
        
        // repeat for y
        double ty = (startPt.y - cPt.y) / (startPt.y - 2 * cPt.y + endPt.y);
        if (Double.isFinite(ty) && ty >= 0 && ty <= 1) {
            double one_minus_ty = 1d - ty;
            double y = one_minus_ty * one_minus_ty * startPt.y
                    + 2 * one_minus_ty * ty * cPt.y + ty * ty * endPt.y;
            ymin = Math.min(ymin, y);
            ymax = Math.max(ymax, y);
        }
        
        return new Rectangle2D.Double(xmin, ymin, xmax - xmin, ymax - ymin);
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
    public void setControlPoint(Point cPt) {
        this.cPt = cPt;
    }

    public ArrayList<Point> toStraightLineSegmentsWithIrregularLength(
            double deCasteljauTol) {
        assert (deCasteljauTol > 0);

        // FIXME d should be a parameter
        double d = deCasteljauTol;

        ArrayList<Point> irregularPoints = new ArrayList<>();
        GeneralPath path = new GeneralPath();
        path.moveTo(startPt.x, startPt.y);
        path.quadTo(cPt.x, cPt.y, endPt.x, endPt.y);
        PathIterator iter = path.getPathIterator(null, deCasteljauTol / 100);
        double[] coords = new double[6];
        while (!iter.isDone()) {
            iter.currentSegment(coords);
            irregularPoints.add(new Point(coords[0], coords[1]));
            iter.next();
        }

        return irregularPoints;
    }

    private LineString pointsToLineString(ArrayList<Point> points) {
        // construct LineString from the current Bezier flow geometry
        GeometryFactory geometryFactory = new GeometryFactory();
        int numPoints = points.size();
        Coordinate[] xy = new Coordinate[numPoints];
        for (int i = 0; i < numPoints; i++) {
            Point point = points.get(i);
            xy[i] = new Coordinate(point.x, point.y);
        }

        return geometryFactory.createLineString(xy);
    }

    @Override
    public ArrayList<Point> toStraightLineSegments(double deCasteljauTol) {
        QuadraticBezierFlow clippedFlow = getClippedFlow(deCasteljauTol);
        return clippedFlow.toUnclippedStraightLineSegments(deCasteljauTol);
    }

    /**
     * Converts this Bezier curve to straight line segments.
     *
     * @param deCasteljauTol The maximum distance between the curve and the
     * straight line segments.
     * @return An list of irregularPoints, including copies of the start point
     * and the end point.
     */
    public ArrayList<Point> toUnclippedStraightLineSegments(double deCasteljauTol) {
        assert (deCasteljauTol > 0);

        // FIXME d should be a parameter
        double d = deCasteljauTol;

        ArrayList<Point> regularPoints = new ArrayList<>();
        ArrayList<Point> irregularPoints
                = toStraightLineSegmentsWithIrregularLength(d);

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

        // add end point
        regularPoints.add(irregularPoints.get(irregularPoints.size() - 1));
        return regularPoints;
    }

    /**
     * Returns the location on the BŽzier curve at parameter value tx.
     *
     * @param t Parameter [0..1]
     * @return Location on curve.
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
     * @return Two new flows if tx is > 0 and tx < 1. Otherwise two references
     * to this.
     */
    public QuadraticBezierFlow[] split(double t) {
        if (t <= 0 || t >= 1) {
            return new QuadraticBezierFlow[]{this, this};
        }

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
        flow1.setValue(this.getValue());

        QuadraticBezierFlow flow2 = new QuadraticBezierFlow(start2, ctrl2, end2);
        flow2.setValue(this.getValue());

        return new QuadraticBezierFlow[]{flow1, flow2};
    }

    /**
     * Returns the curve parameter where a circle with radius r around the end
     * point intersects the BŽzier curve.
     *
     * @param r Radius of circle
     * @return tx parameter where the circle intersects the flow.
     */
    public double getIntersectionTWithCircleAroundEndPoint(double r) {
        if (r <= 0) {
            return 1;   // tx = 1: end of curve
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
     * @return tx parameter where the circle intersects the flow.
     */
    public double getIntersectionTWithCircleAroundStartPoint(double r) {
        if (r <= 0) {
            return 0;   // tx = 0: start of curve
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
     * Returns a flow with the the start and/or end masking areas removed.
     *
     * @param deCasteljauTol Tolerance for conversion to straight line segments.
     * @return A new flow object (if something was clipped), or this object.
     */
    public QuadraticBezierFlow getClippedFlow(double deCasteljauTol) {

        boolean clipWithStartArea = getStartClipArea() != null;
        boolean clipWithEndArea = getEndClipArea() != null;

        if (clipWithStartArea == false && clipWithEndArea == false) {
            return this;
        }

        // construct LineString from the current Bezier flow geometry
        ArrayList<Point> points = toUnclippedStraightLineSegments(deCasteljauTol);
        LineString lineString = pointsToLineString(points);
        QuadraticBezierFlow splitFlow = this;

        // clip linestring with clip areas around start point
        if (clipWithStartArea) {
            Geometry clippedFlowLineGeometry = lineString.difference(getStartClipArea());
            double d = 0;
            Iterator geomi = new GeometryCollectionIterator(clippedFlowLineGeometry);
            while (geomi.hasNext()) {
                Geometry geometry = (Geometry) geomi.next();
                if (geometry instanceof LineString) {
                    LineString l = (LineString) geometry;
                    com.vividsolutions.jts.geom.Point pt = l.getStartPoint();
                    if (l.getNumPoints() >= 2) {
                        double dx = startPt.x - pt.getX();
                        double dy = startPt.y - pt.getY();
                        double dist = Math.sqrt(dx * dx + dy * dy);
                        if (dist > d) {
                            d = dist;
                        }
                    }
                }
            }
            double startT = splitFlow.getIntersectionTWithCircleAroundStartPoint(d);
            splitFlow = split(startT)[1];
        }

        // clip linestring with clip areas around end point
        if (clipWithEndArea) {
            Geometry clippedFlowLineGeometry = lineString.difference(getEndClipArea());
            double d = 0;
            Iterator geomi = new GeometryCollectionIterator(clippedFlowLineGeometry);
            while (geomi.hasNext()) {
                Geometry geometry = (Geometry) geomi.next();
                if (geometry instanceof LineString) {
                    LineString l = (LineString) geometry;
                    com.vividsolutions.jts.geom.Point pt = l.getEndPoint();
                    if (l.getNumPoints() >= 2) {
                        double dx = endPt.x - pt.getX();
                        double dy = endPt.y - pt.getY();
                        double dist = Math.sqrt(dx * dx + dy * dy);
                        if (dist > d) {
                            d = dist;
                        }
                    }
                }
            }
            double endT = splitFlow.getIntersectionTWithCircleAroundEndPoint(d);
            splitFlow = splitFlow.split(endT)[0];
        }

        return splitFlow;
    }

    /**
     * Computes the shortest distance between a point and any point on this
     * quadratic BŽzier curve. Attention: xy parameter is changed.
     *
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @return The distance.
     */
    public double distance(double[] xy) {
        return GeometryUtils.getDistanceToQuadraticBezierCurve(startPt.x, startPt.y,
                cPt.x, cPt.y, endPt.x, endPt.y, xy);
    }

    /**
     * Computes the square of the shortest distance between a point and any
     * point on this quadratic BŽzier curve. Attention: xy parameter is changed.
     *
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @return The distance.
     */
    public double distanceSq(double[] xy) {
        return GeometryUtils.getDistanceToQuadraticBezierCurveSq(startPt.x, startPt.y,
                cPt.x, cPt.y, endPt.x, endPt.y, xy);
    }

    public double getDistanceBetweenStartPointAndControlPoint() {
        double dx = cPt.x - startPt.x;
        double dy = cPt.y - startPt.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getDistanceBetweenEndPointAndControlPoint() {
        double dx = cPt.x - endPt.x;
        double dy = cPt.y - endPt.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double[] getDirectionVectorFromStartPointToControlPoint() {
        double dx = cPt.x - startPt.x;
        double dy = cPt.y - startPt.y;
        double d = Math.sqrt(dx * dx + dy * dy);
        return new double[]{dx / d, dy / d};
    }

    public double[] getDirectionVectorFromEndPointToControlPoint() {
        double dx = cPt.x - endPt.x;
        double dy = cPt.y - endPt.y;
        double d = Math.sqrt(dx * dx + dy * dy);
        return new double[]{dx / d, dy / d};
    }

    /**
     * Orientation of the line between the start point and the control point.
     *
     * @return Angle in radians relative to horizontal x axis in
     * counter-clockwise direction.
     */
    public double startToCtrlAngle() {
        double dx = cPt.x - startPt.x;
        double dy = cPt.y - startPt.y;
        return Math.atan2(dy, dx);
    }

    /**
     * Orientation of the line between the end point and the control point.
     *
     * @return Angle in radians relative to horizontal x axis in
     * counter-clockwise direction.
     */
    public double endToCtrlAngle() {
        double dx = cPt.x - endPt.x;
        double dy = cPt.y - endPt.y;
        return Math.atan2(dy, dx);
    }
}
