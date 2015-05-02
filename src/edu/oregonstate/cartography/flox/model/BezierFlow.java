package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 * @author Dan Stephen, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class BezierFlow extends Flow {

    /**
     * First control point.
     */
    private Point cPt1;

    /**
     * Second control point.
     */
    private Point cPt2;

    /**
     * Construct a BezierFlow.
     *
     * @param startX Start of flow, x
     * @param startY Start of flow, y
     * @param c1x First control point, x
     * @param c1y First control point, y
     * @param c2x Second control point, x
     * @param c2y Second control point, y
     * @param endX End point, x
     * @param endY End point, y
     */
    public BezierFlow(double startX, double startY, double c1x, double c1y,
            double c2x, double c2y, double endX, double endY) {
        this.startPt = new Point(startX, startY);
        cPt1 = new Point(c1x, c1y);
        cPt2 = new Point(c2x, c2y);
        this.endPt = new Point(endX, endY);
    }

    /**
     * Construct a simple BezierFlow from 2 Point objects
     *
     * @param startPt Start point
     * @param endPt End point
     */
    public BezierFlow(Point startPt, Point endPt) {

        this.startPt = startPt;
        this.endPt = endPt;

        // Angle between the straight line connecting start and end point and 
        // the line connecting the start/end point with the corresponding Bezier 
        // control point.
        double alpha = .5;

        // Distance between startPt and endPt
        double dist = getBaselineLength();
        double tangentLength = dist * .33;
        computeStartCtrlPt(alpha, tangentLength);
        computeEndCtrlPt(alpha, tangentLength);
    }

    /**
     * Construct a BezierFlow from 2 Point objects, a tangent angle, a tangent
     * length, and a value
     *
     * @param startPt Start point of line
     * @param endPt End point of line
     * @param alpha Angle (in radians) between a line drawn from startPt to
     * endPt, and the line drawn to the control point.
     * @param distPerc A percentage of the distance from startPt to endPt
     * @param value Value for changing flow width
     * @value Flow volume, determines width of flow
     */
    public BezierFlow(Point startPt, Point endPt, double alpha, int distPerc, double value) {
        this.startPt = startPt;
        this.endPt = endPt;
        this.value = value;
        double dist = getBaselineLength();
        double tangentLength = dist * distPerc / 100d;
        computeStartCtrlPt(alpha, tangentLength);
        computeEndCtrlPt(alpha, tangentLength);
    }

    /**
     * Compute first control point from orientation of base line
     *
     * @param alpha angle between the base line and the line connecting the
     * start point with the first control point.
     * @param dist Distance between start point and first control point.
     */
    private void computeStartCtrlPt(double alpha, double dist) {
        final double lineOrientation = getBaselineAzimuth();
        final double azimuth = lineOrientation + alpha;
        final double dx1 = Math.sin(azimuth) * dist;
        final double dy1 = Math.cos(azimuth) * dist;
        double cPt1X = startPt.x + dx1;
        double cPt1Y = startPt.y + dy1;
        cPt1 = new Point(cPt1X, cPt1Y);
    }

    /**
     * Compute second control point from orientation of base line
     *
     * @param alpha angle between the base line and the line connecting the end
     * point with the second control point.
     * @param dist Distance between end point and second control point.
     */
    private void computeEndCtrlPt(double alpha, double dist) {
        final double lineOrientation = getBaselineAzimuth();
        final double azimuth = lineOrientation + Math.PI - alpha;
        final double dx2 = Math.sin(azimuth) * dist;
        final double dy2 = Math.cos(azimuth) * dist;
        double cPt2X = endPt.x + dx2;
        double cPt2Y = endPt.y + dy2;
        cPt2 = new Point(cPt2X, cPt2Y);
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
        // the four points.
        Rectangle2D.Double bb = new Rectangle2D.Double(startPt.x, startPt.y, 0, 0);
        bb.add(endPt.x, endPt.y);
        bb.add(cPt1.x, cPt1.y);
        bb.add(cPt2.x, cPt2.y);
        return bb;
    }

    /**
     * Returns the first control point.
     *
     * @return the cPt1
     */
    public Point getcPt1() {
        return cPt1;
    }

    /**
     * Sets the first control point.
     *
     * @param cPt1 the cPt1 to set
     */
    public void setcPt1(Point cPt1) {
        this.cPt1 = cPt1;
    }

    /**
     * Returns the second control point.
     *
     * @return the cPt2
     */
    public Point getcPt2() {
        return cPt2;
    }

    /**
     * Set the second control point.
     *
     * @param cPt2 the cPt2 to set
     */
    public void setcPt2(Point cPt2) {
        this.cPt2 = cPt2;
    }

}
