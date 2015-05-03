package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;

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
     * Construct a simple BezierFlow from 2 Point objects
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
        double tangentLength = dist * .33;
        computeCtrlPt(alpha, tangentLength);
    }

    public QuadraticBezierFlow(Point startPt, Point endPt, double alpha, int distPerc, double value) {
        this.startPt = startPt;
        this.endPt = endPt;
        this.value = value;
        double dist = getBaselineLength();
        dist *= distPerc / 100d;
        double lineOrientation = getBaselineAzimuth();
        double azimuth = lineOrientation + alpha;
        double dx = Math.sin(azimuth) * dist;
        double dy = Math.cos(azimuth) * dist;
        cPt = new Point(startPt.x + dx, startPt.y + dy);
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
        // the four points.
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
    public Point getcPt() {
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

}
