package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class BezierFlow extends Flow {

    private Point cPt1;
    private Point cPt2;

    public BezierFlow(double startX, double startY, double c1x, double c1y,
            double c2x, double c2y, double endX, double endY) {
        this.startPt = new Point(startX, startY);
        cPt1 = new Point(c1x, c1y);
        cPt2 = new Point(c2x, c2y);
        this.endPt = new Point(endX, endY);
    }

    /**
     * Construct a simple BesierFlow from 2 Point objects
     *
     * @param startPt
     * @param endPt
     */
    public BezierFlow(Point startPt, Point endPt) {

        double x1 = startPt.x;
        double x2 = endPt.x;
        double y1 = startPt.y;
        double y2 = endPt.y;

        // Angle between the straight line connecting start and end point and 
        // the line connecting the start/end point with the corresponding Bezier 
        // control point.
        double alpha = 0.3;
        
        // Distance between startPt and endPt
        double dist = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

        
    }

    /**
     * Computes the azimuthal angle for a line between a start and end point
     * @return Angle in radians
     */
    private double getBaselineAzimuth() {
        final double dx = endPt.x- startPt.x;
        final double dy = endPt.y - startPt.y;
        return Math.atan2(dx, dy);
    }

    // Need the distance between the points
    private void computeStartCtrlPt(double alpha, double dist) {
        final double lineOrientation = getBaselineAzimuth();
        final double azimuth1 = lineOrientation + alpha;
        final double dx1 = Math.sin(azimuth1) * dist;
        final double dy1 = Math.cos(azimuth1) * dist;
        //this.cPt1 = this. + dx1;
    }

    @Override
    public Rectangle2D.Double getBoundingBox() {
        Rectangle2D.Double bb = new Rectangle2D.Double(startPt.x, startPt.y, 0, 0);
        bb.add(endPt.x, endPt.y);
        bb.add(cPt1.x, cPt1.y);
        bb.add(cPt2.x, cPt2.y);
        return bb;
    }

    /**
     * @return the cPt1
     */
    public Point getcPt1() {
        return cPt1;
    }

    /**
     * @param cPt1 the cPt1 to set
     */
    public void setcPt1(Point cPt1) {
        this.cPt1 = cPt1;
    }

    /**
     * @return the cPt2
     */
    public Point getcPt2() {
        return cPt2;
    }

    /**
     * @param cPt2 the cPt2 to set
     */
    public void setcPt2(Point cPt2) {
        this.cPt2 = cPt2;
    }

}
