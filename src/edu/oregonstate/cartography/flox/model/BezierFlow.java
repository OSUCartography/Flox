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

        this.startPt = startPt;
        this.endPt = endPt;
        
        // Angle between the straight line connecting start and end point and 
        // the line connecting the start/end point with the corresponding Bezier 
        // control point.
        double alpha = .5;
        
        // Distance between startPt and endPt
        double x1 = startPt.x;
        double y1 = startPt.y;
        double x2 = endPt.x;
        double y2 = endPt.y;
        double dist = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

        double shortDist = dist * .33;
        
        cPt1 = computeStartCtrlPt(alpha, shortDist);
        cPt2 = computeEndCtrlPt(alpha, shortDist);
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
    private Point computeStartCtrlPt(double alpha, double dist) {
        final double lineOrientation = getBaselineAzimuth();
        final double azimuth1 = lineOrientation + alpha;
        final double dx1 = Math.sin(azimuth1) * dist;
        final double dy1 = Math.cos(azimuth1) * dist;
        
        double cPt1X = startPt.x + dx1;
        double cPt1Y = startPt.y + dy1;
        
        Point startCtrlPt = new Point(cPt1X, cPt1Y);
        return startCtrlPt;
    }

    private Point computeEndCtrlPt(double alpha, double dist) {
        final double lineOrientation = getBaselineAzimuth();
        final double azimuth2 = lineOrientation + Math.PI - alpha;
        final double dx2 = Math.sin(azimuth2) * dist;
        final double dy2 = Math.cos(azimuth2) * dist;
        
        double cPt2X = endPt.x + dx2;
        double cPt2Y = endPt.y + dy2;
        
        Point endCtrlPt = new Point(cPt2X, cPt2Y);
        return endCtrlPt;
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
