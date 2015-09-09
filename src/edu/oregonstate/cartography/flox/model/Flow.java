package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Geometry;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public abstract class Flow {

    /**
     * start point of flow.
     */
    protected Point startPt;

    /**
     * end point of flow.
     */
    protected Point endPt;

    /**
     * mapped value.
     */
    private double value;
    
    /**
     * clip area for the start of the flow.
     */
    private Geometry startClipArea;
    
    /**
     * clip area for the end of the flow.
     */
    private Geometry endClipArea;

    /**
     * Flag for selected flows.
     */
    private boolean selected = false;
    
    /**
     * Flag for locked flows. Locked flows are not affected by forces, but still
     * emit forces onto other flows. 
     */
    private boolean locked = false;
    
    /**
     * Return bounding box of this flow.
     *
     * @return
     */
    public abstract Rectangle2D.Double getBoundingBox();
    
    /**
     * Apply bending to flow
     * @param alpha Angle parameter
     * @param distPerc Distance parameter
     */
    public abstract void bend(double alpha, int distPerc);

    /**
     * Returns the start point of the flow.
     *
     * @return the startPt
     */
    public Point getStartPt() {
        return startPt;
    }

    /**
     * Set the start point of the flow.
     *
     * @param startPt the startPt to set
     */
    public void setStartPt(Point startPt) {
        this.startPt = startPt;
    }

    /**
     * Returns the end point of the flow.
     *
     * @return the endPt
     */
    public Point getEndPt() {
        return endPt;
    }

    /**
     * Set the end point of the flow.
     *
     * @param endPt the endPt to set
     */
    public void setEndPt(Point endPt) {
        this.endPt = endPt;
    }

    /**
     * Returns the distance between start and end point
     * @return 
     */
    public double getBaselineLength() {
        double dx = startPt.x - endPt.x;
        double dy = startPt.y - endPt.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
   
    /**
     * Returns the azimuthal angle for a line between a start and end point
     *
     * @return Angle in radians, counter-clockwise, 0 is pointing eastwards
     */
    public double getBaselineAzimuth() {
        final double dx = endPt.x - startPt.x;
        final double dy = endPt.y - startPt.y;
        return Math.atan2(dy, dx);
    }

    public Point getBaseLineMidPoint() {
        return new Point((endPt.x + startPt.x) / 2, (endPt.y + startPt.y) / 2);
    }
    
    public void reverseFlow() {
        Point temp = startPt;
        startPt = endPt;
        endPt = temp;
    }
    
    /**
     * @return the flow value
     */
    public double getValue() {
        return value;
    }

    /**
     * @param value the value to set
     */
    public void setValue(double value) {
        this.value = value;
    }
    
    /**
    * Converts this Bezier curve to straight line segments.
    * @param flatness The maximum distance between the curve and the straight
    * line segments.
    * @return An list of points, including copies of the start point and the end point.
    */
    public abstract ArrayList<Point> toStraightLineSegments(double flatness);

    /**
     * Returns the location on the BŽzier curve at parameter value t.
     *
     * @param t Parameter [0..1]
     * @return Location on curve.
     */
    public abstract Point pointOnCurve(double t);
    
    /**
     * @return the startClipArea
     */
    public Geometry getStartClipArea() {
        return startClipArea;
    }

    /**
     * @param startClipArea the startClipArea to set
     */
    public void setStartClipArea(Geometry startClipArea) {
        this.startClipArea = startClipArea;
    }

    /**
     * @return the endClipArea
     */
    public Geometry getEndClipArea() {
        return endClipArea;
    }

    /**
     * @param endClipArea the endClipArea to set
     */
    public void setEndClipArea(Geometry endClipArea) {
        this.endClipArea = endClipArea;
    }

    /**
     * @return the selected
     */
    public boolean isSelected() {
        return selected;
    }

    /**
     * @param selected the selected to set
     */
    public void setSelected(boolean selected) {
        this.selected = selected;
    }

    /**
     * @return the locked
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * @param locked the locked to set
     */
    public void setLocked(boolean locked) {
        this.locked = locked;
    }
}
