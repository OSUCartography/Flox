package edu.oregonstate.cartography.flox.model;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

/**
 * Two-dimensional point.
 *
 * @author Bernhard Jenny and Dan Stephen, Cartography and Geovisualization
 * Group, Oregon State University, and RMIT University, Melbourne
 */

//Every non static, non transient field in a JAXB-bound class will be 
//automatically bound to XML, unless annotated by @XmlTransient
@XmlAccessorType(XmlAccessType.FIELD)

public final class Point {

    public double x;
    public double y;

    private boolean selected = false;

    private double value;

    public Point(double x, double y) {
        assert(Double.isFinite(x));
        assert(Double.isFinite(y));
        
        this.x = x;
        this.y = y;
        this.value = 1;
    }
    
    public Point(double x, double y, double value) {
        assert(Double.isFinite(x));
        assert(Double.isFinite(y));
        assert(Double.isFinite(value));
        
        this.x = x;
        this.y = y;
        this.value = value;
    }
    
    public Point() {
        x = y = 0;
        value = Model.DEFAULT_NODE_VALUE;
    }
    
    /**
     * Rotates this point by the provided angle around an origin point and
     * returns the rotated point. The position of this point is not changed.
     *
     * @param origin Pivot around which the point will be rotated
     * @param angle Rotation angle in radians. Positive numbers will rotate
     * counter- clockwise, negative numbers will rotate clockwise.
     * @return
     */
    public Point rotatePoint(Point origin, double angle) {
        assert(Double.isFinite(angle));
        
        double tempX = x - origin.x;
        double tempY = y - origin.y;
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double newX = tempX * cos - tempY * sin;
        double newY = tempX * sin + tempY * cos;
        return new Point(newX + origin.x, newY + origin.y);
    }

    /**
     * Rotate and translate
     *
     * @param dx horizontal translation
     * @param dy vertical translation
     * @param angle Rotation angle in radians. Positive numbers will rotate
     * counter- clockwise, negative numbers will rotate clockwise.
     */
    public void transform(double dx, double dy, double angle) {
        assert(Double.isFinite(dx));
        assert(Double.isFinite(dy));
        assert(Double.isFinite(angle));
        
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double newX = x * cos - y * sin + dx;
        y = x * sin + y * cos + dy;
        x = newX;
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

    @Override
    public String toString() {
        return "[" + x + "/" + y + "]";
    }

    /**
     * @return the value
     */
    public double getValue() {
        return value;
    }

    /**
     * Change the point value. <STRONG>Must be followed by a call to
     * Graph.updateCachedValues().</STRONG>
     *
     * @param value the value to set
     */
    protected void setValue(double value) {
        assert(Double.isFinite(value));
        this.value = value;
    }

    /**
     * Distance to another point.
     *
     * @param p Another point.
     * @return The distance to the other point.
     */
    public double distance(Point p) {
        double dx = x - p.x;
        double dy = y - p.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
     * Distance to another point.
     *
     * @param x
     * @param y
     * @return The distance to the other point.
     */
    public double distance(double x, double y) {
        double dx = this.x - x;
        double dy = this.y - y;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
