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
        assert (Double.isFinite(x));
        assert (Double.isFinite(y));

        this.x = x;
        this.y = y;
        this.value = 1;
    }

    public Point(double x, double y, double value) {
        assert (Double.isFinite(x));
        assert (Double.isFinite(y));
        assert (Double.isFinite(value));

        this.x = x;
        this.y = y;
        this.value = value;
    }

    public Point() {
        x = y = 0;
        value = Model.DEFAULT_NODE_VALUE;
    }

    /**
     * Copy constructor.
     *
     * @param point
     */
    public Point(Point point) {
        this.x = point.x;
        this.y = point.y;
        this.value = point.value;
        this.selected = point.selected;
    }

    /**
     * Rotate and translate. Rotation is applied first, then the point is
     * translated.
     *
     * @param dx horizontal translation
     * @param dy vertical translation
     * @param sin sin of rotation angle
     * @param cos cos of rotation angle
     */
    public void transform(double dx, double dy, double sin, double cos) {
        double newX = x * cos - y * sin + dx;
        y = x * sin + y * cos + dy;
        x = newX;
    }

    /**
     * Translate point.
     *
     * @param dx horizontal offset
     * @param dy vertical offset
     */
    public void offset(double dx, double dy) {
        assert (Double.isFinite(dx));
        assert (Double.isFinite(dy));

        x += dx;
        y += dy;
    }

    /**
     * Returns true if this point is on the left side of the line defined by
     * point a and point b when viewed from a towards b.
     *
     * @param a start point of line
     * @param b end point of line
     * @return true if this point is on the left side of the line; false if the
     * point is on the line or on the right side of the line
     */
    public boolean isLeft(Point a, Point b) {
        return ((b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x)) > 0;
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
        assert (Double.isFinite(value));
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

    /**
     * Square distance to another point.
     *
     * @param p another point.
     * @return the square distance to the other point.
     */
    public double distanceSquare(Point p) {
        double dx = x - p.x;
        double dy = y - p.y;
        return dx * dx + dy * dy;
    }

    /**
     * Square distance to another point.
     *
     * @param x x coordinate of another point.
     * @param y y coordinate of another point.
     * @return the square distance to the other point.
     */
    public double distanceSquare(double x, double y) {
        double dx = this.x - x;
        double dy = this.y - y;
        return dx * dx + dy * dy;
    }

    /**
     * Create a new point between this and a passed point. The value of the new
     * point is the mean of the value of this and the value of the passed point.
     *
     * @param point point
     * @return new point
     */
    public Point mean(Point point) {
        double ptx = (x + point.x) / 2;
        double pty = (y + point.y) / 2;
        double ptv = (value + point.value) / 2;
        return new Point(ptx, pty, ptv);
    }
}
