package edu.oregonstate.cartography.flox.model;

/**
 * Circle with center and radius.
 */
public class Circle {

    static public final double TOL = 0.000000001;

    public final double x;
    public final double y;
    public final double r;

    public Circle(double centerX, double centerY, double radius) {
        this.x = centerX;
        this.y = centerY;
        this.r = radius;
    }

    /**
     * Construct circle passing through three points.
     * http://stackoverflow.com/questions/4103405/what-is-the-algorithm-for-finding-the-center-of-a-circle-from-three-points
     *
     * @param p1
     * @param p2
     * @param p3
     */
    public Circle(final Point p1, final Point p2, final Point p3) {       
        double det = (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p2.y);
        if (Math.abs(det) < TOL) {
            throw new IllegalArgumentException("illegal circle");
        }

        double offset = p2.x * p2.x + p2.y * p2.y;
        double bc = (p1.x * p1.x + p1.y * p1.y - offset) / 2.0;
        double cd = (offset - p3.x * p3.x - p3.y * p3.y) / 2.0;
        x = (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) / det;
        y = (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) / det;
        double dx = p2.x - x;
        double dy = p2.y - y;
        r = Math.sqrt(dx * dx + dy * dy);
    }
}
