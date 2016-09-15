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
        final double offset = Math.pow(p2.x, 2) + Math.pow(p2.y, 2);
        final double bc = (Math.pow(p1.x, 2) + Math.pow(p1.y, 2) - offset) / 2.0;
        final double cd = (offset - Math.pow(p3.x, 2) - Math.pow(p3.y, 2)) / 2.0;
        final double det = (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p2.y);

        if (Math.abs(det) < TOL) {
            throw new IllegalArgumentException("illegal circle");
        }

        final double idet = 1d / det;

        x = (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
        y = (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
        r = Math.sqrt(Math.pow(p2.x - x, 2) + Math.pow(p2.y - y, 2));
    }
}
