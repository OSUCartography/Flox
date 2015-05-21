package edu.oregonstate.cartography.flox.model;

/**
 * Two-dimensional point.
 * 
 * @author Bernhard Jenny and Dan Stephen, Cartography and Geovisualization 
 * Group, Oregon State University
 */
public final class Point {
    public double x;
    public double y;
    
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    /**
     * Rotates this point by the provided angle around an origin point and returns
     * the rotated point. The position of this point is not changed.
     * @param origin Pivot around which the point will be rotated
     * @param angle Rotation angle in radians. Positive numbers will rotate counter-
     * clockwise, negative numbers will rotate clockwise. 
     * @return 
     */
    public Point rotatePoint(Point origin, double angle) {
        double tempX = x - origin.x;
        double tempY = y - origin.y;
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double newX = tempX * cos - tempY * sin;
        double newY = tempX * sin + tempY * cos;
        return new Point(newX + origin.x, newY + origin.y);
    }
    
}
