package edu.oregonstate.cartography.flox.model;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Point {
    public double x;
    public double y;
    
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    /**
     * Rotates this point by the provided angle around an origin point.
     * @param origin Pivot around which the point will be rotated
     * @param angle Degrees of rotation. Positive numbers will rotate counter-
     * clockwise, negative numbers will rotate clockwise
     * @return 
     */
    public Point rotatePoint(Point origin, double angle) {
        
        double radians = angle * (Math.PI / 180);
        
        double tempX = x - origin.x;
        double tempY = y - origin.y;
        
        double newX = (tempX * (Math.cos(radians))) - (tempY * (Math.sin(radians)));
        double newY = (tempX * (Math.sin(radians))) + (tempY * (Math.cos(radians)));
        
        return new Point(newX + origin.x, newY + origin.y);
    }
    
}
