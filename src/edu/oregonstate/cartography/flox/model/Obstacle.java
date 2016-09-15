package edu.oregonstate.cartography.flox.model;

/**
 *
 * @author Bernhard Jenny, School of Mathematical and Geospatial Sciences, RMIT
 * University, Melbourne
 */
public class Obstacle {
    
    public Obstacle(Point node, double x, double y, double r) {
        this.node = node;
        this.x = x;
        this.y = y;
        this.r = r;
    }
    /**
     * The node associated with this obstacle. The node can be the obstacle
     * itself, or it can be the target of an arrowhead obstacle.
     */
    public Point node;
    /**
     * x coordinate of center of obstacle circle in world coordinates.
     */
    public double x;
    /**
     * y coordinate of center of obstacle circle in world coordinates.
     */
    public double y;
    /**
     * radius of obstacle circle in world coordinates.
     */
    public double r;
    
}
