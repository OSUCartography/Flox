package edu.oregonstate.cartography.flox.model;

/**
 * An obstacle is a circle covering a node or an arrowhead. Flows should not
 * overlap obstacles.
 *
 * @author Bernhard Jenny, School of Science, RMIT University, Melbourne
 */
public final class Obstacle extends Circle {

    /**
     * The node associated with this obstacle. The node can be the obstacle
     * itself, or it can be the target of an arrowhead obstacle.
     */
    public final Point node;

    /**
     * Construct new Obstacle.
     *
     * @param node node associated with this obstacle.
     * @param x center x
     * @param y center y
     * @param r radius of obstacle circle
     */
    public Obstacle(Point node, double x, double y, double r) {
        super(x, y, r);
        this.node = node;
    }

    /**
     * Construct an obstacle from three points that are on the obstacle circle.
     *
     * @param node node associated with this obstacle.
     * @param p1 point 1 on obstacle circle
     * @param p2 point 2 on obstacle circle
     * @param p3 point 3 on obstacle circle
     */
    public Obstacle(Point node, Point p1, Point p2, Point p3) {
        super(p1, p2, p3);
        this.node = node;
    }

}
