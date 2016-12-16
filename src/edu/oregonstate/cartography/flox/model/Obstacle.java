package edu.oregonstate.cartography.flox.model;

/**
 * An obstacle is a circle covering a node or an arrowhead. Flows should not
 * overlap obstacles.
 *
 * @author Bernhard Jenny, School of Science, RMIT University, Melbourne
 */
public final class Obstacle extends Circle {

    /**
     * The node associated with this obstacle. Can be null if the obstacle is an
     * arrowhead.
     */
    public final Point node;

    /**
     * If this obstacle is for an arrowhead, the arrowhead is for this flow.
     * Null if this obstacle is a node.
     */
    public final Flow flow;

    /**
     * Construct new Obstacle for a node.
     *
     * @param node node associated with this obstacle.
     * @param x center x
     * @param y center y
     * @param r radius of obstacle circle
     */
    public Obstacle(Point node, double x, double y, double r) {
        super(x, y, r);
        this.node = node;
        this.flow = null;
    }

    /**
     * Returns whether this obstacle is for an arrowhead.
     *
     * @return true if for arrowhead, false if for a node.
     */
    public boolean isArrowObstacle() {
        return flow != null;
    }

    /**
     * Returns whether this obstacle is the arrowhead for the passed flow.
     *
     * @param flow flow with an arrowhead
     * @return true if this is an obstacle for the passed flow, false otherwise.
     */
    public boolean isArrowObstacleForFlow(Flow flow) {
        return this.flow != null && this.flow == flow;
    }

    /**
     * Construct an obstacle from three points that are on the obstacle circle.
     *
     * @param p1 point 1 on obstacle circle
     * @param p2 point 2 on obstacle circle
     * @param p3 point 3 on obstacle circle
     * @param flow
     */
    public Obstacle(Point p1, Point p2, Point p3, Flow flow) {
        super(p1, p2, p3);
        this.node = null;
        this.flow = flow;
    }

}
