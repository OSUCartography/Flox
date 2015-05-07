package edu.oregonstate.cartography.flox.model;

import java.util.Iterator;

/**
 * ForceLayouter contains the algorithms that compute the total force that 
 * each map feature emits and receives.
 * @author danielstephen
 */
public class ForceLayouter {

    private double maxFlowLengthSpringConstant = 0.5; // Spring stiffness of longest flow
    private double minFlowLengthSpringConstant = 0.5; // Spring stiffness of zero-length flow
    
    // This determines the amount of force that objects far away from the target
    // can apply to the target.  The lower the idwExponent, the more force distant
    // objects are permitted to apply.
    private double idwExponent = 4; // 
    
    // Stores the model, which contains all map features.
    private Model model;

    /**
     * Sets the spring constants. This is ultimately called by slider bars in the GUI.
     * @param maxFlowLengthSpringConstant The stiffness of the spring of the
     * longest flow
     * @param minFlowLengthSpringConstant The minimum spring stiffness of all
     * springs on the map.
     */
    public void setSpringConstants(double maxFlowLengthSpringConstant, double minFlowLengthSpringConstant) {
        this.maxFlowLengthSpringConstant = maxFlowLengthSpringConstant;
        this.minFlowLengthSpringConstant = minFlowLengthSpringConstant;
    }

    /**
     * Sets the idwExponent. This is currently set by the slider bar in the GUI.
     * @param idwExponent The idwExponent to set
     */
    public void setIDWExponent(double idwExponent) {
        this.idwExponent = idwExponent;
    }

    /**
     * Constructor for the ForceLayouter. Requires a Model object containing
     * flow map features.
     * @param model 
     */
    public ForceLayouter(Model model) {
        this.model = model;
    }

    /**
     * Computes the total acting force on a point as applied by neighboring
     * points. This is currently used to calculate the forces applied to the
     * control point(s) of a bezier flow. There are two forces that will be 
     * calculated for each control point: The combined force of all nodes on the 
     * target node, and the force of the spring that pulls the target node 
     * towards the center point between the start/end points of the flow. The 
     * force of the spring is stronger for flows with start/end points that are 
     * closer together.  
     * 
     * @param targetPoint The point that will be moved.
     * @param startPoint The start point of a BezierFlow
     * @param endPoint The end point of a BezierFlow
     * @param referencePoint The center point of a line drawn from the start point
     * to the end point
     * @param maxFlowLength The distance between the start and end point of the flow
     * who's start and end points are the furthest apart of all flows in the model.
     * @param flowBaseLength The distance between the start and end points of
     * a flow
     */
    public void computeTotalForce(Point targetPoint, Point startPoint, 
            Point endPoint, Point referencePoint, double maxFlowLength, double flowBaseLength) {
        
        // Get the nodes from the model
        Iterator<Point> nodeIterator = model.nodeIterator();

        double fxTotal = 0; // Total force along the x axis
        double fyTotal = 0; // total force along the y axis 
        double wTotal = 0; // sum of the weight of all forces

        // Iterate through the nodes. The forces of each node on the target will
        // be calculated and added to the totals
        while (nodeIterator.hasNext()) {
            Point node = nodeIterator.next();

            double xDist = targetPoint.x - node.x; // x distance from node to target
            double yDist = targetPoint.y - node.y; // y distance from node to target
            double l = Math.sqrt((xDist * xDist) + (yDist * yDist)); // euclidean distance from node to target
            // FIXME length of 0 causes division by 0
            double w = Math.pow(l, -idwExponent); // distance weight

            double fx = xDist / l; //normalized x distance
            double fy = yDist / l; //normalized y distance

            // Apply the weight to each focre
            fx *= w; // The force along the x-axis after weighting
            fy *= w; // The force along the y-axix after weighting

            // Add forces to the totals
            fxTotal += fx;
            fyTotal += fy;
            wTotal += w;
        }

        // Calculate the final total force of all nodes on the target point
        double fxFinal = fxTotal / wTotal;
        double fyFinal = fyTotal / wTotal;

        // Calculates the length of the spring.  The spring is a vector connecting
        // the reference point to the control point.
        double springLengthX = referencePoint.x - targetPoint.x; // x-length of the spring
        double springLengthY = referencePoint.y - targetPoint.y; // y-length of the spring
        
        // Calculates the stiffness of the spring based on the distance between
        // the start and end nodes. The closer together the nodes are, the 
        // stiffer the spring is.
        double flowSpringConstant = (-minFlowLengthSpringConstant 
                + maxFlowLengthSpringConstant) / maxFlowLength 
                * flowBaseLength + minFlowLengthSpringConstant;
        
        
        // Calculates the total spring force as determined by the
        // distance between start/end points.
        double springForceX = flowSpringConstant * springLengthX; 
        double springForceY = flowSpringConstant * springLengthY;

        // Adds the force of the spring to the force of all nodes
        double totalForceX = fxFinal + springForceX;
        double totalForceY = fyFinal + springForceY;

        // Moves the target point by the sum total of all forces
        double dx = totalForceX;
        double dy = totalForceY;
        targetPoint.x += dx;
        targetPoint.y += dy;
    }

}
