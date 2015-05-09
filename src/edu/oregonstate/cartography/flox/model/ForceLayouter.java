package edu.oregonstate.cartography.flox.model;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * ForceLayouter contains the algorithms that compute the total force that each
 * map feature emits and receives.
 *
 * @author danielstephen
 */
public class ForceLayouter {

    /**
     * spring stiffness of longest flow
     */
    private double maxFlowLengthSpringConstant = 0.5;

    /**
     * spring stiffness of zero-length flow
     */
    private double minFlowLengthSpringConstant = 0.5;

    // This determines the amount of force that objects far away from the target
    // can apply to the target.  The lower the idwExponent, the more force distant
    // objects are permitted to apply.
    private double idwExponent = 4;

    // Stores the model, which contains all map features.
    private final Model model;

    /**
     * Sets the spring constants. This is ultimately called by slider bars in
     * the GUI.
     *
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
     *
     * @param idwExponent The idwExponent to set
     */
    public void setIDWExponent(double idwExponent) {
        this.idwExponent = idwExponent;
    }

    /**
     * Constructor for the ForceLayouter. Requires a Model object containing
     * flow map features.
     *
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
     * @param targetFlow The flow that receives the forces.
     * @param referencePoint The center point of a line drawn from the start
     * point to the end point
     * @param maxFlowLength The distance between the start and end point of the
     * flow who's start and end points are the furthest apart of all flows in
     * the model.
     * @param flowBaseLength The distance between the start and end points of a
     * flow
     */
    public void computeTotalForce(Point targetPoint, Flow targetFlow,
            Point referencePoint, double maxFlowLength, double flowBaseLength) {

        Iterator<Flow> flowIterator = model.flowIterator();

        double fxTotal = 0; // Total force along the x axis
        double fyTotal = 0; // total force along the y axis 
        double wTotal = 0; // sum of the weight of all forces

        // Iterate through the flows. The forces of each flow on the target is
        // calculated and added to the total force
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            if (!model.isFlowExertingForcesOnItself() && targetFlow == flow) {
                continue;
            }
            ArrayList<Point> points = flow.toStraightLineSegments(0.01);
            int nPoints = points.size();
            for (int ptID = 0; ptID < nPoints; ptID++) {
                boolean startOrEndPoint = (ptID == 0 || ptID == nPoints - 1);
                double nodeWeight = startOrEndPoint? model.getNodeWeightFactor() : 1;
                Point point = points.get(ptID);

                double xDist = targetPoint.x - point.x; // x distance from node to target
                double yDist = targetPoint.y - point.y; // y distance from node to target
                double l = Math.sqrt((xDist * xDist) + (yDist * yDist)); // euclidean distance from node to target
                // avoid division by zero
                if (l == 0) {
                    continue;
                }
                
                //double w = Math.pow(l, -idwExponent); // distance weight
                double w = gaussianWeight(l, maxFlowLength);
                double fx = xDist / l; //normalized x distance
                double fy = yDist / l; //normalized y distance

                // Apply the distance weight to each focre
                fx *= w; // The force along the x-axis after weighting
                fy *= w; // The force along the y-axix after weighting

                // start and end points have bigger weight
                if (startOrEndPoint) {
                    fx *= nodeWeight;
                    fy *= nodeWeight;
                }

                // Add forces to the totals
                fxTotal += fx;
                fyTotal += fy;
                wTotal += w;
            }
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
    
    private double gaussianWeight(double d, double maxFlowLength) {
        double K = idwExponent / maxFlowLength;
        return Math.exp(-K * d * d);
    }

    private double inverseDistanceWeight(double d) {
        return 1. / Math.pow(d, idwExponent);
    }

    /**
     * Apply forces onto a quadratic BŽzier flow. The BŽzier curve is split into
     * small segments. The force exerted onto each node in the segmented flow is
     * computed and then these forces are summed. The summed force is then
     * applied onto the control point of the BŽzier curve.
     *
     * @param flow
     * @param maxFlowLength
     */
    public void applyForces(QuadraticBezierFlow flow, double maxFlowLength) {
        double flowBaseLength = flow.getBaselineLength();
        Point basePt = flow.getBaseLineMidPoint();
        ArrayList<Point> flowPoints = flow.toStraightLineSegments(0.01);
        double fxSum = 0;
        double fySum = 0;
        for (Point pt : flowPoints) {
            // a hack to compute the force applied on pt
            double x = pt.x;
            double y = pt.y;
            computeTotalForce(pt, flow, basePt, maxFlowLength, flowBaseLength);
            // force applied to the point
            double fx = pt.x - x;
            double fy = pt.y - y;
            fxSum += fx;
            fySum += fy;
        }

        // move the control point by the total force
        flow.getcPt().x += fxSum / flowPoints.size();
        flow.getcPt().y += fySum / flowPoints.size();
    }

    public void straightenFlows() {
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            flow.bend(0, 0);
        }
    }

}
