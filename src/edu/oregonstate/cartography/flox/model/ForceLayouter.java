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
    // can apply to the target.  The lower the distanceWeightExponent, the more force distant
    // objects are permitted to apply.
    private double distanceWeightExponent = 4;

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
     * Sets the distanceWeightExponent. This is currently set by the slider bar
     * in the GUI.
     *
     * @param idwExponent The distanceWeightExponent to set
     */
    public void setDistanceWeightExponent(double idwExponent) {
        this.distanceWeightExponent = idwExponent;
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
     * control point(s) of a BŽzier flow. There are two forces that will be
     * calculated for each control point: The combined force of all nodes on the
     * target node, and the force of the spring that pulls the target node
     * towards the center point between the start/end points of the flow. The
     * force of the spring is stronger for flows with start/end points that are
     * closer together.
     *
     * @param targetPoint The point that will be moved.
     * @param targetFlow The flow that receives the forces.
     * @return The force exerted on the targetPoint.
     */
    private Force computeForceOnPoint(Point targetPoint, Flow targetFlow) {

        Iterator<Flow> flowIterator = model.flowIterator();

        double fxTotal = 0; // Total force along the x axis
        double fyTotal = 0; // total force along the y axis 
        double wTotal = 0; // sum of the weight of all forces

        double nodeWeight = model.getNodeWeightFactor();

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

                Point point = points.get(ptID);

                double xDist = targetPoint.x - point.x; // x distance from node to target
                double yDist = targetPoint.y - point.y; // y distance from node to target
                double l = Math.sqrt((xDist * xDist) + (yDist * yDist)); // euclidean distance from node to target
                // avoid division by zero
                if (l == 0) {
                    continue;
                }

                // double w = gaussianWeight(l, maxFlowLength);
                double w = inverseDistanceWeight(l);
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

        return new Force(fxFinal, fyFinal);
    }

    private Force computeSpringForce(Point startPt, Point endPt, double springConstant) {
        // Calculates the length of the spring.  The spring is a vector connecting
        // the two points.
        double springLengthX = startPt.x - endPt.x; // x-length of the spring
        double springLengthY = startPt.y - endPt.y; // y-length of the spring

        // Calculates the total spring force as determined by the
        // distance between start/end points.
        double springForceX = springConstant * springLengthX;
        double springForceY = springConstant * springLengthY;

        return new Force(springForceX, springForceY);
    }

    /**
     * Calculates the stiffness of the spring based on the distance between the 
     * start and end nodes. The closer together the nodes are, the stiffer the 
     * spring usually is.
     * @param flow
     * @param maxFlowLength
     * @return 
     */
    private double computeSpringConstant(QuadraticBezierFlow flow, double maxFlowLength) {
       
        double flowBaseLength = flow.getBaselineLength();
        double relativeFlowLength = flowBaseLength / maxFlowLength;
        double flowSpringConstant = (-minFlowLengthSpringConstant
                + maxFlowLengthSpringConstant) * relativeFlowLength
                + minFlowLengthSpringConstant;
        return flowSpringConstant;
    }

    /**
     * S-shaped smooth function using cubic Hermite interpolation
     * http://en.wikipedia.org/wiki/Smoothstep
     *
     * @param edge0 interpolated values for x below edge0 will be 0.
     * @param edge1 interpolated values for x above edge1 will be 1.
     * @param x The x value to interpolate a value for.
     * @return
     */
    private static double smoothstep(double edge0, double edge1, double x) {
        // scale, bias and saturate x to 0..1 range
        x = Math.max(0, Math.min(1, (x - edge0) / (edge1 - edge0)));
        // evaluate polynomial
        return x * x * (3 - 2 * x);

        // alternative smootherstep
        // return x * x * x * (x * (x * 6 - 15) + 10);
    }

    private double gaussianWeight(double d, double maxFlowLength) {
        double K = distanceWeightExponent / maxFlowLength;
        return Math.exp(-K * d * d);
    }

    private double inverseDistanceWeight(double d) {
        return 1. / Math.pow(d, distanceWeightExponent);
    }

    /**
     * Compute forces onto a quadratic BŽzier flow. The BŽzier curve is split
     * into small segments. The force exerted onto each node in the segmented
     * flow is computed and then these forces are summed. The summed force is
     * then applied onto the control point of the BŽzier curve.
     *
     * @param flow
     * @param maxFlowLength
     * @return The force that is exerted onto the control point
     */
    public Force computeForceOnFlow(QuadraticBezierFlow flow, double maxFlowLength) {

        Point basePt = flow.getBaseLineMidPoint();
        Point cPt = flow.getCtrlPt();
        ArrayList<Point> flowPoints = flow.toStraightLineSegments(0.01);
        double flowSpringConstant = computeSpringConstant(flow, maxFlowLength);
        
        // compute the sum of all force vectors that are applied on each 
        // flow segment
        Force fSum = new Force();
        for (Point pt : flowPoints) {
            // compute force applied by nodes and flows
            Force externalF = computeForceOnPoint(pt, flow);
            // compute spring force of flow
            Force springF = computeSpringForce(basePt, cPt, flowSpringConstant);
            // Adds the force of the spring to the force of all nodes
            Force localForce = Force.add(externalF, springF);
            fSum.add(localForce);
        }

        // compute the anti-torsion force
        double dx = basePt.x - cPt.x;
        double dy = basePt.y - cPt.y;
        double l = Math.sqrt(dx * dx + dy * dy);
        double alpha = Math.atan2(dy, dx);
        double baseLineAzimuth = flow.getBaselineAzimuth();
        double diffToBaseNormal = Math.PI / 2 - baseLineAzimuth + alpha;
        double torsionF = Math.sin(diffToBaseNormal) * l;
        double torsionFx = Math.cos(baseLineAzimuth) * torsionF;
        double torsionFy = Math.sin(baseLineAzimuth) * torsionF;

        // move the control point by the total force
        double fx = fSum.fx / flowPoints.size() + torsionFx * model.getAntiTorsionWeight();
        double fy = fSum.fy / flowPoints.size() + torsionFy * model.getAntiTorsionWeight();
        return new Force(fx, fy);
    }

    public void layoutAllFlows(double weight) {
        double maxFlowLength = model.getLongestFlowLength();

        // compute force for each flow for current configuration
        ArrayList<Force> forces = new ArrayList<>();
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow instanceof QuadraticBezierFlow) {
                QuadraticBezierFlow qFlow = (QuadraticBezierFlow) flow;
                //layouter.computeForceOnPoint(qFlow.getCtrlPt(), flow.getStartPt(), 
                //        flow.getEndPt(), basePt, maxFlowLength, flowBaseLength);
                Force f = computeForceOnFlow(qFlow, maxFlowLength);
                forces.add(f);
            } else {
                //CubicBezierFlow cFlow = (CubicBezierFlow) flow;
                //double flowBaseLength = flow.getBaselineLength();
                //Point basePt = flow.getBaseLineMidPoint();
                //layouter.computeForceOnPoint(cFlow.getcPt1(), cFlow, basePt, maxFlowLength, flowBaseLength);
                //layouter.computeForceOnPoint(cFlow.getcPt2(), cFlow, basePt, maxFlowLength, flowBaseLength);
            }
        }

        iterator = model.flowIterator();

        // apply forces onto control points of flows
        int i = 0;
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow instanceof QuadraticBezierFlow) {
                QuadraticBezierFlow qFlow = (QuadraticBezierFlow) flow;
                Point ctrlPt = qFlow.getCtrlPt();
                Force f = forces.get(i++);
                ctrlPt.x += weight * f.fx;
                ctrlPt.y += weight * f.fy;
            }
        }
    }

    public void straightenFlows() {
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            flow.bend(0, 0);
        }
    }

}
