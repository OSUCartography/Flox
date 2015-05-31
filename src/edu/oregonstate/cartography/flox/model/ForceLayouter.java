package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

/**
 * ForceLayouter contains the algorithms that compute the total force that each
 * map feature emits and receives.
 *
 * @author danielstephen
 */
public class ForceLayouter {

    // model with all map features.
    private final Model model;
    
    /**
     * hash map with a line string for each flow to accelerate computations.
     * The content of the hash map needs to be updated whenever the start, end, 
     * or control point of a BŽzier flow changes.
     */
    HashMap<Flow, ArrayList<Point>> straightLinesMap = new HashMap<>();

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
     * Updates the hash map with line strings for each flow.
     */
    private void initStraightLinesHashMap(){
        straightLinesMap.clear();
        Iterator<Flow> iter = model.flowIterator();
        while(iter.hasNext()) {
            Flow flow = iter.next();
            ArrayList<Point> points = flow.toStraightLineSegments(0.01);
            straightLinesMap.put(flow, points);
        }
    }

    /**
     * Computes the total acting force on a point as applied by neighboring
     * points. This is used to calculate the forces applied to the
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

        double fxTotal = 0; // total force along the x axis
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
            ArrayList<Point> points = straightLinesMap.get(flow);
            int nPoints = points.size();

            // FIXME
            if (nPoints > 200) {
                System.err.println("flow lines with too many points");
                return new Force();
            }

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
     *
     * @param flow
     * @param maxFlowLength
     * @return
     */
    private double computeSpringConstant(QuadraticBezierFlow flow, double maxFlowLength) {

        double flowBaseLength = flow.getBaselineLength();
        double relativeFlowLength = flowBaseLength / maxFlowLength;
        double flowSpringConstant = (-model.getMinFlowLengthSpringConstant()
                + model.getMaxFlowLengthSpringConstant()) * relativeFlowLength
                + model.getMinFlowLengthSpringConstant();
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

    /**
     * Computes the anti-torsion force for a quadratic BŽzier flow
     *
     * @param flow
     * @return A force pulling the control point towards a perpendicular line on
     * the base line.
     */
    private Force computeAntiTorsionForce(QuadraticBezierFlow flow) {
        Point basePt = flow.getBaseLineMidPoint();
        Point cPt = flow.getCtrlPt();
        double dx = basePt.x - cPt.x;
        double dy = basePt.y - cPt.y;
        double l = Math.sqrt(dx * dx + dy * dy);
        double alpha = Math.atan2(dy, dx);
        double baseLineAzimuth = flow.getBaselineAzimuth();
        double diffToBaseNormal = Math.PI / 2 - baseLineAzimuth + alpha;
        double torsionF = Math.sin(diffToBaseNormal) * l;
        double antiTorsionW = model.getAntiTorsionWeight();
        double torsionFx = Math.cos(baseLineAzimuth) * torsionF * antiTorsionW;
        double torsionFy = Math.sin(baseLineAzimuth) * torsionF * antiTorsionW;
        return new Force(torsionFx, torsionFy);
    }

    private double gaussianWeight(double d, double maxFlowLength) {
        double K = model.getDistanceWeightExponent() / maxFlowLength;
        return Math.exp(-K * d * d);
    }

    private double inverseDistanceWeight(double d) {
        return 1. / Math.pow(d, model.getDistanceWeightExponent());
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
        ArrayList<Point> flowPoints = straightLinesMap.get(flow);

        // compute the sum of all force vectors that are applied on each 
        // flow segment
        Force externalF = new Force();
        double lengthOfForceVectorsSum = 0;
        for (Point pt : flowPoints) {
            // compute force applied by nodes and flows
            Force f = computeForceOnPoint(pt, flow);
            // add to totals
            externalF.add(f);
            lengthOfForceVectorsSum += f.length();
        }
        // compute ratio between length of total vector and the summed
        // length of the shorter forces. This is a measure of how peripheral the 
        // flow is.
        double forceRatio = externalF.length() / lengthOfForceVectorsSum;

        externalF.fx /= flowPoints.size();
        externalF.fy /= flowPoints.size();

        // compute anti-torsion force of flow
        Force antiTorsionF = computeAntiTorsionForce(flow);

        // compute spring force of flow
        double flowSpringConstant = computeSpringConstant(flow, maxFlowLength);
        flowSpringConstant *= forceRatio * forceRatio * model.getPeripheralStiffnessFactor() + 1;
        Force springF = computeSpringForce(basePt, cPt, flowSpringConstant);
        double springFLength = springF.length();
        double externalFLength = externalF.length();
        // FIXME
        final double K = 10;
        if (springFLength > externalFLength * K) {
            springF.scale(K * externalFLength / springFLength);
        }
        // compute total force: external forces + spring force + anti-torsion force
        double fx = externalF.fx + springF.fx + antiTorsionF.fx;
        double fy = externalF.fy + springF.fy + antiTorsionF.fy;
        return new Force(fx, fy);
    }

    public void layoutAllFlows(double weight) {
        ArrayList<Flow> flows = model.getFlows();
        if (flows.size() < 2) {
            return;
        }
        
        initStraightLinesHashMap();
        
        double maxFlowLength = model.getLongestFlowLength();

        // compute force for each flow for current configuration
        ArrayList<Force> forces = new ArrayList<>();
        
        for (Flow flow : flows) {
            if (flow instanceof QuadraticBezierFlow) {
                QuadraticBezierFlow qFlow = (QuadraticBezierFlow) flow;
                Force f = computeForceOnFlow(qFlow, maxFlowLength);
                forces.add(f);
            }
        }

        // apply forces onto control points of each flow
        RangeboxEnforcer enforcer = new RangeboxEnforcer(model);
        int nbrFlows = flows.size();
        for (int i = 0; i < nbrFlows; i++) {
            Flow flow = flows.get(i);
            if (flow instanceof QuadraticBezierFlow) {
                QuadraticBezierFlow qFlow = (QuadraticBezierFlow) flow;
                Point ctrlPt = qFlow.getCtrlPt();
                Force f = forces.get(i);

                // Move the control point by the total force
                ctrlPt.x += weight * f.fx;
                ctrlPt.y += weight * f.fy;

                // Enforce control point range if enforceRangebox
                // is true
                if (model.isEnforceRangebox()) {
                    Point tempPoint = enforcer.enforceFlowControlPointRange(qFlow);
                    ctrlPt.x = tempPoint.x;
                    ctrlPt.y = tempPoint.y;
                }

                if (model.isEnforceCanvasRange()) {
                    Rectangle2D canvasRect = model.getCanvas();
                    Point tempPoint = enforcer.enforceCanvasBoundingBox(qFlow, canvasRect);
                    ctrlPt.x = tempPoint.x;
                    ctrlPt.y = tempPoint.y;
                }
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
