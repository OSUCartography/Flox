package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.utils.GeometryUtils;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

/**
 * ForceLayouter contains the algorithms that compute the total force that each
 * map feature emits and receives.
 *
 * @author Bernhard Jenny
 * @author danielstephen
 */
public class ForceLayouter {

    public static final int NBR_ITERATIONS = 100;

    // model with all map features.
    private final Model model;

    /**
     * hash map with a line string for each flow to accelerate computations. The
     * content of the hash map needs to be updated whenever the start, end, or
     * control point of a Bézier flow changes.
     */
    HashMap<Flow, Point[]> straightLinesMap = new HashMap<>();

    // store force for each flow for friction computation
    ArrayList<Force> forces;

    // store angular distribution force for each flow for friction computation
    ArrayList<Force> angularDistForces;

    /**
     * Constructor for the ForceLayouter. Requires a Model object containing
     * flow map features.
     *
     * @param model
     */
    public ForceLayouter(Model model) {
        this.model = model;

        // store force for each flow. This is used for friction computation.
        int nFlows = model.getNbrFlows();
        forces = new ArrayList<>(nFlows);
        for (int i = 0; i < nFlows; i++) {
            forces.add(new Force());
        }

        // store angular distribution force for each flow in this array
        // Angular distribution forces are computed separately, because they 
        // require a different weight than the other forces.
        angularDistForces = new ArrayList<>(nFlows);
        for (int i = 0; i < nFlows; i++) {
            angularDistForces.add(new Force());
        }
    }

    /**
     * Updates the hash map with line strings for each flow.
     */
    private void initStraightLinesHashMap() {
        straightLinesMap.clear();
        double deCasteljauTol = model.getDeCasteljauTolerance();
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            // FIXME value for first parameter is 0.
            ArrayList<Point> points = flow.toClippedStraightLineSegments(0, 0, deCasteljauTol);
            straightLinesMap.put(flow, points.toArray(new Point[points.size()]));
        }
    }

    /**
     * Computes the total acting force on a point as applied by neighboring
     * points. This is used to calculate the forces applied to the control
     * point(s) of a Bézier flow. There are two forces that will be calculated
     * for each control point: The combined force of all nodes on the target
     * node, and the force of the spring that pulls the target node towards the
     * center point between the start/end points of the flow. The force of the
     * spring is stronger for flows with start/end points that are closer
     * together.
     *
     * @param targetPoint The point that will be moved.
     * @param targetFlow The flow that receives the forces.
     * @return The force exerted on the targetPoint.
     */
    private Force computeForceOnPoint(Point targetPoint, Flow targetFlow) {

        Iterator<Flow> flowIterator = model.flowIterator();
        int distWeightExponent = model.getDistanceWeightExponent();

        double fxTotal = 0; // total force along the x axis
        double fyTotal = 0; // total force along the y axis 
        double wTotal = 0; // sum of the weight of all forces

        // Iterate through the flows. The forces of each flow on the target is
        // calculated and added to the total force
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            if (targetFlow == flow) {
                continue;
            }
            Point[] points = straightLinesMap.get(flow);

            // FIXME
            if (points.length > 200) {
                System.err.println("flow lines with too many points");
                return new Force();
            }

            for (int ptID = 0; ptID < points.length; ptID++) {
                Point point = points[ptID];
                double xDist = targetPoint.x - point.x; // x distance from node to target
                double yDist = targetPoint.y - point.y; // y distance from node to target

                // square of euclidean distance from node to target
                double lSq = xDist * xDist + yDist * yDist;
                // avoid division by zero
                if (lSq == 0) {
                    continue;
                }

                // inverse distance weighting
                double w = 1d / geometricSeriesPower(lSq, distWeightExponent);

                // Apply the distance weight to each force
                xDist *= w; // The force along the x-axis after weighting
                yDist *= w; // The force along the y-axix after weighting

                // Add forces to the totals
                fxTotal += xDist;
                fyTotal += yDist;
                wTotal += w;
            }
        }

        // Calculate the final force of all nodes on the target point
        double fxFinal = fxTotal / wTotal;
        double fyFinal = fyTotal / wTotal;

        return new Force(fxFinal, fyFinal);
    }

    private static Force computeSpringForce(Point startPt, Point endPt, double springConstant) {
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
    private double computeSpringConstant(Flow flow, double maxFlowLength) {

        double flowBaseLength = flow.getBaselineLength();
        double relativeFlowLength = flowBaseLength / maxFlowLength;
        double flowSpringConstant = (-model.getMinFlowLengthSpringConstant()
                + model.getMaxFlowLengthSpringConstant()) * relativeFlowLength
                + model.getMinFlowLengthSpringConstant();
        return flowSpringConstant;
    }

    /**
     * Computes the anti-torsion force for a quadratic Bézier flow.
     *
     * @param flow
     * @return A force pulling the control point towards a perpendicular line on
     * the base line.
     */
    private Force computeAntiTorsionForce(Flow flow) {
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

    private static boolean isEven(int i) {
        return (i % 2) == 0;
    }

    private static double pow(double a, int exp) {
        if (exp == 0) {
            return 1;
        }
        if (exp == 1) {
            return a;
        }
        if (isEven(exp)) {
            return pow(a * a, exp / 2); //even a=(a^2)^exp/2
        } else {
            return a * pow(a * a, exp / 2); //odd  a=a*(a^2)^exp/2
        }
    }

    /**
     * Returns a2 raised to the next geometric power. Example: a2^6 returns
     * a2^8. Note 1: a2 is the squared value: a2= a^2 Note 2: exp is clamped to
     * 32.
     *
     * @param a2 The square of a.
     * @param exp The exponent. Clamped to 32 if larger than 32
     * @return Parameter a raised to the next geometric power.
     */
    private static double geometricSeriesPower(double a2, int exp) {
        if (exp == 0) {
            return 1;
        }
        if (exp == 1) {
            return Math.sqrt(a2);
        }
        if (exp == 2) {
            return a2;
        }
        double a4 = a2 * a2;
        if (exp <= 4) {
            return a4;
        }
        double a8 = a4 * a4;
        if (exp <= 8) {
            return a8;
        }
        double a16 = a8 * a8;
        if (exp <= 16) {
            return a16;
        }
        assert (exp <= 32);
        return a16 * a16;
    }

    /**
     * Compute forces onto a quadratic Bézier flow. The Bézier curve is split
     * into small segments. The force exerted onto each node in the segmented
     * flow is computed and then these forces are summed. The summed force is
     * then applied onto the control point of the Bézier curve.
     *
     * @param flow
     * @param maxFlowLength
     * @return The force that is exerted onto the control point
     */
    private Force computeForceOnFlow(Flow flow, double maxFlowLength) {

        Point basePt = flow.getBaseLineMidPoint();
        Point cPt = flow.getCtrlPt();
        Point[] flowPoints = straightLinesMap.get(flow);

        // Compute forces applied by all flows on current flow
        Force externalF = new Force();
        double lengthOfForceVectorsSum = 0;
        int counter = 0;
        for (Point pt : flowPoints) {
            Force f = computeForceOnPoint(pt, flow);
            if (Double.isNaN(f.fx)) {
                System.out.println("problem " + counter);
            }
            counter++;
            // add to totals
            externalF.add(f);
            lengthOfForceVectorsSum += f.length();
        }

        // compute ratio between length of total vector and the summed
        // length of the shorter forces. This is a measure of how peripheral the 
        // flow is.
        double forceRatio = externalF.length() / lengthOfForceVectorsSum;

        externalF.fx /= flowPoints.length;
        externalF.fy /= flowPoints.length;

        // compute force applied by all start and end nodes on current flow
        Force nodeF = computeNodeForceOnFlow(flow);

        // compute anti-torsion force of flow
        Force antiTorsionF = computeAntiTorsionForce(flow);

        // compute spring force of flow
        double flowSpringConstant = computeSpringConstant(flow, maxFlowLength);
        flowSpringConstant *= forceRatio * forceRatio * model.getPeripheralStiffnessFactor() + 1;
        Force springF = computeSpringForce(basePt, cPt, flowSpringConstant);

        // compute total force: external forces + spring force + anti-torsion force
        // FIXME add nodes force with optional weight (maybe)
        double fx = externalF.fx + springF.fx + antiTorsionF.fx + nodeF.fx;
        double fy = externalF.fy + springF.fy + antiTorsionF.fy + nodeF.fy;
        return new Force(fx, fy);
    }

    private Force computeNodeForceOnFlow(Flow flow) {

        double nodeWeight = model.getNodesWeight();
        int distWeightExponent = model.getDistanceWeightExponent();

        double[] xy = new double[2];
        double wTotal = 0;
        double fxTotal = 0;
        double fyTotal = 0;

        // for each node: 
        Iterator nodeIterator = model.nodeIterator();
        while (nodeIterator.hasNext()) {
            Point node = (Point) nodeIterator.next();

            // If the node is the start or end point of the current flow
            if ((node == flow.getStartPt() || node == flow.getEndPt())) {
                continue;
            }

            // only consider start and end nodes that are above or below the 
            // base line of the flow.
            // project vector 'a' from base point to the node point onto the 
            // base line of the flow defined by vector 'b'.
            // if the length of the projected vector > base line length / 2
            // then the node is not vertically above or below the base line
            Point baseLineMidPoint = flow.getBaseLineMidPoint();
            Point endPoint = flow.getEndPt();
            double ax = node.x - baseLineMidPoint.x;
            double ay = node.y - baseLineMidPoint.y;
            double baseLineLength = flow.getBaselineLength(); // twice the tolerance
            double bx = endPoint.x - baseLineMidPoint.x;
            double by = endPoint.y - baseLineMidPoint.y;
            double projectedLength = Math.abs((ax * bx + ay * by) / baseLineLength);
            // a node not vertically above or below the base line will have a 
            // weight = 0.
            // A node on the normal vector on the base line passing through the 
            // base line mid point has a weight of 1 (i.e. projected Length = 0).
            // A node with a projected length = baseLineLength / 2 has a weight of 0.
            double wDist;
            if (projectedLength > baseLineLength / 2) {
                wDist = 0;
            } else {
                wDist = 1 - projectedLength / (baseLineLength / 2);
            }

            // find nearest point on target flow
            xy[0] = node.x;
            xy[1] = node.y;
            double d = flow.distance(xy);

            // start with unary direction vector
            double dx = (xy[0] - node.x) / d;
            double dy = (xy[1] - node.y) / d;

            // avoid division by zero
            if (d == 0) {
                continue;
            }

            // compute idw from distance
            // Maybe this could use a different method designed for nodes in
            // order to get a different distance weight.
            // FIXME should we use a different IDW exponent than for flows?
            double idw = 1d / pow(d, distWeightExponent); // inverseDistanceWeight(d);

            // apply IDW weight and distance-to-centra-normal weight
            dx *= idw * wDist;
            dy *= idw * wDist;

            // add to nodes force sum
            fxTotal += dx;
            fyTotal += dy;
            wTotal += idw;
        }

        double fxFinal = fxTotal / wTotal;
        double fyFinal = fyTotal / wTotal;

        // Multiply by the value of the GUI slider for node weight.
        fxFinal *= nodeWeight;
        fyFinal *= nodeWeight;

        return new Force(fxFinal, fyFinal);
    }

    /**
     * Applies a layout iteration to all unlocked flows.
     *
     * @param weight
     */
    public void layoutAllFlows(double weight) {
        int nbrFlows = model.getNbrFlows();
        if (nbrFlows < 2) {
            return;
        }

        initStraightLinesHashMap();

        double maxFlowLength = model.getLongestFlowLength();

        Iterator<Flow> iterator = model.flowIterator();
        int j = 0;
        double friction = 0.25; // FIXME should be a field
        // TODO Dorling used 0.25, but other values might work better?
        // add the new vector to the previous vector, and scale the sum.

        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isLocked()) {
                continue;
            }
            // compute force exerted by flows and nodes
            Force fnew = computeForceOnFlow(flow, maxFlowLength);
            Force f = forces.get(j);
            if (model.useFrictionForForcesHack) {
                f.fx = friction * (f.fx + fnew.fx);
                f.fy = friction * (f.fy + fnew.fy);
            } else {
                f.fx = fnew.fx;
                f.fy = fnew.fy;
            }
            // compute force creating an even angular distribution of flows around 
            // nodes
            Force angularDistF = angularDistForces.get(j);
            Force newAngularDistF = computeAngularDistributionForce(flow);
            if (model.useFrictionForAngularDistortionHack) {
                angularDistF.fx = friction * (angularDistF.fx + newAngularDistF.fx);
                angularDistF.fy = friction * (angularDistF.fy + newAngularDistF.fy);
            } else {
                angularDistF.fx = newAngularDistF.fx;
                angularDistF.fy = newAngularDistF.fy;
            }

            j++;
        }

        // apply forces onto control points of each flow
        RangeboxEnforcer enforcer = new RangeboxEnforcer(model);
        iterator = model.flowIterator();
        int i = 0;
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isLocked()) {
                continue;
            }

            Point ctrlPt = flow.getCtrlPt();

            // Move the control point by the total force
            Force f = forces.get(i);
            if (model.useFrictionForForcesHack) {
                ctrlPt.x += f.fx;
                ctrlPt.y += f.fy;
            } else {
                ctrlPt.x += weight * f.fx;
                ctrlPt.y += weight * f.fy;
            }

            // Move the control point by the angular distribution force.
            // Angular distribution forces are not applied from the beginning 
            // of the iterative layout computation. Angular distribution forces 
            // kick in slowly to avoid creating crossing flows. 
            // The weight for regular forces varies from 
            // 1 to 0 with each iteration. The weight for angular 
            // distribution forces is w’ = -weight * weight + weight.    
            double angularDistWeight = weight * (1 - weight);
            Force angularDistForce = angularDistForces.get(i);
            ctrlPt.x += angularDistWeight * angularDistForce.fx;
            ctrlPt.y += angularDistWeight * angularDistForce.fy;

            // Enforce control point range if enforceRangebox is true
            if (model.isEnforceRangebox()) {
                Point tempPoint = enforcer.enforceFlowControlPointRange(flow);
                ctrlPt.x = tempPoint.x;
                ctrlPt.y = tempPoint.y;
            }

            // Enforce the canvas range
            if (model.isEnforceCanvasRange()) {
                Rectangle2D canvasRect = model.getNodesBoundingBox();
                Point tempPoint = enforcer.enforceCanvasBoundingBox(flow, canvasRect);
                ctrlPt.x = tempPoint.x;
                ctrlPt.y = tempPoint.y;
            }
            i++;
        }
    }

    /**
     * Convert an angular difference to an angular force. A small difference
     * results in a large force, and vice versa. The sign of the angular
     * difference is retained by the angular force.
     *
     * @param angleDiff An signed angular difference in radians.
     * @return An signed angular force.
     */
    private double angularW(double angleDiff) {
        //FIXME hard-coded parameter
        final double K = 4;
        double w = Math.exp(-K * angleDiff * angleDiff);
        return angleDiff < 0 ? -w : w;
    }

    /**
     * Minimize angular tensions for one flow.
     *
     * @param flow The flow for which the angular distances to its neighbors are
     * balanced.
     * @return Force to be applied as displacement on control point.
     */
    public Force computeAngularDistributionForce(Flow flow) {

        Point startPoint = flow.getStartPt();
        Point endPoint = flow.getEndPt();

        // azimuthal angle for the line connecting the start point and the control point
        double startToCtrlAngle = flow.startToCtrlAngle();
        // azimuthal angle for the line connecting the end point and the control point
        double endToCtrlAngle = flow.endToCtrlAngle(); // FIXME this is = startToCtrlAngle - 180

        // loop over all other flows. Sum angular forces at start and end points of
        // the passed flow. These angular forces are exerted by other flows ending
        // or starting at the end or start nodes of the passed flow.
        // The angular differences between the flow and each other flow at the 
        // start and end nodes are computed. If the angular difference is small,
        // the angular force is large, and vice versa. angularW() converts an
        // angular difference to an angular force.
        double startAngleSum = 0;
        double endAngleSum = 0;

        // TODO instead of iterating over all flows, better use
        // graph.incomingEdgesOf() and graph.outgoingEdgesOf(endPoint)
        // to find edges connected to the start node and the end node
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow f = iter.next();
            if (f == flow) {
                continue;
            }
            Point fStart = f.getStartPt();
            Point fEnd = f.getEndPt();
            if (startPoint == fStart) {
                double fStartToCtrlAngle = f.startToCtrlAngle();
                double d = GeometryUtils.angleDif(startToCtrlAngle, fStartToCtrlAngle);
                startAngleSum += angularW(d);
            }

            if (startPoint == fEnd) {
                double fEndToCtrlAngle = f.endToCtrlAngle();
                double d = GeometryUtils.angleDif(startToCtrlAngle, fEndToCtrlAngle);
                startAngleSum += angularW(d);
            }

            if (endPoint == fStart) {
                double fStartToCtrlAngle = f.startToCtrlAngle();
                double d = GeometryUtils.angleDif(endToCtrlAngle, fStartToCtrlAngle);
                endAngleSum += angularW(d);
            }

            if (endPoint == fEnd) {
                double fEndToCtrlAngle = f.endToCtrlAngle();
                double d = GeometryUtils.angleDif(endToCtrlAngle, fEndToCtrlAngle);
                endAngleSum += angularW(d);
            }
        }

        // Convert the angles to two vectors that are tangent to a circle around 
        // the start / end nodes. The circles pass through the control point.
        // The length of the tangent Vector is angle * r
        // The direction of the tangent vector is normal to the vector connecting
        // the start or end point and the control point
        // length of the two vectors between start/end points and the control point
        double startVectorLength = startAngleSum * flow.getDistanceBetweenStartPointAndControlPoint();
        double endVectorLength = endAngleSum * flow.getDistanceBetweenEndPointAndControlPoint();

        // direction vectors between start/end points and the control point
        double[] startDir = flow.getDirectionVectorFromStartPointToControlPoint();
        double[] endDir = flow.getDirectionVectorFromEndPointToControlPoint();

        // vector tangent to the circle around the start point
        double startTangentX = -startDir[1] * startVectorLength;
        double startTangentY = startDir[0] * startVectorLength;

        // vector tangent to the circle around the end point
        double endTangentX = -endDir[1] * endVectorLength;
        double endTangentY = endDir[0] * endVectorLength;

        // sum the two vectors
        double angularDistributionWeight = model.getAngularDistributionWeight();
        double vectX = startTangentX + endTangentX;
        double vectY = startTangentY + endTangentY;
        Force f = new Force(vectX, vectY);

        // limit the lenght of the total vector
        // FIXME hard coded parameter
        double K = 4;
        double d1 = flow.getDistanceBetweenEndPointAndControlPoint();
        double d2 = flow.getDistanceBetweenStartPointAndControlPoint();
        double lmax = Math.min(d1, d2) / K;
        double l = f.length();
        if (l > lmax) {
            f.scale(lmax / l);
        }

        // scale by weight
        f.scale(angularDistributionWeight);

        return f;
    }

    /**
     * Converts all flows that are not locked to straight lines.
     *
     * @param onlySelected If true, only flows that are selected are converted
     * to straight lines.
     */
    public void straightenFlows(boolean onlySelected) {
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (onlySelected && flow.isSelected() == false) {
                continue;
            }
            if (!flow.isLocked()) {
                flow.straighten();
            }
        }
    }

    public void moveFlowsOverlappingNodes(double scale) {

        // Get an ArrayList of all flows that intersect nodes.
        ArrayList<Flow> flowsArray;
        flowsArray = GeometryUtils.getFlowsThatIntersectNodes(model, scale);

        // If flowsArray has anything in it, move flows that overlap nodes, update
        // flowsArray with flows that intersect nodes, and repeat until 
        // flowsArray is empty.
        // FIXME This is a potentially infinite loop. There might exist configurations
        // where there are always some flows that overlap some nodes
        while (flowsArray.size() > 0) {
            GeometryUtils.moveFlowsThatCrossNodes(flowsArray, scale);
            flowsArray = GeometryUtils.getFlowsThatIntersectNodes(model, scale);
        }
    }

}
