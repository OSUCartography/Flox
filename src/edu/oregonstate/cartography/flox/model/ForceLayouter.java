package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.utils.GeometryUtils;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

/**
 * ForceLayouter contains the algorithms that compute the total force that each
 * map feature emits and receives.
 *
 * @author Bernhard Jenny
 * @author danielstephen
 */
public class ForceLayouter {

    /**
     * A utility structure for the integration over time using the velocity
     * Verlet method.
     * https://en.wikipedia.org/wiki/Verlet_integration#Velocity_Verlet
     */
    private class Verlet {

        public double vx; // velocity of current time step t
        public double vy;
        public double ax; // acceleration for current time step t
        public double ay;
        public double ax_; // acceleration for time step t + dt
        public double ay_;

        public double ang_vx; // angular distribution velocity of current time step t
        public double ang_vy;
        public double ang_ax; // angular distribution acceleration for current time step t
        public double ang_ay;
        public double ang_ax_; // angular distribution acceleration for time step t + dt
        public double ang_ay_;
    }

    public static final int NBR_ITERATIONS = 100;

    // model with all map features.
    private final Model model;

    /**
     * hash map with a line string of straight segments for each curved flow.
     * Used for accelerating computations. The content of the hash map needs to
     * be updated whenever the start, end, or control point of a Bézier flow
     * changes.
     */
    HashMap<Flow, Point[]> straightLinesMap = new HashMap<>();

    // store per flow values for Verlet velocity integration
    ArrayList<Verlet> verletVelocities;

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
        verletVelocities = new ArrayList<>(nFlows);
        for (int i = 0; i < nFlows; i++) {
            verletVelocities.add(new Verlet());
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
     * Computes the force exerted by all other flows on a point.
     *
     * @param targetPoint the point that receives the forces.
     * @param targetFlow the flow that receives the forces. targetPt is on this
     * flow.
     * @return the force exerted on the targetPoint.
     */
    private Force computeForceOnPoint(Point targetPoint, Flow targetFlow) {

        Rectangle2D.Double targetFlowBB = targetFlow.getBoundingBox();

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

            // ignor flow if it is too far away from the targetFlow
            double distSq = GeometryUtils.rectDistSq(targetFlowBB, flow.getBoundingBox());
            // distSq is 0 if rectangles intersect
            if (distSq > 0d) {
                double rectW = 1d / geometricSeriesPower(distSq, distWeightExponent);
                if (rectW < Model.MIN_W) {
                    continue;
                }
//                System.out.println(targetFlow);
//                System.out.println(flow);
//                System.out.println(distSq);
//                System.out.println(rectW);
//                System.out.println();
            }
            
            Point[] points = straightLinesMap.get(flow);
            for (Point point : points) {
                double xDist = targetPoint.x - point.x; // x distance from node to target
                double yDist = targetPoint.y - point.y; // y distance from node to target

                // square of euclidean distance from point to targetPoint
                double lSq = xDist * xDist + yDist * yDist;
                // avoid division by zero
                if (lSq == 0) {
                    continue;
                }

                // inverse distance weighting
                double w = 1d / geometricSeriesPower(lSq, distWeightExponent);

                // apply the distance weight to each force
                xDist *= w; // The force along the x-axis after weighting
                yDist *= w; // The force along the y-axix after weighting

                // add forces to the totals
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
        for (Point pt : flowPoints) {
            Force f = computeForceOnPoint(pt, flow);
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
            double wDist;
            // the following is experimental and apparently not of any benefit.
            // FIXME should be removed
            if (model.limitNodesRepulsionToBandHack) {
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
                if (projectedLength > baseLineLength / 2) {
                    wDist = 0;
                } else {
                    wDist = 1 - projectedLength / (baseLineLength / 2);
                }
            } else {
                wDist = 1;
            }

            // find nearest point on target flow
            xy[0] = node.x;
            xy[1] = node.y;
            double d = flow.distance(xy);
            double dx = (xy[0] - node.x);
            double dy = (xy[1] - node.y);

            // compute IDW from distance
            // avoid division by zero
            if (d == 0) {
                continue;
            }
            // TODO this could use a different method designed for nodes in
            // order to get a different distance weight.
            double idw = 1d / pow(d, distWeightExponent);
            idw *= wDist;
            fxTotal += dx * idw;
            fyTotal += dy * idw;
            wTotal += idw;
        }

        double fxFinal = fxTotal / wTotal;
        double fyFinal = fyTotal / wTotal;

        // Multiply by the node weight.
        fxFinal *= nodeWeight;
        fyFinal *= nodeWeight;

        return new Force(fxFinal, fyFinal);
    }

    /**
     * Applies a layout iteration to all unlocked flows. Uses Verlet velocity
     * integration.
     *
     * See https://en.wikipedia.org/wiki/Verlet_integration#Velocity_Verlet
     *
     * @param weight the weight for the displacements resulting from this
     * iteration
     */
    public void layoutAllFlows(double weight) {
        if (model.getNbrFlows() < 2) {
            return;
        }

        initStraightLinesHashMap();

        double maxFlowLength = model.getLongestFlowLength();

        Iterator<Flow> iterator = model.flowIterator();
        int j = 0;
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isLocked()) {
                continue;
            }

            Verlet v = verletVelocities.get(j++);
            Point ctrlPt = flow.getCtrlPt();

            // update position for time t + dt
            // dt is 1
            ctrlPt.x += v.vx + 0.5 * v.ax;
            ctrlPt.y += v.vy + 0.5 * v.ay;

            // Move the control point by the angular distribution force.
            // Angular distribution forces are not applied from the beginning 
            // of the iterative layout computation. Angular distribution forces 
            // kick in slowly to avoid creating crossing flows. 
            // The weight for regular forces varies from 
            // 1 to 0 with each iteration. The weight for angular 
            // distribution forces is w’ = -weight * weight + weight.    
            ctrlPt.x += v.ang_vx + 0.5 * v.ang_ax;
            ctrlPt.y += v.ang_vy + 0.5 * v.ang_ay;
        }

        // compute acceleration at time t + dt
        j = 0;
        iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isLocked()) {
                continue;
            }

            // compute force exerted by flows and nodes
            Force f = computeForceOnFlow(flow, maxFlowLength);
            Verlet v = verletVelocities.get(j);
            v.ax_ = weight * f.fx;
            v.ay_ = weight * f.fy;

            // compute force creating an even angular distribution of flows around 
            // nodes
            Force newAngularDistF = computeAngularDistributionForce(flow);
            double angularDistWeight = weight * (1 - weight);
            v.ang_ax_ = angularDistWeight * newAngularDistF.fx;
            v.ang_ay_ = angularDistWeight * newAngularDistF.fy;

            j++;
        }

        // compute velocity at time t + dt
        RangeboxEnforcer enforcer = new RangeboxEnforcer(model);
        iterator = model.flowIterator();
        int i = 0;
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isLocked()) {
                continue;
            }

            // update velocities and accelerations of Verlet velocity 
            // integration for next time step
            Verlet v = verletVelocities.get(i);
            v.vx += 0.5 * (v.ax + v.ax_);
            v.vy += 0.5 * (v.ay + v.ay_);
            v.ax = v.ax_;
            v.ay = v.ay_;
            v.ang_vx += 0.5 * (v.ang_ax + v.ang_ax_);
            v.ang_vy += 0.5 * (v.ang_ay + v.ang_ay_);
            v.ang_ax = v.ang_ax_;
            v.ang_ay = v.ang_ay_;

            Point ctrlPt = flow.getCtrlPt();

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
    private Force computeAngularDistributionForce(Flow flow) {

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

    /**
     * Returns true if the provided flow intersects the provided node.
     *
     * @param flow A Flow.
     * @param node A Node.
     * @param mapScale The current scale of the map
     * @return
     */
    private boolean flowIntersectsNode(Flow flow, Point node, double mapScale) {

        // Get the locked scale factor needed to calculate flow widths
        double lockedScaleFactor;
        if (!model.isScaleLocked()) {
            lockedScaleFactor = 1;
        } else {
            // compare the locked scale to the current scale
            double lockedMapScale = model.getLockedMapScale();
            lockedScaleFactor = mapScale / lockedMapScale;
        }

        // Get the current stroke width of the flow in pixels
        double flowStrokeWidthPx = Math.abs(flow.getValue()) * model.getFlowWidthScaleFactor()
                * lockedScaleFactor;

        // Find out what that width is in world coordinates
        double worldStrokeWidth = (flowStrokeWidthPx) / mapScale;

        // Get the current pixel radius of the node
        double nodeArea = Math.abs(node.getValue() * model.getNodeSizeScaleFactor());
        double nodeRadiusPx = (Math.sqrt(nodeArea / Math.PI)) * lockedScaleFactor;

        // Find out what that radius is in world coordinates
        // Add a bit to the pixel radius in order to make the radius a few pixels 
        // wider than the actual node and to account for the node's stroke width. 
        double worldNodeRadius = (nodeRadiusPx + model.getNodeTolerancePx()) / mapScale;

        // Add the worldNodeRadius to half the worldFlowWidth
        double threshDist = (worldStrokeWidth / 2) + worldNodeRadius;

        // Get the flow's bounding box, and add a padding to it of threshDist.
        Rectangle2D flowBB = flow.getBoundingBox();
        flowBB.add((flowBB.getMinX() - threshDist), (flowBB.getMinY() - threshDist));
        flowBB.add((flowBB.getMaxX() + threshDist), (flowBB.getMaxY() + threshDist));

        // the node must be inside the extended bounding box
        if (flowBB.contains(node.x, node.y) == false) {
            return false;
        }

        // Check the shortest distance between the node and the flow. If it's 
        // less than the threshold, then the flow intersects the node. 
        double[] xy = {node.x, node.y};
        double shortestDistSquare = flow.distanceSq(xy);
        return shortestDistSquare < threshDist * threshDist;
    }

    /**
     * Tests whether a flow overlaps a start or end node of any other flow.
     *
     * @param flow the flow to test
     * @param mapScale the current scale of the map
     * @return true if the flow overlaps a node
     */
    private boolean flowIntersectsANode(Flow flow, double mapScale) {
        Iterator<Point> nodeIterator = model.nodeIterator();
        while (nodeIterator.hasNext()) {
            Point node = nodeIterator.next();
            if (node != flow.getStartPt() && node != flow.getEndPt()) {
                if (flowIntersectsNode(flow, node, mapScale)) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Test whether a flow can be moved off a node. A flow cannot be moved off a
     * node if the node overlaps the center of the flow's start or end point.
     *
     * @param flow flow to test
     * @param node node to test
     * @param mapScale current map scale
     * @return true if flow can be moved off the node
     */
    public boolean isFlowMovable(Flow flow, Point node, double mapScale) {

        // FIXME hard-coded parameter
        double NODE_TOLERANCE_PX = 10;

        Point sPt = flow.getStartPt();
        Point ePt = flow.getEndPt();

        // TODO
        // Dan Comment: All this code involved in getting the pixel and world 
        // coordinates of flow width and node radius is repeated in several 
        // places. It might be better if it lived in just one place. 
        // Get the locked scale factor needed to calculate flow widths
        double lockedScaleFactor;
        if (!model.isScaleLocked()) {
            lockedScaleFactor = 1;
        } else {
            double lockedMapScale = model.getLockedMapScale();
            lockedScaleFactor = mapScale / lockedMapScale;
        }

        double flowStrokeWidthPx = Math.abs(flow.getValue()) * model.getFlowWidthScaleFactor()
                * lockedScaleFactor;

        double worldStrokeWidth = (flowStrokeWidthPx) / mapScale;

        double nodeArea = Math.abs(node.getValue() * model.getNodeSizeScaleFactor());

        double nodeRadiusPx = (Math.sqrt(nodeArea / Math.PI)) * lockedScaleFactor;

        double worldNodeRadius = (nodeRadiusPx + NODE_TOLERANCE_PX) / mapScale;

        // Comment by Dan: This checks to see if the node overlaps the center
        // of the flow's start point or end point. If it does, then there is 
        // no way (yet) to move the flow such that it no
        // longer intersects the node, and should not be attempted.
        return !(sPt.distance(node) - (worldNodeRadius + worldStrokeWidth / 2) < 0
                || ePt.distance(node) - (worldNodeRadius + worldStrokeWidth / 2) < 0);
    }

    /**
     * Returns an List of all flows that intersect nodes.
     *
     * @param scale the scale of the map.
     * @return
     */
    private ArrayList<Flow> getFlowsOverlappingNodes(double scale) {
        ArrayList<Flow> flowsArray = new ArrayList();
        Iterator<Flow> flowIterator = model.flowIterator();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            Iterator<Point> nodeIterator = model.nodeIterator();
            while (nodeIterator.hasNext()) {
                Point node = nodeIterator.next();
                if (node != flow.getStartPt() && node != flow.getEndPt()) {
                    if (flowIntersectsNode(flow, node, scale)) {
                        // check to see if the flow can be moved off the node
                        if (isFlowMovable(flow, node, scale)) {
                            flowsArray.add(flow);
                            break;
                        }

                    }
                }
            }
        }
        return flowsArray;
    }

    /**
     * Identifies flows that overlap nodes they are not connected to, and moves
     * the control point of overlapping flows away from nodes.
     *
     * @param scale current map scale
     */
    public void moveFlowsOverlappingNodes(double scale) {
        // get a list of all flows that intersect nodes.
        List<Flow> flowsArray = getFlowsOverlappingNodes(scale);
        // sort flows in decreasing order
        Model.sortFlows(flowsArray, false);
        // move control points of overlapping flows, starts with largest flow
        for (Flow flow : flowsArray) {
            moveFlowOverlappingANode(flow, scale);
        }
    }

    private void moveFlowOverlappingANode(Flow flow, double scale) {
        Point cPt = flow.getCtrlPt();
        double originalX = cPt.x;
        double originalY = cPt.y;
        double angleRad = Math.PI;

        // TODO control point may be moved outside of range box
        double dist = flow.getBaselineLength() / 50;
        for (int i = 0; i < 100; i++) { // FIXME hard-coded random parameter
            double spiralR = dist * angleRad / Math.PI / 2;
            cPt.x = Math.cos(angleRad) * spiralR + originalX;
            cPt.y = Math.sin(angleRad) * spiralR + originalY;
            angleRad += dist / spiralR;
            if (flowIntersectsANode(flow, scale) == false) {
                return;
            }
        }

        // could not find a position that does not overlap a node. Restore 
        // original coordinates.
        cPt.x = originalX;
        cPt.y = originalY;

    }

    public void computeArrowHeads(double mapScale) {
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();

            // Compute radius of clipping circle around end point.
            // Clip the flow with the clipping area and/or a circle around the end node
            double endClipRadius = model.endClipRadius(flow.getEndPt(), mapScale);

            // Create an arrowhead
            // Calculate the stroke width of the flow based on its value.
            double flowWidthScaleFactor = model.getFlowWidthScaleFactor();
            double flowStrokeWidth = Math.abs(flow.getValue()) * flowWidthScaleFactor
                    * model.getLockedScaleFactor(mapScale) / mapScale;
            flow.configureArrow(model, flowStrokeWidth, endClipRadius);
        }

        // TODO adjust the width of arrowheads
        ArrayList<Point> points = model.getNodes();
        for (Point point : points) {
            ArrayList<Flow> incomingFlows = model.getAnticlockwiseOrderedIncomingFlows(point);
            // System.out.println("Number of incoming flows at node " + point + ": " + incomingFlows.size());
        }

    }
}
