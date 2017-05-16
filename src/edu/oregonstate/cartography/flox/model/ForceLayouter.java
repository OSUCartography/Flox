package edu.oregonstate.cartography.flox.model;

import static edu.oregonstate.cartography.flox.model.Model.ARROWHEAD_WEIGHT_FOR_INTERSECTION_INDEX;
import edu.oregonstate.cartography.utils.GeometryUtils;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import net.jafama.FastMath;

/**
 * ForceLayouter contains the algorithms that compute the total force that each
 * map feature emits and receives. A ForceLayouter changes the location of
 * control points and the geometry of arrowheads.
 *
 * @author Bernhard Jenny
 * @author danielstephen
 */
public class ForceLayouter {

    /**
     * data model with all flows, nodes, and layout settings.
     */
    private final Model model;

    /**
     * regularly checked for cancellation; can be null
     */
    private ProcessMonitor processMonitor = null;

    // store per flow force
    private final ArrayList<Force> forces;

    // store angular distribution force for each flow for friction computation
    private final ArrayList<Force> angularDistForces;

    /**
     * Constructor
     *
     * @param model data model
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
     * Returns the model. The model must not be changed while a layout is
     * computed.
     *
     * @return model
     */
    public Model getModel() {
        return model;
    }

    /**
     * Apply new control point locations to a model. This method should be
     * called after all iterations are completed and a new layout has been
     * generated.
     *
     * @param destinationModel the flow control points of this model will be
     * replaced with a flow control points computed by this ForceLayouter.
     */
    public void applyChangesToModel(Model destinationModel) {
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            destinationModel.updateControlPointAndShortening(iterator.next());
        }
    }

    /**
     * Compute one iteration, including forces computation, moving overlapping
     * sibling flows, moving nodes from obstacles, and constraining control
     * points to range boxes.
     *
     * @param iteration current iteration, between 0 and
     * model.getNbrIterations() - 1
     * @param iterBeforeMovingFlowsOffObstacles number of remaining iterations
     * until flows are moved away from obstacles
     * @param canvas control points are to be constrained to this rectangle
     * @return number of remaining iterations until flows are moved away from
     * obstacles
     */
    public int layoutIteration(int iteration, int iterBeforeMovingFlowsOffObstacles,
            Rectangle2D canvas) {

        System.out.println(1 + iteration + "/" + model.getNbrIterations());

        model.invalidateCachedValues();

        RangeboxEnforcer enforcer = new RangeboxEnforcer(model);

        // compute one iteration of forces with a linearly decreasing weight
        double weight = 1d - (double) iteration / model.getNbrIterations();
        computeForces(weight, canvas);

        // try moving flows that intersect and are connected to the same node
        if (model.isResolveIntersectionsForSiblings()) {
            ArrayList<Model.IntersectingFlowPair> pairs = getSortedIntersectingSiblings();
            if (isCancelled()) {
                return 0;
            }

            System.out.println("# intersecting pairs: " + pairs.size());

            for (Model.IntersectingFlowPair pair : pairs) {
                if (isCancelled()) {
                    return 0;
                }

                pair.resolveIntersection();

                // move control points if they are outside of the range box or the canvas
                if (model.isEnforceRangebox()) {
                    enforcer.enforceFlowControlPointRange(pair.flow1);
                    enforcer.enforceFlowControlPointRange(pair.flow2);
                }
                if (model.isEnforceCanvasRange()) {
                    enforcer.enforceCanvasBoundingBox(pair.flow1, canvas);
                    enforcer.enforceCanvasBoundingBox(pair.flow2, canvas);
                }
            }
        }

        // move flows away from obstacles. Moved flows will be locked.
        if (model.isMoveFlowsOverlappingObstacles() && iterBeforeMovingFlowsOffObstacles == 0) {
            int remainingIterations = model.getNbrIterations() - iteration - 1;

            // nodes and arrowheads are obstacles
            List<Obstacle> obstacles = getObstaclesFromCachedCurves(model);

            // get a list of flows that intersect obstacles
            ArrayList<Flow> sortedOverlappingFlows = getFlowsOverlappingObstacles(
                    obstacles);
            // sort flows by decreasing intersection index, which assigns a smaller weight to arrowheads than nodes.
            sortedOverlappingFlows.sort(Collections.reverseOrder(
                    new Comparator<Flow>() {
                @Override
                public int compare(Flow f1, Flow f2) {
                    int minObstacleDistPx = model.getMinObstacleDistPx();
                    double nbrIntersectingObstacles1 = intersectionIndex(
                            f1, obstacles, minObstacleDistPx, ARROWHEAD_WEIGHT_FOR_INTERSECTION_INDEX);
                    double nbrIntersectingObstacles2 = intersectionIndex(
                            f2, obstacles, minObstacleDistPx, ARROWHEAD_WEIGHT_FOR_INTERSECTION_INDEX);
                    return Double.compare(nbrIntersectingObstacles1, nbrIntersectingObstacles2);
                }
            }));

            int nbrOverlaps = sortedOverlappingFlows.size();
            System.out.println("# flows overlapping obstacles: " + nbrOverlaps);

            // Compute the number of flows to move. Default is 1, but this might 
            // have to be larger when there are more overlapping flows than 
            // remaining iterations.
            int nbrFlowsToMove = 1;
            if (nbrOverlaps > remainingIterations && remainingIterations > 0) {
                nbrFlowsToMove = (int) Math.ceil(nbrOverlaps / remainingIterations);
            }

            // move flows
            int nbrRemainingOverlaps = moveFlowsAwayFromObstacles(obstacles,
                    sortedOverlappingFlows, nbrFlowsToMove);

            // compute the number of iterations until the next flow will 
            // be moved away from obstacles
            if (nbrRemainingOverlaps > 0) {
                // division by empirical factor = 2 to increase the 
                // number of iterations at the end of calculations
                iterBeforeMovingFlowsOffObstacles = (model.getNbrIterations() - iteration) / (nbrRemainingOverlaps + 1) / 2;
            } else {
                // There are no flows left hat overlap obstacles. Future
                // iterations may again create overlaps. So check after
                // 50% of the remaining iterations for new overlaps.
                iterBeforeMovingFlowsOffObstacles = remainingIterations / 2;
            }
        } else {
            --iterBeforeMovingFlowsOffObstacles;
        }

        return iterBeforeMovingFlowsOffObstacles;
    }

    /**
     * Compute one iteration of forces exerted on control points of all flows.
     *
     * @param weight the weight for the displacements resulting from this
     * iteration
     * @param canvas map canvas. Control points may be forces to stay within
     * this rectangle.
     */
    private void computeForces(double weight, Rectangle2D canvas) {
        if (model.getNbrFlows() < 2) {
            return;
        }

        double maxFlowLength = model.getLongestFlowLength();

        Iterator<Flow> flowIterator = model.flowIterator();
        int j = 0;
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            if (flow.isLocked()) {
                continue;
            }

            // compute and store force exerted by flows and nodes
            Force f = forces.get(j);
            computeForceOnFlow(flow, maxFlowLength, f);

            // compute force creating an even angular distribution of flows around 
            // nodes
            Force angularDistF = angularDistForces.get(j);
            Force newAngularDistF = computeAngularDistributionForce(flow);
            if (isCancelled()) {
                return;
            }
            angularDistF.fx = newAngularDistF.fx;
            angularDistF.fy = newAngularDistF.fy;
            j++;
        }

        // compute velocity at time t + dt
        RangeboxEnforcer enforcer = new RangeboxEnforcer(model);
        flowIterator = model.flowIterator();
        int i = 0;
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            if (flow.isLocked()) {
                continue;
            }

            // displacement of the control point by the total force
            Force f = forces.get(i);
            double dx = weight * f.fx;
            double dy = weight * f.fy;

            // Displacement of the control point by the angular distribution force.
            // Angular distribution forces are not applied from the beginning 
            // of the iterative layout computation. Angular distribution forces 
            // kick in slowly to avoid creating crossing flows. 
            // The weight for regular forces varies from 
            // 1 to 0 with each iteration. The weight for angular 
            // distribution forces is wÕ = -weight * weight + weight.    
            double angularDistWeight = weight * (1 - weight);
            Force angularDistForce = angularDistForces.get(i);
            flow.offsetCtrlPt(weight * f.fx, weight * f.fy);
            dx += angularDistWeight * angularDistForce.fx;
            dy += angularDistWeight * angularDistForce.fy;

            // move control point
            flow.offsetCtrlPt(dx, dy);

            // move control point if it is outside of the range box or the canvas
            if (model.isEnforceRangebox()) {
                enforcer.enforceFlowControlPointRange(flow);
            }
            if (model.isEnforceCanvasRange()) {
                enforcer.enforceCanvasBoundingBox(flow, canvas);
            }
            i++;

            if (isCancelled()) {
                return;
            }
        }
    }

    /**
     * Computes the force exerted by all other flows on a point.
     *
     * @param targetPoint the point that receives the forces.
     * @param targetFlow the flow that receives the forces. targetPt is on this
     * flow.
     * @param outForce the force exerted on the targetPoint. Only used to return
     * the new force.
     */
    private void computeForceOnPoint(Point targetPoint, Flow targetFlow, Force outForce) {
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

            Point[] points = flow.cachedClippedPolylineIncludingArrow(model);
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

            if (isCancelled()) {
                return;
            }
        }

        // Calculate the final force of all nodes on the target point
        outForce.fx = fxTotal / wTotal;
        outForce.fy = fyTotal / wTotal;
    }

    private static Force computeSpringForce(Point startPt, double endX, double endY, double springConstant) {
        // Calculates the length of the spring.  The spring is a vector connecting
        // the two points.
        double springLengthX = startPt.x - endX; // x-length of the spring
        double springLengthY = startPt.y - endY; // y-length of the spring

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
        double dx = basePt.x - flow.cPtX();
        double dy = basePt.y - flow.cPtY();
        double l = Math.sqrt(dx * dx + dy * dy);
        double alpha = FastMath.atan2(dy, dx);
        double baseLineAzimuth = flow.getBaselineOrientation();
        double diffToBaseNormal = Math.PI / 2 - baseLineAzimuth + alpha;
        double torsionF = FastMath.sin(diffToBaseNormal) * l;
        double antiTorsionW = model.getAntiTorsionWeight();
        double torsionFx = FastMath.cos(baseLineAzimuth) * torsionF * antiTorsionW;
        double torsionFy = FastMath.sin(baseLineAzimuth) * torsionF * antiTorsionW;
        return new Force(torsionFx, torsionFy);
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
     * @param flow flow to compute force for.
     * @param maxFlowLength FIXME
     * @param on output outForce the force that is exerted onto the control
     * point.
     */
    private void computeForceOnFlow(Flow flow, double maxFlowLength, Force outForce) {

        Point basePt = flow.getBaseLineMidPoint();
        Point[] flowPoints = flow.cachedClippedPolylineIncludingArrow(model);

        // Compute forces applied by all flows on current flow
        Force externalF = new Force();
        double lengthOfForceVectorsSum = 0;

        Force f = new Force();
        for (Point pt : flowPoints) {
            computeForceOnPoint(pt, flow, f);
            if (isCancelled()) {
                return;
            }

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
        if (isCancelled()) {
            return;
        }

        // compute anti-torsion force of flow
        Force antiTorsionF = computeAntiTorsionForce(flow);

        // compute spring force of flow
        double flowSpringConstant = computeSpringConstant(flow, maxFlowLength);
        flowSpringConstant *= forceRatio * forceRatio * model.getPeripheralStiffnessFactor() + 1;
        Force springF = computeSpringForce(basePt, flow.cPtX(), flow.cPtY(), flowSpringConstant);

        // compute total force: external forces + spring force + anti-torsion force
        outForce.fx = externalF.fx + springF.fx + antiTorsionF.fx + nodeF.fx;
        outForce.fy = externalF.fy + springF.fy + antiTorsionF.fy + nodeF.fy;
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

            // find nearest point on target flow
            xy[0] = node.x;
            xy[1] = node.y;
            double d = Math.sqrt(flow.distanceSquare(xy));
            double dx = (xy[0] - node.x);
            double dy = (xy[1] - node.y);

            // compute IDW from distance
            // avoid division by zero
            if (d == 0) {
                continue;
            }
            
            // FIXME no need to compute sqrt of distanceSquare: should use the fact that pow of a power is power * power.
            double idw = 1d / FastMath.powFast(d, distWeightExponent);
            fxTotal += dx * idw;
            fyTotal += dy * idw;
            wTotal += idw;

            if (isCancelled()) {
                return null;
            }
        }

        // if there are are only two nodes and all flows are between these two 
        // nodes, the total weigth will be 0.
        if (wTotal == 0) {
            return new Force(0, 0);
        }

        double fxFinal = fxTotal / wTotal;
        double fyFinal = fyTotal / wTotal;

        // Multiply by the node weight.
        fxFinal *= nodeWeight;
        fyFinal *= nodeWeight;

        return new Force(fxFinal, fyFinal);
    }

    /**
     * Returns a list with pairs of flows that intersect and are connected to a
     * shared node. The list does not include pairs where both flows are locked.
     *
     * @return pairs of flows that have a common start or end node.
     */
    public ArrayList<Model.IntersectingFlowPair> getSortedIntersectingSiblings() {
        ArrayList<Flow> flows = model.getFlows();
        ArrayList<Model.IntersectingFlowPair> pairs = new ArrayList<>();

        for (int i = 0; i < flows.size(); i++) {
            Flow flow1 = flows.get(i);
            // FIXME need a better way here
            for (int j = i + 1; j < flows.size(); j++) {
                Flow flow2 = flows.get(j);
                // pairs where both flows are locked are ignored, as nothing can be changed for locked flows.
                if (flow1.isLocked() && flow2.isLocked()) {
                    continue;
                }
                Point sharedNode = flow1.getSharedNode(flow2);
                if (sharedNode != null && flow1.cachedClippedCurveIncludingArrowIntersects(flow2, model)) {
                    pairs.add(new Model.IntersectingFlowPair(flow1, flow2, sharedNode));
                }
                if (isCancelled()) {
                    return null;
                }
            }
        }

        pairs.sort(null);
        return pairs;
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
        double w = FastMath.exp(-K * angleDiff * angleDiff);
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

            if (isCancelled()) {
                return null;
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
        double angularDistributionWeight = model.getAngularDistributionWeight();
        f.scale(angularDistributionWeight);

        return f;
    }

    /**
     * Tests whether a flow overlaps any obstacle.
     *
     * @param flow the flow to test
     * @param obstacles circular obstacles
     * @param testArrowheadObstacles flag to indicate whether arrowhead
     * obstacles are considered
     * @param minObstaclesDistPx minimum empty space between the flow and
     * obstacles (in pixels)
     * @return true if the flow overlaps a node
     */
    private boolean flowIntersectsObstacles(Flow flow, List<Obstacle> obstacles,
            int minObstacleDistPx) {
        for (Obstacle obstacle : obstacles) {
            if (flow.isOverlappingObstacle(obstacle, model, minObstacleDistPx)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns the number of overlaps between a flow and a list of obstacles.
     *
     * @param flow the flow to test
     * @param obstacles obstacles to test against
     * @param minObstacleDistPx minimum empty space between the flow and
     * obstacles (in pixels)
     * @return true if the flow overlaps a node
     */
    public int countIntersectingObstacles(Flow flow, List<Obstacle> obstacles,
            int minObstacleDistPx) {
        return (int) Math.round(intersectionIndex(flow, obstacles, minObstacleDistPx, 1d));
    }

    /**
     * Computes an index quantifying the number of intersections of a flow with
     * obstacles. The index is a weighted sum of intersections. Node obstacles
     * have a weight of 1. The arrowheadWeight parameter sets the weight of
     * arrowheads.
     *
     * @param flow flow to compute the intersection index for
     * @param obstacles obstacles
     * @param minObstacleDistPx the minimum distance between the flow and the obstacles (in pixel)
     * @param arrowheadWeight the weight for arrowheads
     * @return the index larger or equal to 0. 0 indicates no intersection.
     */
    public double intersectionIndex(Flow flow, List<Obstacle> obstacles,
            int minObstacleDistPx, double arrowheadWeight) {
        double nbrIntersections = 0;
        for (Obstacle obstacle : obstacles) {
            if (flow.isOverlappingObstacle(obstacle, model, minObstacleDistPx)) {
                if (obstacle.isArrowObstacle()) {
                    nbrIntersections += arrowheadWeight;
                } else {
                    ++nbrIntersections;
                }
            }
        }
        return nbrIntersections;
    }

    /**
     * Returns a list of all flows that intersect obstacles.
     *
     * @return a list of flows overlapping any of the passed obstacles
     */
    public ArrayList<Flow> getFlowsOverlappingObstacles() {
        model.invalidateCachedValues();
        List<Obstacle> obstacles = getObstaclesFromCachedCurves(model);
        return getFlowsOverlappingObstacles(obstacles);
    }

    public List<Obstacle> getObstacles() {
        model.invalidateCachedValues();
        return getObstaclesFromCachedCurves(model);
    }

    /**
     * Returns a list of obstacles, i.e., arrowheads and nodes.
     *
     * @return list of obstacles
     */
    private List<Obstacle> getObstaclesFromCachedCurves(Model model) {
        List<Obstacle> obstacles = new ArrayList<>();

        // nodes are obstacles
        Iterator<Point> nodeIterator = model.nodeIterator();
        while (nodeIterator.hasNext()) {
            Point node = nodeIterator.next();

            // nodes are obstacles
            double nodeRadiusPx = model.getNodeRadiusPx(node);
            double strokeWidthPx = model.getNodeStrokeWidthPx();
            final double rPx;
            if (strokeWidthPx > nodeRadiusPx * 2) {
                // stroke is wider than the diameter of the circle
                rPx = strokeWidthPx;
            } else {
                rPx = nodeRadiusPx + 0.5 * strokeWidthPx;
            }
            double rWorld = rPx / model.getReferenceMapScale();
            obstacles.add(new Obstacle(node, node.x, node.y, rWorld));
        }

        // arrowheads are obstacles
        if (model.isDrawArrowheads()) {
            Iterator<Flow> flowIterator = model.flowIterator();
            while (flowIterator.hasNext()) {
                Flow flow = flowIterator.next();
                Arrow arrow;
                if (flow instanceof FlowPair) {
                    FlowPair flowPair = (FlowPair) flow;

                    // create obstacle for arrowhead of first flow
                    arrow = flowPair.cachedOffsetFlow1(model).getArrow(model);
                    if (arrow.getLength() > Circle.TOL && arrow.getWidth() > Circle.TOL) {
                        Obstacle obstacle = new Obstacle(arrow.getTipPt(),
                                arrow.getCorner1Pt(), arrow.getCorner2Pt(), flow);
                        obstacles.add(obstacle);
                    }
                    // create obstacle for arrowhead of second flow below
                    arrow = flowPair.cachedOffsetFlow2(model).getArrow(model);
                } else {
                    arrow = flow.getArrow(model);
                }

                // create obstacle for arrowhead 
                if (arrow.getLength() > Circle.TOL && arrow.getWidth() > Circle.TOL) {
                    Obstacle obstacle = new Obstacle(arrow.getTipPt(),
                            arrow.getCorner1Pt(), arrow.getCorner2Pt(), flow);
                    obstacles.add(obstacle);
                }
            }
        }

        return obstacles;
    }

    /**
     * Returns a list of all flows that intersect a list of passed obstacles.
     *
     * @param obstacles a list of obstacles
     */
    private ArrayList<Flow> getFlowsOverlappingObstacles(List<Obstacle> obstacles) {
        ArrayList<Flow> flowsArray = new ArrayList();
        Iterator<Flow> flowIterator = model.flowIterator();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            if (flowIntersectsObstacles(flow, obstacles,
                    model.getMinObstacleDistPx())) {
                flowsArray.add(flow);
            }
        }
        return flowsArray;
    }

    /**
     * Compute spacing of sample points in world coordinates. The spacing
     * between candidate control points is equal to the minimum distance to
     * obstacles. Increase to accelerate computations. minObstacleDistPx can be
     * zero. We require a distance of at least 3 pixels to move along the
     * spiral.
     *
     * @return search increment in world coordinates
     */
    private double searchIncrement() {
        int minObstacleDistPx = model.getMinObstacleDistPx();
        double refScale = model.getReferenceMapScale();
        return Math.max(minObstacleDistPx, Model.MIN_SEARCH_INCREMENT_PX) / refScale;
    }

    /**
     * Moves the control point location of one flow such that the flow does
     * overlap a minimum of obstacles. The control point of the passed flow is
     * changed if possible.
     *
     * First tests control point locations placed along an Archimedean spiral
     * centered on the current control point location. If no position can be
     * found without overlaps, a position resulting in a minimum of overlaps is
     * searched inside the range box.
     *
     * If possible flows keep a distance of model.getMinObstacleDistPx() to
     * obstacles. If this is not possible, this distance is reduced.
     *
     * @param flow flow to change
     * @param obstacles obstacles to avoid
     */
    private void moveFlowAwayFromObstacles(Flow flow, List<Obstacle> obstacles) {

        // search for position inside range box that results in no overlaps
        boolean foundPosition = findControlPointWithoutOverlapsInsideRangeBoxWithFlexibleMinDistance(flow, obstacles);
        if (foundPosition) {
            return;
        }

        // search for candidate positions inside the range box
        // store a measure for the number of overlaps in an array
        int minObstacleDistPx = model.getMinObstacleDistPx();
        ArrayList<Point> candidateControlPoints = new ArrayList<>();
        while (true) {
            double overlaps = findControlPointWithMinimumOverlapsInsideRangeBox(
                    flow, obstacles, minObstacleDistPx);
            // store control point coordinates and number of overlaps in array
            candidateControlPoints.add(new Point(flow.cPtX(), flow.cPtY(), overlaps));
            if (minObstacleDistPx == 0) {
                break;
            }
            minObstacleDistPx /= 2;
            if (minObstacleDistPx == 1) {
                // ignore 1-pixel distance to accelerate computations
                minObstacleDistPx = 0;
            }
        }
        // find control point with smallest amount of overlaps in array
        int minOverlapsIndex = 0;
        double minOverlaps = candidateControlPoints.get(0).getValue();
        for (int i = 1; i < candidateControlPoints.size(); i++) {
            double v = candidateControlPoints.get(i).getValue();
            if (v < minOverlaps) {
                minOverlaps = v;
                minOverlapsIndex = i;
            }
        }
        Point cPt = candidateControlPoints.get(minOverlapsIndex);
        flow.setCtrlPt(cPt.x, cPt.y);
    }

    /**
     * Searches for position inside range box such that the flow does overlap a
     * any obstacles. If no position can be found that is at least
     * model.getMinObstacleDistPx() pixels away from all obstacles, a loop
     * divides the minimum distance by 2 until the distance is 0 or a position
     * without overlaps can be found.
     *
     * @param flow The control point of this flow is changed if possible.
     * @param obstacles obstacles to avoid
     * @return true if a the control point position of flow has been changed.
     */
    private boolean findControlPointWithoutOverlapsInsideRangeBoxWithFlexibleMinDistance(Flow flow, List<Obstacle> obstacles) {
        // 
        int minObstacleDistPx = model.getMinObstacleDistPx();
        while (true) {
            boolean foundPosition = findControlPointWithoutOverlapsInsideRangeBox(
                    flow, obstacles, minObstacleDistPx);
            if (foundPosition) {
                return true;
            }
            if (minObstacleDistPx == 0) {
                return false;
            }
            // half minimum distance between flow and obstacles
            minObstacleDistPx /= 2;
        }
    }

    /**
     * Moves the control point location of one flow such that the flow does not
     * overlap any obstacles. The control point of the passed flow is changed if
     * possible.
     *
     * Tests control point locations placed along an Archimedean spiral centered
     * on the current control point location.
     *
     * @param flow flow to change
     * @param obstacles obstacles to avoid
     * @param minObstacleDistPx minimum distance between the flow and obstacles
     * @return
     */
    private boolean findControlPointWithoutOverlapsInsideRangeBox(Flow flow,
            List<Obstacle> obstacles, int minObstacleDistPx) {

        double searchIncrement = searchIncrement();
        double originalX = flow.cPtX();
        double originalY = flow.cPtY();
        double angleRad = Math.PI;
        RangeboxEnforcer rangeBoxEnforcer = new RangeboxEnforcer(model);

        // compute the maximum possible radius for the spiral
        // the maximum radius is the distance between the control point (which
        // is the center of the spiral) and the corner point of the range box
        // that is the farthest away from the control point.
        Point[] rangeBox = rangeBoxEnforcer.computeRangebox(flow);
        double maxSpiralRadiusSquare = rangeBoxEnforcer.longestDistanceSqToCorner(
                rangeBox, originalX, originalY);

        // place the control point along the spiral until a position is found 
        // that does not result in any overlap
        double spiralR;
        do {
            // radius of spiral for the current angle.
            // The distance between two windings is searchIncrement.
            spiralR = searchIncrement * angleRad / Math.PI / 2;

            // new control point location
            double dx = FastMath.cos(angleRad) * spiralR;
            double dy = FastMath.sin(angleRad) * spiralR;
            double cPtX = dx + originalX;
            double cPtY = dy + originalY;
            // increment rotation angle, such that the next point on the spiral 
            // has an approximate distance of searchIncrement to the current point
            angleRad += searchIncrement / spiralR;

            /// test whether new control point position is within rangebox and 
            // does not result in a flow that is too close ot other flows or obstacles
            if (assignControlPointPositionIfAcceptable(flow, cPtX, cPtY, rangeBoxEnforcer, obstacles, minObstacleDistPx, true)) {
                return true;
            }
        } // move along the spiral until the entire range box is covered
        while (spiralR * spiralR < maxSpiralRadiusSquare);

        // Could not find good new position. Revert to original control point.
        flow.setCtrlPt(originalX, originalY);
        return false;
    }

    /**
     * Replaces the control point position with a new position if the new
     * position does not result in a flow that is too close to other flows,
     * intersects obstacles, or has the control point outside the range box.
     *
     * @param flow Flow to change
     * @param cPtX new control point position x
     * @param cPtY new control point position y
     * @param rangeBoxEnforcer range box that must contain the control point
     * @param obstacles obstacles to avoid
     * @param minObstacleDistPx minimum distance to obstacles in pixels
     * @param onlyTestWithLockedFlows if true, only flows that are locked are
     * considered when testing whether the new control point position results in
     * a flow that is too close to other (locked) flows.
     * @return
     */
    private boolean assignControlPointPositionIfAcceptable(Flow flow,
            double cPtX, double cPtY,
            RangeboxEnforcer rangeBoxEnforcer,
            List<Obstacle> obstacles,
            int minObstacleDistPx,
            boolean onlyTestWithLockedFlows) {

        double originalX = flow.cPtX();
        double originalY = flow.cPtY();
        if (rangeBoxEnforcer.isPointInRangebox(flow, cPtX, cPtY) == false) {
            return false;
        }

        flow.setCtrlPt(cPtX, cPtY);

        // test whether new control point position is too close to another flow
        double touchPercentage = largestTouchPercentage(flow, onlyTestWithLockedFlows);
        if (touchPercentage > Model.MAX_TOUCH_PERCENTAGE) {
            flow.setCtrlPt(originalX, originalY);
            return false;
        }

        // test whether new control point position results in a flow intersecting obstacles
        if (flowIntersectsObstacles(flow, obstacles, minObstacleDistPx)) {
            flow.setCtrlPt(originalX, originalY);
            return false;
        }

        return true;
    }

    /**
     * Moves control point to location that results in the smallest amount of
     * overlaps. The amount of overlaps is determined by the number of
     * overlapped obstacles and the closeness to other locked flows.
     *
     * @param flow flow
     * @param obstacles obstacles
     * @param minObstacleDistPx minimum distance between flow and obstacles
     * @return a quantification of the amount of overlaps. 0 means no overlaps,
     * larger numbers mean more overlaps.
     */
    private double findControlPointWithMinimumOverlapsInsideRangeBox(Flow flow,
            List<Obstacle> obstacles, int minObstacleDistPx) {

        double searchIncrement = searchIncrement();
        double originalX = flow.cPtX();
        double originalY = flow.cPtY();
        double angleRad = Math.PI;
        RangeboxEnforcer rangeBoxEnforcer = new RangeboxEnforcer(model);

        // search for position that results in smallest number of overlaps
        double minIntersectionIndex = Double.MAX_VALUE;
        double minNbrOverlapsX = originalX;
        double minNbrOverlapsY = originalY;

        // FIXME instead of spiral mvt, proceed in grid pattern along axes of range box
        double distToBasePointSquare = Double.MAX_VALUE;
        Point[] rangeBox = rangeBoxEnforcer.computeRangebox(flow);
        double maxSpiralRadiusSquare = rangeBoxEnforcer.longestDistanceSqToCorner(
                rangeBox, originalX, originalY);
        double spiralR;
        
        ArrayList<Obstacle> nodeObstacles = new ArrayList<>();
        for (Obstacle obstacle : obstacles) {
            if (obstacle.isNode()) {
                nodeObstacles.add(obstacle);
            }
        }
        
        do {
            // radius of spiral for the current angle.
            // The distance between two windings is searchIncrement.
            spiralR = searchIncrement * angleRad / Math.PI / 2;

            // new control point location
            double cPtX = FastMath.cos(angleRad) * spiralR + originalX;
            double cPtY = FastMath.sin(angleRad) * spiralR + originalY;
            // increment rotation angle, such that the next point on the spiral 
            // has an approximate distance of searchIncrement to the current point
            angleRad += searchIncrement / spiralR;

            if (rangeBoxEnforcer.isPointInRangebox(flow, cPtX, cPtY) == false) {
                continue;
            }
            
            if (flowIntersectsObstacles(flow, nodeObstacles, minObstacleDistPx)) {
                continue;
                
            }
            flow.setCtrlPt(cPtX, cPtY);

            // Compute the largest "touch percentage" of this flow with all 
            // other locked flows. The "touch percentage" is between 0 and 1. A
            // value of 0 indicates the flow is not touching any other. A value 
            // of 1 indicates this flow touches another flow along the entire 
            // length of this flow.
            boolean onlyTestWithLockedFlows = true;
            double largestTouchPercentage = largestTouchPercentage(flow, onlyTestWithLockedFlows);
            double touchW = 1 + largestTouchPercentage;

            // number of intersections of this flow with obstacles (ignoring arrowhead obstacles)
            double intersectionIndex = intersectionIndex(flow, obstacles,
                    minObstacleDistPx, 0d); // ignore intersecting arrowheads
            intersectionIndex *= touchW;
            if (intersectionIndex < minIntersectionIndex) {
                minNbrOverlapsX = cPtX;
                minNbrOverlapsY = cPtY;
                minIntersectionIndex = intersectionIndex;
            } else if (intersectionIndex == minIntersectionIndex) {
                // test whether this position with the same amount of overlaps 
                // is closer to the base point

                // option with Eucledian distance
                //double dsq = basePoint.distanceSquare(cPtX, cPtY);
                // option with Manhattan distance with axes aligned to base line.
                // The Manhatten distance should result in more symmetric flows.
                // FIXME it is not clear whether Manhattan distance is any better. Results with US commodity flows are identical for both distances.
                double p1 = flow.scalarProjectionOnBaselineRelativeToMidPoint(cPtX, cPtY);
                double dSqr = flow.getSquareDistanceToBaseLineMidPoint(cPtX, cPtY);
                double p2 = Math.sqrt(dSqr - p1 * p1);
                double dsq = p1 + p2;

                if (dsq < distToBasePointSquare) {
                    minNbrOverlapsX = cPtX;
                    minNbrOverlapsY = cPtY;
                    distToBasePointSquare = dsq;
//                  FIXME also do with only nodes (no arrowheads)
                }
            }
        } // move along the spiral until the entire range box is covered
        while (spiralR * spiralR < maxSpiralRadiusSquare);

        flow.setCtrlPt(minNbrOverlapsX, minNbrOverlapsY);

        return minIntersectionIndex;
    }

    /**
     * Computes the "touch percentage" of this flow with all other flows and
     * returns the largest value found. The "touch percentage" is between 0 and
     * 1. A value of 0 indicates that the flows are not touching. 1 indicates
     * this flow touches another flow along the entire length of this flow.
     *
     * @param flow flow to compute touch percentage for
     * @param onlyTestWithLockedFlows If true, the touch percentage will only be
     * computed for the passed flow and any other flow that is locked.
     * @return largest found touch percentage, between 0 and 1.
     */
    public double largestTouchPercentage(Flow flow, boolean onlyTestWithLockedFlows) {

        int minObstacleDistPx = model.getMinObstacleDistPx();
        Flow flow1 = model.clipFlowForComputations(flow);
        double flow1WidthPx = model.getFlowWidthPx(flow1);
        double referenceMapScale = model.getReferenceMapScale();

        Rectangle2D.Double boundingBox1 = flow1.getBoundingBox();

        double maxTouchPercentage = 0;
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow2 = iterator.next();
            if (flow == flow2) {
                continue;
            }

            if (onlyTestWithLockedFlows && flow2.isLocked() == false) {
                continue;
            }

            double flow2WidthPx = model.getFlowWidthPx(flow2);
            double minDistPx = minObstacleDistPx + (flow1WidthPx + flow2WidthPx) / 2;
            double minDist = minDistPx / referenceMapScale;

            flow2 = model.clipFlowForComputations(flow2);

            // test with distance between bounding boxes
            Rectangle2D.Double boundingBox2 = flow2.getBoundingBox();
            double bbDistSqr = GeometryUtils.rectDistSq(boundingBox1, boundingBox2);
            if (bbDistSqr < minDist * minDist) {
                int nbrPointsToTest = 10; // FIXME hard-coded parameter Model.MAX_TOUCH_PERCENTAGE should depend on this parameter
                double touchPercentage = flow.touchPercentage(flow2, minDist, nbrPointsToTest);
                maxTouchPercentage = Math.max(touchPercentage, maxTouchPercentage);
            }
        }

        return maxTouchPercentage;
    }

    /**
     * Move control points of flows overlapping obstacles
     *
     * @param obstacles obstacles to avoid
     * @param overlappingFlows flows overlapping an obstacle that are to be
     * moved
     * @param nbrFlowsToMove stop when this many flows have been moved
     * @return number of remaining flows that overlap an obstacle
     */
    public int moveFlowsAwayFromObstacles(List<Obstacle> obstacles,
            ArrayList<Flow> overlappingFlows, int nbrFlowsToMove) {
        int nbrMovedFlows = 0;
        for (int i = 0; i < Math.min(overlappingFlows.size(), nbrFlowsToMove); i++) {
            Flow flow = overlappingFlows.get(i);
            if (!flow.isLocked()) {
                moveFlowAwayFromObstacles(flow, obstacles);
                // lock the flow no matter whether it was moved or not
                flow.setLocked(true);
            }

            if (isCancelled()) {
                return 0;
            }
        }

        // return initial number of flows overlapping obstacles
        return overlappingFlows.size() - nbrMovedFlows;
    }

    /**
     * @param processMonitor the processMonitor to set
     */
    public void setProcessMonitor(ProcessMonitor processMonitor) {
        this.processMonitor = processMonitor;
    }

    private boolean isCancelled() {
        return (processMonitor == null) ? false : processMonitor.isCancelled();
    }

    /**
     * Makes skewed flows more symmetric and makes flows with long arrowheads
     * more straight.
     */
    public void symmetrizeFlows() {
        // nodes and arrowheads are obstacles
        List<Obstacle> obstacles = getObstaclesFromCachedCurves(model);

        RangeboxEnforcer rangeboxEnforcer = new RangeboxEnforcer(model);
        Iterator<Flow> flowIterator = model.flowIterator();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            if (flow.isLocked()) {
                continue;
            }
            symmetrizeFlow(flow, obstacles, rangeboxEnforcer);
        }
    }

    /**
     * Makes a skewed flow more symmetric, and makes a flow with a long
     * arrowhead more straight.
     *
     * @param flow flow
     * @param obstacles obstacles to avoid
     * @param rangeboxEnforcer to make the control point stay within the range
     * box
     */
    public void symmetrizeFlow(Flow flow, List<Obstacle> obstacles,
            RangeboxEnforcer rangeboxEnforcer) {

        Flow clippedFlow = model.clipFlow(flow, false, true, true);

        // Test whether control point is far away from the perpendicular 
        // axis on the base line passing through the point between the start
        // and the end point.
        // project control point onto base line
        double cPtX = flow.cPtX();
        double cPtY = flow.cPtY();
        double startPtMidPtDist = clippedFlow.scalarProjectionOnBaseline(cPtX, cPtY);
        double l = clippedFlow.getBaselineLength();
        double endPtMidPtDist = l - startPtMidPtDist;
        double ratio = Math.min(startPtMidPtDist, endPtMidPtDist) / l;
        if (ratio < 0.3) { // FIXME hard-coded parameter
            Point cPt = clippedFlow.getSymmetricControlPoint();
            // try a few positions between the symmetrical point and the original point
            for (int i = 5; i > 0; i--) { // FIXME hard-coded parameter
                double w = i / 5d;
                double x = w * cPt.x + (1d - w) * cPtX;
                double y = w * cPt.y + (1d - w) * cPtY;
                if (assignControlPointPositionIfAcceptable(flow, x, y,
                        rangeboxEnforcer, obstacles, model.getMinObstacleDistPx(), false)) {
                    break;
                }
            }
        }

        // FIXME the following is not really adding much
        // try straightening flows where the arrowhead is longer than the flow trunk
        if (model.isDrawArrowheads()) {
            if (flow.flowTrunkToFlowRatio(model) < 0.8) { // FIXME hard-coded parameter
                double mx = (clippedFlow.getStartPt().x + clippedFlow.getEndPt().x) / 2d;
                double my = (clippedFlow.getStartPt().y + clippedFlow.getEndPt().y) / 2d;
                if (assignControlPointPositionIfAcceptable(flow, mx, my,
                        rangeboxEnforcer, obstacles, model.getMinObstacleDistPx(), false)) {

                }
            }
        }
    }

}
