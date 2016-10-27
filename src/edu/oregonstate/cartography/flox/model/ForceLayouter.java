package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.PrecisionModel;
import edu.oregonstate.cartography.utils.GeometryUtils;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

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
     * Updates the approximative line string of each flow.
     */
    private void updateCachedPolylineApproximations() {
        double segmentLength = model.segmentLength();
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            iter.next().updateCachedPolylineApproximation(model, segmentLength);
            if (isCancelled()) {
                return;
            }
        }
    }

    /**
     * Apply now control point locations to a model. This method should be
     * called when all iterations are completed and a new layout has been
     * generated.
     *
     * @param destinationModel the graph of this model will be replaced with a
     * reference to the model of this ForceLayouter.
     */
    public void applyChangesToModel(Model destinationModel) {
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            destinationModel.replaceControlPoint(flow.id, flow.cPtX(), flow.cPtY());
        }
    }

    /**
     * Compute one iteration, including forces computation, moving overlapping
     * sibling flows, moving nodes from obstacles, and constraining control
     * points to range boxes.
     *
     * @param i current iteration, between 0 and ForceLayouter.NBR_ITERATIONS
     * @param iterBeforeMovingFlowsOffObstacles number of remaining iterations
     * until flows are moved away from obstacles
     * @param canvas control points are to be constrained to this rectangle
     * @return number of remaining iterations until flows are moved away from
     * obstacles
     */
    public int layoutIteration(int i, int iterBeforeMovingFlowsOffObstacles,
            Rectangle2D canvas) {
        RangeboxEnforcer enforcer = new RangeboxEnforcer(model);

        // compute one iteration of forces with a linearly decreasing weight
        double weight = 1d - (double) i / model.getNbrIterations();
        computeForces(weight, canvas);

        // try moving flows that intersect and are connected to the same node
        if (model.isResolveIntersectionsForSiblings()) {
            ArrayList<Model.IntersectingFlowPair> pairs = getSortedIntersectingSiblings();
            if (isCancelled()) {
                return 0;
            }

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
            int remainingIterations = model.getNbrIterations() - i - 1;

            // nodes and arrowheads are obstacles
            List<Obstacle> obstacles = model.getObstacles();

            // get a list of sorted flows that intersect obstacles
            ArrayList<Flow> sortedOverlappingFlows = getFlowsOverlappingObstacles(obstacles);
            sortedOverlappingFlows.sort(Collections.reverseOrder());
            int nbrOverlaps = sortedOverlappingFlows.size();

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
                iterBeforeMovingFlowsOffObstacles = (model.getNbrIterations() - i) / (nbrRemainingOverlaps + 1) / 2;
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

        updateCachedPolylineApproximations();

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
            dx += angularDistWeight * angularDistForce.fy;

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

            Point[] points = flow.getCachedPolylineApproximation();
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
        double alpha = Math.atan2(dy, dx);
        double baseLineAzimuth = flow.getBaselineOrientation();
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
     * @param flow flow to compute force for.
     * @param maxFlowLength FIXME
     * @param on output outForce the force that is exerted onto the control
     * point.
     */
    private void computeForceOnFlow(Flow flow, double maxFlowLength, Force outForce) {

        Point basePt = flow.getBaseLineMidPoint();
        Point[] flowPoints = flow.getCachedPolylineApproximation();

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
        double tol = 1 / model.getReferenceMapScale(); // 1 pixel in world coordinates
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
            double d = flow.distance(xy, tol);
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
        updateCachedPolylineApproximations();
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
                if (sharedNode != null && flow1.intersects(flow2)) {
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
     * Returns true if a flow intersects an obstacle.
     *
     * @param flow a flow.
     * @param obstacle obstacle
     * @return
     */
    private boolean flowIntersectsObstacle(Flow flow, Obstacle obstacle) {
        double tol = 1d / model.getReferenceMapScale(); // 1 pixel in world coordinates

        // flow width in world coordinates
        double strokeWidthWorld = model.getFlowWidthPx(flow) / model.getReferenceMapScale();

        // obstacle radius is in world coordinates
        // add minimum obstacle distance
        double obstacleRadiusWorld = obstacle.r
                + model.getMinObstacleDistPx() / model.getReferenceMapScale();

        // the minimum distance between the obstacle center and the flow axis
        double minDist = (strokeWidthWorld / 2) + obstacleRadiusWorld;

        // test with flow bounding box
        // extend bounding box by minDist.
        Rectangle2D flowBB = flow.getBoundingBox();
        flowBB.add((flowBB.getMinX() - minDist), (flowBB.getMinY() - minDist));
        flowBB.add((flowBB.getMaxX() + minDist), (flowBB.getMaxY() + minDist));

        // the obstacle's circle center must be inside the extended bounding box
        if (flowBB.contains(obstacle.x, obstacle.y) == false) {
            return false;
        }

        // Check the shortest distance between the obstacle and the flow. If it's 
        // less than the minimum distance, then the flow intersects the obstacle. 
        double shortestDistSquare;
        if (flow instanceof FlowPair) {
            shortestDistSquare = 0;
        } else {
            shortestDistSquare = flow.distanceSq(obstacle.x, obstacle.y, tol);
        }        
        return shortestDistSquare < minDist * minDist;
    }

    /**
     * Tests whether a flow overlaps any obstacle.
     *
     * @param flow the flow to test
     * @param obstacles circular obstacles
     * @return true if the flow overlaps a node
     */
    private boolean flowIntersectsObstacle(Flow flow, List<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles) {

            // ignore obstacles that are start or end nodes of the flow
            if (obstacle.node == flow.getStartPt() || obstacle.node == flow.getEndPt()) {
                continue;
            }

            // ignore arrowhead attached to this flow
            if (model.isDrawArrowheads() && flow == obstacle.flow) {
                continue;
            }

            Flow clippedFlow = model.clipFlow(flow, false, true);
            if (flowIntersectsObstacle(clippedFlow, obstacle)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns a list of all flows that intersect obstacles.
     *
     * @return a list of flows overlapping any of the passed obstacles
     */
    public ArrayList<Flow> getFlowsOverlappingObstacles() {
        List<Obstacle> obstacles = model.getObstacles();
        return getFlowsOverlappingObstacles(obstacles);
    }

    /**
     * Returns a list of all flows that intersect a list of passed obstacles.
     *
     * @param obstacles a list of obstacles
     * @return a list of flows overlapping any of the passed obstacles
     */
    private ArrayList<Flow> getFlowsOverlappingObstacles(List<Obstacle> obstacles) {
        ArrayList<Flow> flowsArray = new ArrayList();
        Iterator<Flow> flowIterator = model.flowIterator();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            if (flowIntersectsObstacle(flow, obstacles)) {
                flowsArray.add(flow);
            }
        }
        return flowsArray;
    }

    /**
     * Moves the control point location such that the flow does not overlap any
     * obstacle. If no position can be found, the control point is not changed.
     * Tests control point locations placed along an Archimedean spiral centered
     * on the current control point location.
     *
     * @param flow flow to change. Nothing is changed if the flow is locked.
     * @param obstacles obstacles to avoid
     * @return true if a new position was found, false otherwise.
     */
    private boolean moveFlowAwayFromObstacles(Flow flow, List<Obstacle> obstacles) {
        if (flow.isLocked()) {
            return false;
        }
        // compute spacing of sample points in world coordinates
        // The spacing between candidate control points is equal to the minimum
        // distance to obstacles. Increase to accelerate computations.
        // minObstacleDistPx can be zero. We require a distance of at least 3 
        // pixel to move along the spiral.
        double minObstacleDistPx = Math.max(model.getMinObstacleDistPx(), 3);
        double dist = minObstacleDistPx / model.getReferenceMapScale();

        double originalX = flow.cPtX();
        double originalY = flow.cPtY();
        double angleRad = Math.PI;
        RangeboxEnforcer rangeBoxEnforcer = new RangeboxEnforcer(model);

        // compute the maximum possible radius for the spiral
        // the maximum radius is the distance between the control point (which
        // is the center of the spiral) and the corner point of the range box
        // that is the farthest away from the control point.
        Point[] rangeBox = rangeBoxEnforcer.computeRangebox(flow);
        double maxSpiralRSq = rangeBoxEnforcer.longestDistanceSqToCorner(rangeBox, originalX, originalY);

        // place the control point along the spiral until a position is found 
        // that does not result in any overlap
        double spiralR;
        do {
            // radius of spiral for the current angle.
            // The distance between two windings is dist.
            spiralR = dist * angleRad / Math.PI / 2;

            // new control point location
            double dx = Math.cos(angleRad) * spiralR;
            double dy = Math.sin(angleRad) * spiralR;
            double cPtX = dx + originalX;
            double cPtY = dy + originalY;
            flow.setCtrlPt(cPtX, cPtY);
            
            // increment rotation angle, such that the next point on the spiral 
            // has an approximate distance of dist to the current point
            angleRad += dist / spiralR;

            if (rangeBoxEnforcer.isPointInRangebox(flow, cPtX, cPtY)
                    && flowIntersectsObstacle(flow, obstacles) == false) {
                // found a new position for the control point that does not 
                // result in an overlap with any obstacle
                return true;
            }
        } // move along the spiral until the entire range box is covered
        while (spiralR * spiralR < maxSpiralRSq);

        // could not find a control point position that does not overlap an 
        // obstacle. Restore the original coordinates.
        flow.setCtrlPt(originalX, originalY);
        
        return false;
    }

    /**
     * Move control points of flows overlapping obstacles
     *
     * @param obstacles obstacles to avoid
     * @param flows flows overlapping an obstacle that are to be moved
     * @param nbrFlowsToMove stop when this many flows have been moved
     * @return number of remaining flows that overlap an obstacle
     */
    private int moveFlowsAwayFromObstacles(List<Obstacle> obstacles,
            ArrayList<Flow> flows, int nbrFlowsToMove) {
        int nbrMovedFlows = 0;
        for (Flow flow : flows) {
            if (moveFlowAwayFromObstacles(flow, obstacles)) {
                // moved one flow. Lock it.
                flow.setLocked(true);
                if (++nbrMovedFlows == nbrFlowsToMove) {
                    break;
                }
            }

            if (isCancelled()) {
                return 0;
            }
        }

        // return initial number of flows overlapping obstacles
        return flows.size() - nbrMovedFlows;
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

}
