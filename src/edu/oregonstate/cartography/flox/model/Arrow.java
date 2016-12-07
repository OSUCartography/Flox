package edu.oregonstate.cartography.flox.model;

import static edu.oregonstate.cartography.flox.model.Circle.TOL;
import edu.oregonstate.cartography.utils.GeometryUtils;

/**
 * The Arrow class contains algorithms for finding the shape, location, and
 * orientation of Arrow objects based on flows and parameters collected from the
 * model.
 *
 * @author danielstephen
 */
public final class Arrow {

    /**
     * The length of the arrowhead.
     */
    private final double arrowLength;

    /**
     * The width of the arrowhead.
     */
    private final double arrowWidth;

    /**
     * The position of the arrowheads corners relative to the base of the
     * arrowhead.
     */
    private final double arrowCornerPosition;

    /**
     * The flow that is passed in at instantiation.
     */
    private final Flow flow;

    /**
     * The location of the base of the Arrow in world coordinates.
     */
    private Point basePt = new Point(0, 0);

    /**
     * Radius around end node used to clip the end of the flow line. The base
     * point of the arrowhead is placed at the intersection of this circle and
     * the flow line.
     */
    private double clipRadius = 0;

    /**
     * The location of the tip of the Arrow.
     */
    private final Point tipPt = new Point(0, 0);

    /**
     * The locations of the 2 corners of the Arrow.
     */
    private final Point corner1Pt = new Point(0, 0);
    private final Point corner2Pt = new Point(0, 0);

    /**
     * The locations of the 2 control points that determine the curved shape of
     * the sides of the Arrow.
     */
    private final Point corner1cPt = new Point(0, 0);
    private final Point corner2cPt = new Point(0, 0);

    /**
     * Constructor for the Arrow. Computes the location of the points comprising
     * the arrow head based on the stroke width and azimuth of the flow.
     *
     * @param model model
     * @param flow The flow an arrow will be created for
     * @param endClipRadius the tip of the arrow is placed at this distance from
     * the end of the flow
     */
    public Arrow(Model model, Flow flow, double endClipRadius) {
        this.flow = flow;

        // stroke width in world coordinates of the flow based on its value.
        double flowStrokeWidth = model.getFlowWidthPx(flow) / model.getReferenceMapScale();

        // Gets the ratio of the flows stroke width to it's value. This ratio
        // is the same for all drawn flows.
        double valueToStrokeRatio = flowStrokeWidth / flow.getValue();

        // Get the max and min flow values, and the corresponding flow widths
        double maxFlowStrokeWidth = model.getMaxFlowValue() * valueToStrokeRatio;
        double minFlowStrokeWidth = model.getMinFlowValue() * valueToStrokeRatio;

        // Get the difference between this flow's stroke size and the biggest
        // stroke size.
        double strokeDiff = maxFlowStrokeWidth - flowStrokeWidth;

        // Get the difference between this flow's stroke size and the smallest
        // stroke size.
        double smallStrokeDiff = flowStrokeWidth - minFlowStrokeWidth;

        // Get a percentage of that difference based on valRatio
        double plusStroke = strokeDiff * (model.getArrowSizeRatio());

        // This much length will be subtracted from the lengths of arrowheads, 
        // proportionally to how relatively large they are compared to the 
        // smallest arrowhead. 
        // So the smallest arrowhead will have nothing subtracted, and the 
        // other arrowheads will have the difference between it and the smallest
        // arrowhead * model.getArrowLengthRatio (a number from 0 - 1) subtracted.
        double minusLength = smallStrokeDiff * (model.getArrowLengthRatio());

        // Determine the distance of the tip of the arrow from the base.
        // Is scaled to the value of the flow, which itself is scaled by the 
        // scale factor of the model.
        arrowLength = (flowStrokeWidth + plusStroke - minusLength)
                * model.getArrowLengthScaleFactor();

        // Determine the perpendicular distance of the corners of the arrow from 
        // a line drawn between the base and tip of the arrow.
        arrowWidth = (flowStrokeWidth + plusStroke)
                * model.getArrowWidthScaleFactor();

        // Get the arrowCornerPosition from the model. This value determines
        // the horizontal location of the corners of the Arrow
        arrowCornerPosition = model.getArrowCornerPosition();

        // Get the t value of the location on the flow where the base of the 
        // arrowhead will sit
        clipRadius = arrowLength + endClipRadius;
        double t = flow.getIntersectionTWithCircleAroundEndPoint(clipRadius);

        // Set the base of the Arrow to the point on the curve determined above.
        basePt = flow.pointOnCurve(t);

        // Locate the various points that determine the shape and location of 
        // the arrowhead.
        // Locate the tip
        tipPt.x = arrowLength;
        tipPt.y = 0;

        // Locate the first corner
        corner1Pt.x = arrowLength * arrowCornerPosition;
        corner1Pt.y = arrowWidth;

        // Locate the first control point
        corner1cPt.x = corner1Pt.x + ((tipPt.x - corner1Pt.x) * model.getArrowEdgeCtrlLength());
        corner1cPt.y = arrowWidth * model.getArrowEdgeCtrlWidth();

        // locate the second corner
        corner2Pt.x = arrowLength * arrowCornerPosition;
        corner2Pt.y = -arrowWidth;

        // locate the second control point
        corner2cPt.x = corner2Pt.x + ((tipPt.x - corner2Pt.x) * model.getArrowEdgeCtrlLength());
        corner2cPt.y = -arrowWidth * model.getArrowEdgeCtrlWidth();

        // Get the orientation of the line connecting the base of the arrow to the
        // endPoint of the flow.
        double arrowheadOrientation = GeometryUtils.orientation(basePt, flow.getEndPt());

        // Rotate and translate all the points that make up the shape of the Arrow, using
        // the Arrow's base point as the pivot.
        tipPt.transform(basePt.x, basePt.y, arrowheadOrientation);
        corner1Pt.transform(basePt.x, basePt.y, arrowheadOrientation);
        corner2Pt.transform(basePt.x, basePt.y, arrowheadOrientation);
        corner1cPt.transform(basePt.x, basePt.y, arrowheadOrientation);
        corner2cPt.transform(basePt.x, basePt.y, arrowheadOrientation);

        // For thick flows there is a small gap between the end of the clipped
        // line and the arrow base. This is due to the fact that the line cap 
        // and the base line of the arrow are aligned diffrently. The clipRadius
        // is reduced to compensate for this gap. The size of the gap is 
        // computed from the orientation of the line and the orientation of the arrow.
        Flow f = flow.split(t)[0];
        Point cPt = new Point(f.cPtX(), f.cPtY());
        double lineOrientation = GeometryUtils.orientation(cPt, getBasePt());
        double dAlpha = GeometryUtils.angleDif(lineOrientation, arrowheadOrientation);
        double dRadius = Math.abs(Math.tan(dAlpha) * flowStrokeWidth / 2);

        // Additionally reduce the clipping radius by 0.5 pixel to prevent a
        // rendering artifact that shows a small white line between end of the flow 
        // line and the arrowhead. This seems to be an antialiaising bug in Java2D.
        double tol = 0.5 / model.getReferenceMapScale();

        clipRadius -= Math.max(tol, dRadius);
    }

    /**
     * Tests whether this Arrow overlaps with another Arrow. The two arrows are
     * treated as triangles consisting of the tip point and the two corner
     * points.
     *
     * @param arrow arrow to test with
     * @return true if there is an overlap, false otherwise.
     */
    public boolean isOverlappingArrow(Arrow arrow) {
        return GeometryUtils.trianglesOverlap(tipPt.x, tipPt.y,
                corner1Pt.x, corner1Pt.y,
                corner2Pt.x, corner2Pt.y,
                arrow.tipPt.x, arrow.tipPt.y,
                arrow.corner1Pt.x, arrow.corner1Pt.y,
                arrow.corner2Pt.x, arrow.corner2Pt.y);
    }

    /**
     * Tests whether this Arrow overlaps with a Flow. The arrow is treated as a
     * triangle consisting of the tip point and the two corner points. The flow
     * width is taken into account.
     *
     * @param flow flow to test with
     * @param flowWidth width of the flow in world coordinates
     * @return true if there is an overlap, false otherwise.
     */
    public boolean isOverlappingFlow(Flow flow, double flowWidth) {
        // FIXME add test with bounding boxes?

        // FIXME abuse Bezier-Bezier intersection code. use Bezier-line intersection test instead.
        Flow cheesyTrickFlow1 = new Flow(tipPt, corner1Pt, 1);
        Flow cheesyTrickFlow2 = new Flow(tipPt, corner2Pt, 1);
        Flow cheesyTrickFlow3 = new Flow(corner1Pt, corner2Pt, 1);
        cheesyTrickFlow1.offsetCtrlPt(1, 1);
        cheesyTrickFlow2.offsetCtrlPt(1, 1);
        cheesyTrickFlow3.offsetCtrlPt(1, 1);
        
        // test whether the flow intersects the triangle formed by the arrow
        Point[] intersections = cheesyTrickFlow1.intersections(flow);
        if (intersections != null && intersections.length > 0) {
            return true;
        }
        intersections = cheesyTrickFlow2.intersections(flow);
        if (intersections != null && intersections.length > 0) {
            return true;
        }
        
        intersections = cheesyTrickFlow3.intersections(flow);
        if (intersections != null && intersections.length > 0) {
            return true;
        }

        // the center line of the flow does not intersect the arrow triangle,
        // but it may still overlay parts of the arrow. So test whether any 
        // triangle vertices overlap the flow band.
        double minDist = flowWidth / 2d;
        double minDistSqr = minDist * minDist;
        // FIXME distanceSq should exclude start and end points
        return flow.distanceSq(tipPt.x, tipPt.y, TOL) < minDistSqr
                || flow.distanceSq(corner1Pt.x, corner1Pt.y, TOL) < minDistSqr
                || flow.distanceSq(corner2Pt.x, corner2Pt.y, TOL) < minDistSqr;
    }

    /**
     * Returns the arrow length.
     *
     * @return the length
     */
    public double getLength() {
        return arrowLength;
    }

    /**
     * Returns the arrow width.
     *
     * @return the width
     */
    public double getWidth() {
        return arrowWidth;
    }

    /**
     * @return the basePt
     */
    public Point getBasePt() {
        return basePt;
    }

    /**
     * @return the tipPt
     */
    public Point getTipPt() {
        return tipPt;
    }

    /**
     * @return the corner1Pt
     */
    public Point getCorner1Pt() {
        return corner1Pt;
    }

    /**
     * @return the corner2Pt
     */
    public Point getCorner2Pt() {
        return corner2Pt;
    }

    /**
     * @return the corner1cPt
     */
    public Point getCorner1cPt() {
        return corner1cPt;
    }

    /**
     * @return the corner2cPt
     */
    public Point getCorner2cPt() {
        return corner2cPt;
    }

    /**
     * The base point of this arrowhead is placed at the intersection of a
     * circle with clip radius around the end node and the flow line.
     *
     * @return the clipRadius radius of the clip circle.
     */
    public double getClipRadius() {
        return clipRadius;
    }

    /**
     *
     * @return flow for this arrow
     */
    public Flow getFlow() {
        return flow;
    }

}
