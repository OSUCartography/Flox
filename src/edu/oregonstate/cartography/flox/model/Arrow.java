package edu.oregonstate.cartography.flox.model;

import static edu.oregonstate.cartography.flox.model.Circle.TOL;
import edu.oregonstate.cartography.utils.GeometryUtils;
import net.jafama.FastMath;

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
        double dx = flow.getEndPt().x - basePt.x;
        double dy = flow.getEndPt().y - basePt.y;
        // dot product for computing cosine of rotation angle
        double l = Math.sqrt(dx * dx + dy * dy);
        double cos = dx / l;
        // cross product for computing sine of rotation angle
        double sin = dy / l;

        tipPt.transform(basePt.x, basePt.y, sin, cos);
        corner1Pt.transform(basePt.x, basePt.y, sin, cos);
        corner2Pt.transform(basePt.x, basePt.y, sin, cos);
        corner1cPt.transform(basePt.x, basePt.y, sin, cos);
        corner2cPt.transform(basePt.x, basePt.y, sin, cos);

        // For thick flows there is a small gap between the end of the clipped
        // line and the arrow base. This is due to the fact that the line cap 
        // and the base line of the arrow are aligned diffrently. The clipRadius
        // is reduced to compensate for this gap. The size of the gap is 
        // computed from the orientation of the line and the orientation of the arrow.
        Flow f = flow.split(t)[0];
        Point cPt = new Point(f.cPtX(), f.cPtY());
        double lineOrientation = GeometryUtils.orientation(cPt, getBasePt());
        double dAlpha = GeometryUtils.angleDif(lineOrientation, arrowheadOrientation);
        double dRadius = Math.abs(FastMath.tan(dAlpha) * flowStrokeWidth / 2);

        // Additionally reduce the clipping radius by 0.5 pixel to prevent a
        // rendering artifact that shows a small white line between end of the flow 
        // line and the arrowhead. This seems to be an antialiaising bug in Java2D.
        double tol = 0.5 / model.getReferenceMapScale();

        clipRadius -= Math.max(tol, dRadius);
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
