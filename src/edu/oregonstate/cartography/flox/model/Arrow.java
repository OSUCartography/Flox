package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.utils.GeometryUtils;

/**
 * The Arrow class contains algorithms for finding the shape, location, and
 * orientation of Arrow objects based on flows and parameters collected from the
 * model.
 *
 * @author danielstephen
 */
public class Arrow {

    /**
     * The length of the arrowhead.
     */
    private double arrowLength;

    /**
     * The width of the arrowhead.
     */
    private double arrowWidth;

    /**
     * The position of the arrowheads corners relative to the base of the
     * arrowhead.
     */
    private double arrowCornerPosition;

    /**
     * The flow that is passed in at instantiation.
     */
    private final Flow flow;

    private double baseT = 1;

    /**
     * The location of the base of the Arrow in world coordinates.
     */
    private Point basePt;

    /**
     * The location of the tip of the Arrow.
     */
    private Point tipPt = new Point(0, 0);

    /**
     * The locations of the 2 corners of the Arrow.
     */
    private Point corner1Pt = new Point(0, 0);
    private Point corner2Pt = new Point(0, 0);

    /**
     * The locations of the 2 control points that determine the curved shape of
     * the sides of the Arrow.
     */
    private Point corner1cPt = new Point(0, 0);
    private Point corner2cPt = new Point(0, 0);

    /**
     * Constructor for the Arrow. Computes the location of the points comprising
     * the arrow head based on the stroke width and azimuth of the flow.
     *
     * @param flow The flow an arrow will be created for
     */
    public Arrow(Flow flow) {
        this.flow = flow;
    }

    /**
     * Computes the arrow head geometry.
     *
     * @param model model
     * @param flowStrokeWidth width of flow in world units.
     * @param endClipRadius the tip of the arrow is placed at this distance from
     * the end of the flow
     */
    public void computeArrowPoints(Model model, double flowStrokeWidth, double endClipRadius) {
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
        double t = flow.getIntersectionTWithCircleAroundEndPoint(arrowLength + endClipRadius);

        // Set the base of the Arrow to the point on the curve determined above.
        basePt = flow.pointOnCurve(t);

        // Split the flow at the base point of the Arrow, plus a little bit.
        // The little bit is to provide sufficient overlap of the flow with the
        // arrowhead to prevent gaps between the flow and arrowhead when the
        // arrowhead is drawn along more curved parts of the flow
        // TODO: This overlap is significantly less when using the other method
        // of orienting arrows, but some overlap is still required to avoid
        // a visible gap between the end of the flow and the arrowhead.
        if (model.isPointArrowTowardsEndpoint()) {
            baseT = t + ((1 - t) * 0.1);
        } else {
            baseT = t + ((1 - t) * 0.025);
        }

        // Locate the various points that determine the shape and location of 
        // the Arrow. This pulls various parameters from the model that are 
        // themselves modified by the GUI to change the shape of the Arrows. 
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

        // Get the azimuth of the line connecting the base of the arrow to the
        // endPoint of the flow. This determines the azimuth of the Arrow.
        // TODO: currently experminenting with a slightly different way of 
        // determining arrowhead orientation
        double azimuth;
        if (model.isPointArrowTowardsEndpoint()) {
            azimuth = GeometryUtils.computeAzimuth(getBasePt(), flow.getEndPt());
        } else {
            azimuth = GeometryUtils.computeAzimuth(getOutFlow().getCtrlPt(), getBasePt());
        }

        // Rotate all the points that make up the shape of the Arrow, using
        // the Arrow's base point as the pivot.
        tipPt.transform(basePt.x, basePt.y, azimuth);
        corner1Pt.transform(basePt.x, basePt.y, azimuth);
        corner2Pt.transform(basePt.x, basePt.y, azimuth);
        corner1cPt.transform(basePt.x, basePt.y, azimuth);
        corner2cPt.transform(basePt.x, basePt.y, azimuth);
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
     * Returns the outFlow, which is a clipped version of the inFlow. This is
     * the flow that will have an arrowhead attached to it.
     *
     * @return
     */
    public Flow getOutFlow() {
        return flow.split(baseT)[0];
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
}
