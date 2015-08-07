package edu.oregonstate.cartography.flox.gui;

import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import edu.oregonstate.cartography.utils.GeometryUtils;
import java.awt.geom.GeneralPath;

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
    private double arrowLengthInWorld;

    /**
     * The width of the arrowhead.
     */
    private double arrowWidthInWorld;

    /**
     * The position of the arrowheads corners relative to the base of the
     * arrowhead.
     */
    private double arrowCornerPosition;

    /**
     * Stores a flow, which is needed to determine the position of the Arrow
     * when it is drawn. This flow is a clipped version of the flow that is
     * passed into the Arrow class, and is returnable by getOutFlow().
     */
    private QuadraticBezierFlow outFlow;

    /**
     * The QuadraticBezierFlow that is passed in at instantiation.
     */
    private QuadraticBezierFlow inFlow;

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

    private double west;
    private double north;
    private double scale;
    private double flowStrokeWidth;
    private Model model;

    /**
     * Constructor for the Arrow. Computes the location of the points comprising
     * the arrow head based on the stroke width and azimuth of the flow.
     *
     * @param inFlow The flow an arrow will be created for
     * @param model The complete data model. Needed for scaling up the smallest
     * arrows
     * @param flowStrokeWidth Determines the size of the arrow
     * @param mapScale Needed for scaling to pixel values
     * @param west Needed for scaling to pixel values
     * @param north Needed for scaling to pixel values
     */
    public Arrow(QuadraticBezierFlow inFlow, Model model, double flowStrokeWidth,
            double scale, double west, double north) {

        this.west = west;
        this.north = north;
        this.scale = scale;
        this.flowStrokeWidth = flowStrokeWidth;
        this.inFlow = inFlow;
        this.model = model;

        computeArrowPoints();

    }

    private void computeArrowPoints() {

        // Gets the ratio of the flows stroke width to it's value. This ratio
        // is the same for all drawn flows.
        double valueToStrokeRatio = flowStrokeWidth / inFlow.getValue();

        // Get the max flow value, and get the stroke value of that width 
        // based on the valueToStrokeRatio
        double maxFlowStrokeWidth = model.getMaxFlowValue() * valueToStrokeRatio;

        // Get the differnce between this flow's stroke size and the biggest
        // stroke size.
        double strokeDiff = maxFlowStrokeWidth - flowStrokeWidth;

        // Get a percentage of that difference based on valRatio
        double plusStroke = strokeDiff * (model.getArrowSizeRatio());

        // Determine the distance of the tip of the arrow from the base.
        // Is scaled to the value of the flow, which itself is scaled by the 
        // scale factor of the model.
        double arrowLengthInPx = (flowStrokeWidth + plusStroke)
                * model.getArrowLengthScaleFactor();

        arrowLengthInWorld = arrowLengthInPx / scale;

        // Determine the perpendicular distance of the corners of the arrow from 
        // a line drawn between the base and tip of the arrow.
        double arrowWidthInPx = (flowStrokeWidth + plusStroke)
                * model.getArrowWidthScaleFactor();

        arrowWidthInWorld = arrowWidthInPx / scale;

        // Get the arrowCornerPosition from the model. This value determines
        // the horizontal location of the corners of the Arrow
        arrowCornerPosition = model.getArrowCornerPosition();

        // Get the t value of the location on the flow where the base of the 
        // arrowhead will sit
        double t = inFlow.getIntersectionTWithCircleAroundEndPoint(arrowLengthInWorld);

        // Set the base of the Arrow to the point on the curve determined above.
        basePt = inFlow.pointOnCurve(t);

        // Split the flow at the base point of the Arrow, plus a little bit.
        // The little bit is to provide sufficient overlap of the flow with the
        // arrowhead to prevent gaps between the flow and arrowhead when the
        // arrowhead is drawn along more curved parts of the flow
        QuadraticBezierFlow[] splitFlows = inFlow.split(t + ((1 - t) * 0.1));

        // Set the flow to the section of the flow that travels from the 
        // start point to the base of the Arrow. The remaining section of the
        // flow will not be drawn.
        this.setOutFlow(splitFlows[0]);

        // Locate the various points that determine the shape and location of 
        // the Arrow. This pulls various parameters from the model that are 
        // themselves modified by the GUI to change the shape of the Arrows. 
        // Locate the tip
        tipPt.x = getBasePt().x + arrowLengthInWorld;
        tipPt.y = getBasePt().y;

        // Locate the first corner
        corner1Pt.x = getBasePt().x + (arrowLengthInWorld * arrowCornerPosition);
        corner1Pt.y = getBasePt().y + arrowWidthInWorld;

        // Locate the first control point
        corner1cPt.x = getBasePt().x + (getCorner1Pt().x - getBasePt().x) + ((getTipPt().x - getCorner1Pt().x) * model.getArrowEdgeCtrlLength());
        corner1cPt.y = getBasePt().y + arrowWidthInWorld * model.getArrowEdgeCtrlWidth();

        // locate the second corner
        corner2Pt.x = getBasePt().x + (arrowLengthInWorld * arrowCornerPosition);
        corner2Pt.y = getBasePt().y - arrowWidthInWorld;

        // locate the second control point
        corner2cPt.x = getBasePt().x + (getCorner2Pt().x - getBasePt().x) + ((getTipPt().x - getCorner2Pt().x) * model.getArrowEdgeCtrlLength());
        corner2cPt.y = getBasePt().y - arrowWidthInWorld * model.getArrowEdgeCtrlWidth();

        // Get the azimuth of the line connecting the base of the arrow to the
        // endPoint of the flow. This determines the azimuth of the Arrow.
        double azimuth = GeometryUtils.computeAzimuth(getBasePt(), inFlow.getEndPt());

        // Rotate all the points that make up the shape of the Arrow, using
        // the Arrow's base point as the pivot.  
        tipPt = getTipPt().rotatePoint(getBasePt(), azimuth);
        corner1Pt = getCorner1Pt().rotatePoint(getBasePt(), azimuth);
        corner2Pt = getCorner2Pt().rotatePoint(getBasePt(), azimuth);
        corner1cPt = getCorner1cPt().rotatePoint(getBasePt(), azimuth);
        corner2cPt = getCorner2cPt().rotatePoint(getBasePt(), azimuth);
    }

    /**
     * Generate a GeneralPath of the outline of the arrowhead.
     *
     * @return GeneralPath of the arrowhead.
     */
    public GeneralPath getArrowPath() {

        GeneralPath arrowPath = new GeneralPath();

        arrowPath.moveTo(xToPx(getBasePt().x), yToPx(getBasePt().y));
        
        arrowPath.lineTo((xToPx(getCorner1Pt().x)), (yToPx(getCorner1Pt().y)));
        
        arrowPath.quadTo(xToPx(getCorner1cPt().x), yToPx(getCorner1cPt().y),
                xToPx(getTipPt().x), yToPx(getTipPt().y));
        
        arrowPath.quadTo(xToPx(getCorner2cPt().x), yToPx(getCorner2cPt().y),
                xToPx(getCorner2Pt().x), yToPx(getCorner2Pt().y));
        
        arrowPath.lineTo(xToPx(getBasePt().x), yToPx(getBasePt().y));

        return arrowPath;
    }

    /**
     * Returns the outFlow, which is a clipped version of the inFlow. This is
     * the flow that will have an arrowhead attached to it.
     *
     * @return
     */
    public QuadraticBezierFlow getOutFlow() {
        return outFlow;
    }

    /**
     * Sets the outFlow
     *
     * @param flow
     */
    public void setOutFlow(QuadraticBezierFlow flow) {
        this.outFlow = flow;
    }

    public double xToPx(double x) {
        return (x - west) * scale;
    }

    public double yToPx(double y) {
        return (north - y) * scale;
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
