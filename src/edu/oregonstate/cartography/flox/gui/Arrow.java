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
     *
     */
    private double arrowLengthInWorldCoordinates;

    private double arrowLengthInPx;

    /**
     * The width of the arrowhead
     */
    private double arrowWidth;

    /**
     * The position of the arrowheads corners relative to the base of the
     * arrowhead
     */
    private double arrowCornerPosition;

    /**
     * Stores a flow, which is needed to determine the position of the Arrow
     * when it is drawn.
     */
    private QuadraticBezierFlow flow;

    /**
     * Returns the flow
     *
     * @return
     */
    public QuadraticBezierFlow getFlow() {
        return flow;
    }

    /**
     * Sets the flow
     *
     * @param flow
     */
    public void setFlow(QuadraticBezierFlow flow) {
        this.flow = flow;
    }

    /**
     * The location of the base of the Arrow in world coordinates. Used to split
     * the flow.
     */
    public Point baseInWorld;

    /**
     * The pixel location of th base of the arrow. Used to draw the arrow.
     */
    public Point baseInPx;

    /**
     * The location of the tip of the Arrow
     */
    public Point tip = new Point(0, 0);

    /**
     * The locations of the 2 corners of the Arrow
     */
    public Point corner1 = new Point(0, 0);
    public Point corner2 = new Point(0, 0);

    /**
     * The locations of the 2 control points that determine the curved shape of
     * the sides of the Arrow
     */
    public Point corner1cPt = new Point(0, 0);
    public Point corner2cPt = new Point(0, 0);

    /**
     * Constructor for the Arrow. Collects various parameters from the model and
     * locates the vertices of the arrow shape.
     *
     * @param flow
     * @param model
     */
    public Arrow(QuadraticBezierFlow flow, Model model, double flowStrokeWidth,
            double mapScale, double west, double north) {

        // Stores the value of the flow
        double value = flow.getValue();
        double valueToStrokeRatio = flowStrokeWidth / value;

        double maxValue = model.getMaxFlowValue();
        double maxFlowStrokeWidth = maxValue * valueToStrokeRatio;

        double valRatio = model.getArrowSizeRatio();

        double valDiff = maxValue - value;
        double plusVal = valDiff * valRatio;

        double strokeDiff = maxFlowStrokeWidth - flowStrokeWidth;
        double plusStroke = strokeDiff * valRatio;

        // Stores the scale factor that the model is currently applying to
        // the flow values to determine their width.
        //double flowWidthScale = model.getFlowWidthScale();
 

        // Determine the distance of the tip of the arrow from the base.
        // Is scaled to the value of the flow, which itself is scaled by the 
        // scale factor of the model.
        //arrowLength = (model.getShortestFlowLength()) * model.getArrowLengthScaleFactor() 
        //        * (value + plusVal) * flowWidthScale;
        arrowLengthInPx = (flowStrokeWidth + plusStroke)
                * model.getArrowLengthScaleFactor();

        arrowLengthInWorldCoordinates = arrowLengthInPx / mapScale;

        // Determine the perpendicular distance of the corners of the arrow from 
        // a line drawn between the base and tip of the arrow.
        arrowWidth = (flowStrokeWidth + plusStroke)
                * model.getArrowWidthScaleFactor();

        // Get the arrowCornerPosition from the model. This value determines
        // the horizontal location of the corners of the Arrow
        arrowCornerPosition = model.getArrowCornerPosition();

        // Get the t value of the location on the flow where the base of the 
        // arrowhead will sit
        double t = flow.getIntersectionTWithCircleAroundEndPoint(arrowLengthInWorldCoordinates);

        // Set the base of the Arrow to the point on the curve determined above.
        baseInWorld = flow.pointOnCurve(t);

        // get the pixel coordinates of base
        // Gonna just make a new version of xToPx and yToPx here. This would be
        // better if it was somehow able to use existing methods. 
        baseInPx = new Point((baseInWorld.x - west) * mapScale,
                (north - baseInWorld.y) * mapScale);

        // Split the flow at the base point of the Arrow, plus a little bit.
        // The little bit is to provide sufficient overlap of the flow with the
        // arrowhead to prevent gaps between the flow and arrowhead when the
        // arrowhead is drawn along more curved parts of the flow
        QuadraticBezierFlow[] splitFlows = flow.split(t + ((1 - t) * 0.1));

        // Set the flow to the section of the flow that travels from the 
        // start point to the base of the Arrow. The remaining section of the
        // flow will not be drawn.
        this.setFlow(splitFlows[0]);

        // Locate the various points that determine the shape and location of 
        // the Arrow. This pulls various parameters from the model that are 
        // themselves modified by the GUI to change the shape of the Arrows. 
        // Locate the tip
        tip.x = baseInPx.x + arrowLengthInPx;
        tip.y = baseInPx.y;

        // Locate the first corner
        corner1.x = baseInPx.x + (arrowLengthInPx * arrowCornerPosition);
        corner1.y = baseInPx.y + arrowWidth;

        // Locate the first control point
        corner1cPt.x = baseInPx.x + (corner1.x - baseInPx.x) + ((tip.x - corner1.x) * model.getArrowEdgeCtrlLength());
        corner1cPt.y = baseInPx.y + arrowWidth * model.getArrowEdgeCtrlWidth();

        // locate the second corner
        corner2.x = baseInPx.x + (arrowLengthInPx * arrowCornerPosition);
        corner2.y = baseInPx.y - arrowWidth;

        // locate the second control point
        corner2cPt.x = baseInPx.x + (corner2.x - baseInPx.x) + ((tip.x - corner2.x) * model.getArrowEdgeCtrlLength());
        corner2cPt.y = baseInPx.y - arrowWidth * model.getArrowEdgeCtrlWidth();

        // Get the azimuth of the line connecting the base of the arrow to the
        // endPoint of the flow. This determines the azimuth of the Arrow.
        double azimuth = GeometryUtils.computeAzimuth(baseInWorld, flow.getEndPt());

        // Rotate all the points that make up the shape of the Arrow, using
        // the Arrow's base point as the pivot.  
        tip = tip.rotatePoint(baseInPx, -azimuth);
        corner1 = corner1.rotatePoint(baseInPx, -azimuth);
        corner2 = corner2.rotatePoint(baseInPx, -azimuth);
        corner1cPt = corner1cPt.rotatePoint(baseInPx, -azimuth);
        corner2cPt = corner2cPt.rotatePoint(baseInPx, -azimuth);

    }

}
