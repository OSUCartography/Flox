package edu.oregonstate.cartography.flox.gui;

import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import edu.oregonstate.cartography.utils.GeometryUtils;

/**
 *
 * @author danielstephen
 */
public class Arrow {

    private double arrowLength;
    private double arrowWidth;
    
    private QuadraticBezierFlow flow;
    public Point base;
    public Point tip = new Point(0,0);
    public Point corner1 = new Point(0,0);
    public Point corner2 = new Point(0,0);

    public QuadraticBezierFlow getFlow() {
        return flow;
    }
    
    public void setFlow(QuadraticBezierFlow flow) {
        this.flow = flow;
    }
    
    public Arrow(QuadraticBezierFlow flow, Model model) {
        
        // This is not going to give me something we want, pretty sure.
        // Still working on it.
        double value = flow.getValue();
        double scale = model.getFlowWidthScale();
        
        arrowLength = model.getArrowLength() * value * scale;
        arrowWidth = arrowLength * model.getArrowWidth();
        
        // Get the location on the flow where the base of the arrowhead will sit
        double t = flow.getIntersectionTWithCircleAroundEndPoint(arrowLength);
        base = flow.pointOnCurve(t);
        
        // Need to make a new flow that is cut back a bit to meet the base
        // of the arrowhead.
        QuadraticBezierFlow[] splitFlows = flow.split(t + ((1-t) * 0.1) );
        this.setFlow(splitFlows[0]);
        
        // Set the points of the arrowhead
        tip.x = base.x + arrowLength;
        tip.y = base.y;
        
        corner1.x = base.x;
        corner1.y = base.y + arrowWidth;
        
        corner2.x = base.x;
        corner2.y = base.y - arrowWidth;
        
        // Get the azimuth of the line connecting base to flow endpoint
        double azimuth = GeometryUtils.computeAzimuth(base, flow.getEndPt());
        
        tip = tip.rotatePoint(base, azimuth);
        corner1 = corner1.rotatePoint(base, azimuth);
        corner2 = corner2.rotatePoint(base, azimuth);
        
    }

}
