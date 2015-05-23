package edu.oregonstate.cartography.flox.gui;

import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;

/**
 *
 * @author danielstephen
 */
public class Arrow {

    private double arrowLength;
    private double arrowWidth;
    
    private QuadraticBezierFlow flow;
    public Point base;
    public Point tip;
    public Point corner1;
    public Point corner2;

    
    public Arrow(QuadraticBezierFlow flow, Model model) {
        
        // This is not going to give me something we want, pretty sure.
        // Still working on it.
        arrowLength = (flow.getValue() * model.getFlowWidthScale()) * 1.5;
        arrowWidth = arrowLength * 0.5;
        
        // Get the location on the flow where the base of the arrowhead will sit
        double t = flow.getIntersectionTWithCircleAroundEndPoint(arrowLength);
        base = flow.pointOnCurve(t);
        
        // Need to make a new flow that is cut back a bit to meet the base
        // of the arrowhead.
        QuadraticBezierFlow[] splitFlows = flow.split(t + (t * 0.05));
        flow = splitFlows[0];
        
        // Set the points of the arrowhead
        tip.x = base.x + arrowLength;
        tip.y = base.y;
        
        corner1.x = base.x;
        corner1.y = base.y + arrowWidth;
        
        corner1.x = base.x;
        corner1.y = base.y - arrowWidth;
        
        
        

        
        
        
    }

}
