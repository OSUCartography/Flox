package edu.oregonstate.cartography.flox.model;

/**
 *
 * @author danielstephen
 */
public class FlowLayouter {
    
    public static BezierFlow bendFlow (Flow flow, int angleDeg, int distPerc) {
        
        // Convert angleDeg into radians
        double radians = angleDeg * (Math.PI / 180);
        
        //get the start and end points
        Point startPt = flow.getStartPt();
        Point endPt = flow.getEndPt();
        double value = flow.getValue();
        
        BezierFlow newFlow = new BezierFlow(startPt, endPt, radians, distPerc, value);
        return newFlow;
    }
}
