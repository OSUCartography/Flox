package edu.oregonstate.cartography.flox.model;

/**
 *
 * @author danielstephen
 */
public class FlowLayouter {
    
    public static CubicBezierFlow bendCubicFlow (Flow flow, int angleDeg, int distPerc) {
        
        // Convert angleDeg into radians
        double radians = angleDeg * (Math.PI / 180);
        
        //get the start and end points
        Point startPt = flow.getStartPt();
        Point endPt = flow.getEndPt();
        double value = flow.getValue();
        
        return new CubicBezierFlow(startPt, endPt, radians, distPerc, value);
    }
    
    public static QuadraticBezierFlow bendQuadraticFlow (Flow flow, int angleDeg, int distPerc) {
        
        // Convert angleDeg into radians
        double radians = angleDeg * (Math.PI / 180);
        
        //get the start and end points
        Point startPt = flow.getStartPt();
        Point endPt = flow.getEndPt();
        double value = flow.getValue();
        
        return new QuadraticBezierFlow(startPt, endPt, radians, distPerc, value);
    }
}
