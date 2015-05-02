/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.oregonstate.cartography.flox.model;

import java.util.Iterator;

/**
 *
 * @author danielstephen
 */
public class FlowLayouter {
    
    // Access the model. It has the flows, so you have to ask it to give them
    // to you. 
    
    public BezierFlow bendFlow (BezierFlow flow, int angleDeg, int distPerc) {
        
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
