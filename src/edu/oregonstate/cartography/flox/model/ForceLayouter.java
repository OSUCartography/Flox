package edu.oregonstate.cartography.flox.model;

import java.util.ArrayList;
import java.util.Iterator;

/**
 *
 * @author danielstephen
 */
public class ForceLayouter {

    // Stores the model
    Model model;

    // Sets the model
    public void setModel(Model model) {
        this.model = model;
    }

    /**
     * Computes the total acting force on a point as applied by neighboring
     * points
     *
     * @param targetPoint The point upon which forces will be applied
     * @return
     */
    public double[] computeTotalForce(Point targetPoint) {

        Iterator<Point> nodeIterator = model.nodeIterator();

        double fxTotal = 0;
        double fyTotal = 0;
        double wTotal = 0;

        while (nodeIterator.hasNext()) {
            Point node = nodeIterator.next();
            
            double xDist = targetPoint.x - node.x; // x distance from node to target
            double yDist = targetPoint.y - node.y; // y distance from node to target
            double L = Math.sqrt( (xDist * xDist) + (yDist * yDist) ); // euclidean distance from node to target
            double w = 1/L; // weight of the distance?
            
            double fx = xDist / L; //normalized x distance
            double fy = yDist / L; //normalized y distance
            
            // weight force
            fx *= w;
            fy *= w;
            
            // sum force
            fxTotal += fx;
            fyTotal += fy;
            wTotal += w;
        }

        double fxFinal = fxTotal/wTotal;
        double fyFinal = fyTotal/wTotal;
        
        double[] totalForce = {fxFinal, fyFinal};
        
        return  totalForce;
    }

}
