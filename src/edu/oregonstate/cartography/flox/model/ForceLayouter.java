package edu.oregonstate.cartography.flox.model;

import java.util.Iterator;

/**
 *
 * @author danielstephen
 */
public class ForceLayouter {

    private static final double K = 0.5;
    private static final double B = 4;

    // Stores the model
    private Model model;
    
    public ForceLayouter(Model model) {
        this.model = model;
    }
    

    /**
     * Computes the total acting force on a point as applied by neighboring
     * points
     *
     * @param targetPoint The point upon which forces will be applied
     * @return
     */
    public void computeTotalForce(Point targetPoint, Point referencePoint) {

        Iterator<Point> nodeIterator = model.nodeIterator();

        double fxTotal = 0;
        double fyTotal = 0;
        double wTotal = 0;

        while (nodeIterator.hasNext()) {
            Point node = nodeIterator.next();

            double xDist = targetPoint.x - node.x; // x distance from node to target
            double yDist = targetPoint.y - node.y; // y distance from node to target
            double l = Math.sqrt((xDist * xDist) + (yDist * yDist)); // euclidean distance from node to target
            // FIXME length of 0 causes division by 0
            double w = Math.pow(l, -B); // weight of the distance?

            double fx = xDist / l; //normalized x distance
            double fy = yDist / l; //normalized y distance

            // weight force
            fx *= w;
            fy *= w;

            // sum force
            fxTotal += fx;
            fyTotal += fy;
            wTotal += w;
        }

        double fxFinal = fxTotal / wTotal;
        double fyFinal = fyTotal / wTotal;

        double springLengthX = referencePoint.x - targetPoint.x;
        double springLengthY = referencePoint.y - targetPoint.y;
        double springForceX = K * springLengthX;
        double springForceY = K * springLengthY;
        
        double totalForceX = fxFinal + springForceX;
        double totalForceY = fyFinal + springForceY;
        
        double dx = totalForceX;
        double dy = totalForceY;
        targetPoint.x += dx;
        targetPoint.y += dy;
    }

}
