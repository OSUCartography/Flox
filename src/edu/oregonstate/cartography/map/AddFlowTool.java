package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.util.Iterator;
import java.awt.Color;

/**
 *
 * @author danielstephen
 */
public class AddFlowTool extends MapTool {
    
    private final Model model;
    private boolean fromNodeSelected = false;
    Point fromNode;
    Point toNode;
    double pixelTolerance = 10;
    
    private boolean hadToMakeUpValue = false;
    private double newFlowValue = 2;
    
    public AddFlowTool(AbstractSimpleFeatureMapComponent mapComponent, Model model) {
        super(mapComponent);
        this.model = model;
    }
    
    @Override
    public void mouseClicked(Point2D.Double point, MouseEvent evt) {
        
        if (fromNodeSelected) {
            addToNode(point);
        } else {
            addFromNode(point);
        }
        
    }
    
    private void addFromNode(Point2D.Double point) {

        Iterator<Point> nodes = model.nodeIterator();
        while(nodes.hasNext()) {
            Point pt = nodes.next();
            
            // Was an existing node clicked?
            if (((mapComponent.xToPx(pt.x) >= mapComponent.xToPx(point.x) - pixelTolerance)
                    && (mapComponent.xToPx(pt.x) <= mapComponent.xToPx(point.x) + pixelTolerance))
                    && ((mapComponent.yToPx(pt.y) >= mapComponent.yToPx(point.y) - pixelTolerance)
                    && (mapComponent.yToPx(pt.y) <= mapComponent.yToPx(point.y) + pixelTolerance))) {
                
                // Set fromNode to it, select it
                fromNode = pt;
                //fromNode.setSelected(true);
                break;
            } else {                
                // Set fromNode to point, select it
                fromNode = new Point(point.x, point.y);
            }
        }

        if(fromNode == null) {
            fromNode = new Point(point.x, point.y);
        }
        
        // repaint the map
        fromNodeSelected = true;
        mapComponent.repaint();
    }
    
    private void addToNode(Point2D.Double point) {
        // add a to node
        // was an existing node clicked?
        Iterator<Point> nodes = model.nodeIterator();
        while(nodes.hasNext()) {
            Point pt = nodes.next();
            
            // Was an existing node clicked?
            if (((mapComponent.xToPx(pt.x) >= mapComponent.xToPx(point.x) - pixelTolerance)
                    && (mapComponent.xToPx(pt.x) <= mapComponent.xToPx(point.x) + pixelTolerance))
                    && ((mapComponent.yToPx(pt.y) >= mapComponent.yToPx(point.y) - pixelTolerance)
                    && (mapComponent.yToPx(pt.y) <= mapComponent.yToPx(point.y) + pixelTolerance))) {
                
                // Set toNode to it
                toNode = pt;
                break;
                
            } else {                
                // Set toNode to point, select it
                toNode = new Point(point.x, point.y);
            }
        }
        
        if(toNode == null) {
            toNode = new Point (point.x, point.y);
        }
        
        // build a flow from the toNode and the fromNode, add it to the model
        QuadraticBezierFlow newFlow = new QuadraticBezierFlow(fromNode, toNode);
        
        double maxFlowValue = model.getMaxFlowValue();
        
        // FIXME
        // Create a flow value that makes sense with the dataset. 
        // If hadToMakeUpValue is set to true (there were no flows when a flow
        // was added) and then a dataset IS added, then the tool needs to be
        // deselected and reselected to get a reasonable flow value for new
        // flows. Flow value should be settable when a flow is added eventually.
        if(hadToMakeUpValue) {
            newFlow.setValue(newFlowValue);
        } else {
            if(maxFlowValue == 0.0) {
                newFlow.setValue(newFlowValue);
                hadToMakeUpValue = true;
            } else {
                newFlow.setValue(maxFlowValue/2);
            }
        }
        
        // FIXME
        // Calling bend because when a newFlow is created, a control point at
        // an arbitrary location is set up. This was useful long ago, less so 
        // now. 
        newFlow.bend(0, 0);
        
        // FIXME test whether start and end point are identical to avoid 
        // exception due to the attempt of creating a loop
        model.addFlow(newFlow);
        
        // Set the longestFlowLength field needed for computing intermediate
        // nodes. 
        model.setLongestFlowLength();
        
        // repaint the map
        fromNodeSelected = false;
        mapComponent.repaint();
    }
    
    public void draw(Graphics2D g2d) {
        // If a from node is selected, and it didn't already exist, draw it
        if(fromNodeSelected) {
            double r = 10;
            g2d.setStroke(new BasicStroke(4));
            double x = mapComponent.xToPx(fromNode.x);
            double y = mapComponent.yToPx(fromNode.y);
            Ellipse2D circle = new Ellipse2D.Double(x-r, y-r, r*2, r*2);
            g2d.setColor(Color.WHITE);
            g2d.fill(circle);
            g2d.setColor(Color.RED);
            g2d.draw(circle);
        }
    }
}
