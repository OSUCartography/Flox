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
 * Tool for adding flows.
 * @author Dan
 */
public class AddFlowTool extends MapTool {
    
    /**
     * The data model containing all flows.
     */
    private final Model model;
    
    /**
     * Flag indicating that an origin node has been created. The next click 
     * should create/assign a destinationNode if this is true.
     */
    private boolean originNodeCreated = false;
    
    /**
     * The origin node of the new flow being added. An originNode is assigned
     * on the first click of the AddFlowTool. If an existing node is clicked, 
     * that Point is assigned to originNode. If an empty space is clicked, a new 
     * Point is created and assigned to originNode.
     */
    Point originNode;
    
    /**
     * The destination node of the new flow being added. A destinationNode is
     * assigned on the second click of the AddFlowTool. If an existing node is 
     * clicked, that Point is assigned to destinationNode. If an empty space is
     * clicked, a new Point is created and assigned to destinationNode.
     */
    Point destinationNode;
    
    /**
     * The distance from an existing node that a click must be within for that
     * node to be assigned to originNode or destinationNode.
     * FIXME The clicking distance should change with the size of the node.
     */
    double pixelTolerance = 10;
    
    /**
     * Flag for indicating that an arbitrary value was assigned to a new 
     * feature.
     * FIXME is this necessary?
     */
    private boolean hadToMakeUpValue = false;
    
    /**
     * An arbitrary value to be assigned to new features when there are no 
     * other features in the layout from which to derive new feature values. 
     */
    private double newFlowValue = 2;
    
    /**
     * Constructor for AddFlowTool.
     * @param mapComponent The current mapComponent.
     * @param model The model containing flow data and settings.
     */
    public AddFlowTool(AbstractSimpleFeatureMapComponent mapComponent, Model model) {
        super(mapComponent);
        this.model = model;
    }
    
    /**
     * Called after a mouse click. 
     * @param point The location of the click.
     * @param evt The mouse event.
     */
    @Override
    public void mouseClicked(Point2D.Double point, MouseEvent evt) {
        
        // If an origin node was assigned, add a destination node. Otherwise, 
        // add an origin node. Aka, if this is the first click, add an origin
        // node. If this is the second click, add a destination node.
        if (originNodeCreated) {
            addToNode(point);
        } else {
            addOriginNode(point);
        }
        
    }
    
    /**
     * Adds an originNode to the map layout. Called on the first click while
     * the addFlowTool is active.
     * @param point The location of the new node.
     */
    private void addOriginNode(Point2D.Double point) {

        destinationNode = null;
        Iterator<Point> nodes = model.nodeIterator();
        while(nodes.hasNext()) {
            Point pt = nodes.next();
            
            // Was an existing node clicked?
            // FIXME this calculation should account for node size, and can
            // be simplified by measuring destances rather than comparing
            // extents.
            if (((mapComponent.xToPx(pt.x) >= mapComponent.xToPx(point.x) - pixelTolerance)
                    && (mapComponent.xToPx(pt.x) <= mapComponent.xToPx(point.x) + pixelTolerance))
                    && ((mapComponent.yToPx(pt.y) >= mapComponent.yToPx(point.y) - pixelTolerance)
                    && (mapComponent.yToPx(pt.y) <= mapComponent.yToPx(point.y) + pixelTolerance))) {
                
                // Assign the point to destinationNode and break the loop
                originNode = pt;
                break;
            }
        }

        if(originNode == null) {
           originNode = new Point(point.x, point.y);
        }
        
        // repaint the map
        originNodeCreated = true;
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
                destinationNode = pt;
                break;
                
            } else {                
                // Set toNode to point, select it
                destinationNode = new Point(point.x, point.y);
            }
        }
        
        if(destinationNode == null) {
            destinationNode = new Point (point.x, point.y);
        }
        
        // build a flow from the toNode and the fromNode, add it to the model
        QuadraticBezierFlow newFlow = new QuadraticBezierFlow(originNode, destinationNode);
        
        double maxFlowValue = model.getMaxFlowValue();
        
        // FIXME
        // Create a flow value that makes sense with the dataset. 
        // If hadToMakeUpValue is set to true (there were no flows when a flow
        // was added) and then a dataset IS added, then the tool needs to be
        // deselected and reselected to get a reasonable flow value for new
        // flows. Flow value should be settable when a flow is added maybe.
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
        
        // update the canvas to include the new flow
        

        // repaint the map
        originNodeCreated = false;
        mapComponent.repaint();
        originNode = null;
    }
    
    public void draw(Graphics2D g2d) {
        // If a from node is selected, and it didn't already exist, draw it
        if(originNodeCreated) {
            double r = 10;
            g2d.setStroke(new BasicStroke(4));
            double x = mapComponent.xToPx(originNode.x);
            double y = mapComponent.yToPx(originNode.y);
            Ellipse2D circle = new Ellipse2D.Double(x-r, y-r, r*2, r*2);
            g2d.setColor(Color.WHITE);
            g2d.fill(circle);
            g2d.setColor(Color.RED);
            g2d.draw(circle);
        }
    }
}
