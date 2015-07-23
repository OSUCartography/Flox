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
     * feature. FIXME This exists because every time a new flow is created, it is 
     * assigned a value of half the maximum existing value. When there are no
     * other existing flows, an arbitrary value is assigned. It was weird then
     * for the second flow to be assigned a value of half the value of the 
     * first flow. There is probably a better way to manage this.
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
            addDestinationNode(point);
        } else {
            addOriginNode(point);
        }
    }
    
    /**
     * Adds an originNode to the map layout. Called on the first click while
     * the addFlowTool is active. If an existing node was clicked, that Point is
     * assigned to originNode. If a blank space was clicked, a new Point is 
     * created and assigned to originNode.
     * @param point The location of the new node.
     */
    private void addOriginNode(Point2D.Double point) {

        // Iterate over all nodes, checking to see if an existing node was
        // clicked.
        Iterator<Point> nodes = model.nodeIterator();
        while(nodes.hasNext()) {
            Point pt = nodes.next();
            // FIXME this calculation should account for node size, and can
            // be simplified by measuring destances rather than comparing
            // extents.
            if (((mapComponent.xToPx(pt.x) >= mapComponent.xToPx(point.x) - pixelTolerance)
                    && (mapComponent.xToPx(pt.x) <= mapComponent.xToPx(point.x) + pixelTolerance))
                    && ((mapComponent.yToPx(pt.y) >= mapComponent.yToPx(point.y) - pixelTolerance)
                    && (mapComponent.yToPx(pt.y) <= mapComponent.yToPx(point.y) + pixelTolerance))) {
                
                // Existing node was clicked; assign it to originNode
                originNode = pt;
                // Stop checking nodes
                break;
            }
        } 
        
        // If an existing node was NOT assigned to originNode, create a new one 
        // and assign it to originNode.
        if(originNode == null) {
           originNode = new Point(point.x, point.y);
        }
        
        // repaint the map
        originNodeCreated = true;
        mapComponent.repaint();
    }
    
    /**
     * Adds a destinationNode to the map layout. Called on the second click while
     * the addFlowTool is active. If an existing node was clicked, that Point is
     * assigned to destinationNode. If a blank space was clicked, a new Point is 
     * created and assigned to destinationNode.
     * @param point The location of the new node.
     */
    private void addDestinationNode(Point2D.Double point) {
        
        // Iterate over existing nodes to see if it was clicked.
        Iterator<Point> nodes = model.nodeIterator();
        while(nodes.hasNext()) {
            Point pt = nodes.next();
            
            // FIXME This needs to account for node size.
            if (((mapComponent.xToPx(pt.x) >= mapComponent.xToPx(point.x) - pixelTolerance)
                    && (mapComponent.xToPx(pt.x) <= mapComponent.xToPx(point.x) + pixelTolerance))
                    && ((mapComponent.yToPx(pt.y) >= mapComponent.yToPx(point.y) - pixelTolerance)
                    && (mapComponent.yToPx(pt.y) <= mapComponent.yToPx(point.y) + pixelTolerance))) {
                
                // An existing node was clicked; assign it to destinationNode.
                destinationNode = pt;
                
                // Stop checking nodes.
                break;
            } 
        }
        
        // If an existing node was NOT assigned to destinationNode, make a new
        // Point and assign it to destinationNode.
        if(destinationNode == null) {
            destinationNode = new Point (point.x, point.y);
        }
        
        // build a flow from the toNode and the fromNode, add it to the model
        QuadraticBezierFlow newFlow = new QuadraticBezierFlow(originNode, destinationNode);
        
        // Get the maximum value of all flows in the model.
        double maxFlowValue = model.getMaxFlowValue();
        
        // FIXME
        // Assign a value to the new flow that makes sense with the existing
        // dataset. 
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
        // Straighten the new flow.
        // This is done because when a newFlow is created, the control point
        // is assigned a strange, arbitrary location.
        newFlow.bend(0, 0);
        
        // FIXME test whether start and end point are identical to avoid 
        // exception due to the attempt of creating a loop
        
        // Add the new flow to the data model.
        model.addFlow(newFlow);
        
        // Update the longestFlowLength field in the model. This is needed for 
        // computing intermediate nodes. 
        // FIXME, could this be done by the model every time intermediate nodes 
        // are created?
        model.setLongestFlowLength();

        // Reinitialize flags, set origin and destination nodes to null. This
        // insures that new nodes will be assigned/created with successive
        // clicks. 
        originNodeCreated = false;
        originNode = null;
        destinationNode = null;
        
        // repaint the map
        mapComponent.repaint();
    }
    
    /**
     * Draw the 
     * @param g2d 
     */
    public void draw(Graphics2D g2d) {
        // If a destinationNode is selected, and it didn't already exist, draw it
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
