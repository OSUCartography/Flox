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
import java.util.ArrayList;

/**
 * Tool for adding flows.
 *
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
     * The origin node of the new flow being added. An originNode is assigned on
     * the first click of the AddFlowTool. If an existing node is clicked, that
     * Point is assigned to originNode. If an empty space is clicked, a new
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
     * node to be assigned to originNode or destinationNode. FIXME The clicking
     * distance should change with the size of the node.
     */
    private final double PIXEL_TOLERANCE = 3;

    /**
     * Constructor for AddFlowTool.
     *
     * @param mapComponent The current mapComponent.
     * @param model The model containing flow data and settings.
     */
    public AddFlowTool(AbstractSimpleFeatureMapComponent mapComponent, Model model) {
        super(mapComponent);
        this.model = model;
    }

    /**
     * Called after a mouse click.
     *
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
     * Adds an originNode to the map layout. Called on the first click while the
     * addFlowTool is active. If an existing node was clicked, that Point is
     * assigned to originNode. If a blank space was clicked, a new Point is
     * created and assigned to originNode.
     *
     * @param point The location of the new node.
     */
    private void addOriginNode(Point2D.Double point) {

        
        
        double scale = mapComponent.getScale();

        // Get the locked scale factor needed to calculate feature sizes
        double lockedScaleFactor;
        if (!model.isScaleLocked()) {
            lockedScaleFactor = 1;
        } else {
            // compare the locked scale to the current scale
            double lockedMapScale = model.getLockedMapScale();
            lockedScaleFactor = scale / lockedMapScale;
        }
        
        Iterator<Point> iterator = model.nodeIterator();
        while (iterator.hasNext()) {
            Point node = iterator.next();

            //get the radius of the node
            //FIXME this exact code for finding the radius is used in the 
            //selection tool and some other places. Maybe make a method in 
            //FloxMapComponent?
            double nodeArea = Math.abs(node.getValue())
                    * model.getNodeSizeScaleFactor();
            double nodePxRadius = (Math.sqrt(nodeArea / Math.PI)) * lockedScaleFactor;
            double nodeRadius = (nodePxRadius + PIXEL_TOLERANCE) / scale;
            
            // calculate the distance of the click from the node center
            double dx = node.x - point.x;
            double dy = node.y - point.y;
            double distSquared = (dx * dx + dy * dy);
            
            if (distSquared <= nodeRadius * nodeRadius) {
                originNode = node;
                
                // Stop checking nodes
                break;
            }
        }

        // If an existing node was NOT assigned to originNode, create a new one 
        // and assign it to originNode.
        if (originNode == null) {
            originNode = new Point(point.x, point.y);
            if(model.getNbrNodes() > 0) {
                originNode.setValue(model.getMeanNodeValue());
            } 
        }

        // repaint the map
        originNodeCreated = true;
        mapComponent.refreshMap();
    }

    /**
     * Adds a destinationNode to the map layout. Called on the second click
     * while the addFlowTool is active. If an existing node was clicked, that
     * Point is assigned to destinationNode. If a blank space was clicked, a new
     * Point is created and assigned to destinationNode.
     *
     * @param point The location of the new node.
     */
    private void addDestinationNode(Point2D.Double point) {

        double scale = mapComponent.getScale();
        
        // Get the locked scale factor needed to calculate feature sizes
        double lockedScaleFactor;
        if (!model.isScaleLocked()) {
            lockedScaleFactor = 1;
        } else {
            // compare the locked scale to the current scale
            double lockedMapScale = model.getLockedMapScale();
            lockedScaleFactor = scale / lockedMapScale;
        }
        
        ArrayList<Point> nodes = model.getNodes();
        for (int i = nodes.size() - 1; i >= 0; i--) {
            Point node = nodes.get(i);

            //get the radius of the node
            //FIXME this exact code for finding the radius is used in the 
            //selection tool and maybe other places. Perhaps make a method in 
            //FloxMapComponent.
            double nodeArea = Math.abs(node.getValue())
                    * model.getNodeSizeScaleFactor();
            double nodeRadius = (Math.sqrt(nodeArea / Math.PI)) * lockedScaleFactor;
            nodeRadius = (nodeRadius + PIXEL_TOLERANCE) / scale;
            
            // calculate the distance of the click from the node center
            double dx = node.x - point.x;
            double dy = node.y - point.y;
            double distSquared = (dx * dx + dy * dy);
            
            if (distSquared <= nodeRadius * nodeRadius) {
                destinationNode = node;
                
                // Stop checking nodes
                break;
            }
        }
        
        // If an existing node was NOT assigned to destinationNode, make a new
        // Point and assign it to destinationNode.
        if (destinationNode == null) {
            destinationNode = new Point(point.x, point.y);
            if(model.getNbrNodes() > 0) {
                destinationNode.setValue(model.getMeanNodeValue());
            }
        }
        
        // build a flow from the toNode and the fromNode, add it to the model
        QuadraticBezierFlow newFlow = new QuadraticBezierFlow(originNode, destinationNode);

        // Set the value of newFlow to the mean of existing flow values.
        // If no other flows exist, set the value of newFlow to 1.
        if (model.getNbrFlows() < 1) {
            newFlow.setValue(1);
        } else {
            newFlow.setValue(model.getMeanFlowValue());
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

        // Reinitialize flags, set origin and destination nodes to null. This
        // insures that new nodes will be assigned/created with successive
        originNodeCreated = false;
        originNode = null;
        destinationNode = null;

        // repaint the map
        mapComponent.refreshMap();
        // FIXME         model.addUndo("Add Flow");
    }

    /**
     * Draw the origin node after the first click and highlight it. If an
     * existing node was clicked, this draws a highlighted node of the same
     * radius on top of it.
     * @param g2d
     */
    public void draw(Graphics2D g2d) {

        if (originNodeCreated) {
            double lockedScaleFactor;
            if (!model.isScaleLocked()) {
                lockedScaleFactor = 1;
            } else {
                // compare the locked scale to the current scale
                double lockedMapScale = model.getLockedMapScale();
                lockedScaleFactor = mapComponent.getScale() / lockedMapScale;
            }
            
            double nodeArea = Math.abs(originNode.getValue())
                    * model.getNodeSizeScaleFactor();            
            double r = (Math.sqrt(nodeArea / Math.PI)) * lockedScaleFactor;
            
            g2d.setStroke(new BasicStroke(4));
            double x = mapComponent.xToPx(originNode.x);
            double y = mapComponent.yToPx(originNode.y);
            Ellipse2D circle = new Ellipse2D.Double(x - r, y - r, r * 2, r * 2);
            g2d.setColor(Color.WHITE);
            g2d.fill(circle);
            g2d.setColor(Color.RED);
            g2d.draw(circle);
        }
    }
}
