package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxRenderer;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.util.Iterator;
import java.awt.RenderingHints;
import java.awt.event.KeyEvent;
import java.util.ArrayList;

/**
 * Tool for adding flows.
 *
 * @author Dan Stephen
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
        // deselect all nodes and flows
        model.setSelectionOfAllFlowsAndNodes(false);
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

        double mapScale = mapComponent.getScale();
        double refScale = model.getReferenceMapScale();
        
        Iterator<Point> iterator = model.nodeIterator();
        while (iterator.hasNext()) {
            Point node = iterator.next();

            //get the radius of the node
            //FIXME this exact code for finding the radius is used in the 
            //selection tool and some other places. Maybe make a method in 
            //FloxMapComponent?
            double nodeArea = Math.abs(node.getValue())
                    * model.getNodeSizeScaleFactor();
            double nodePxRadius = (Math.sqrt(nodeArea / Math.PI)) * refScale;
            double nodeRadius = (nodePxRadius + PIXEL_TOLERANCE) / mapScale;

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
            double v = (model.getNbrNodes() > 0) ? model.getMeanNodeValue() : 0;
            originNode = new Point(point.x, point.y, v);
        }

        originNodeCreated = true;
        
        // repaint the map
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

        double mapScale = mapComponent.getScale();
        double refScale = model.getReferenceMapScale();
        
        ArrayList<Point> nodes = model.getNodes();
        for (int i = nodes.size() - 1; i >= 0; i--) {
            Point node = nodes.get(i);

            //get the radius of the node
            //FIXME this exact code for finding the radius is used in the 
            //selection tool and maybe other places. Perhaps make a method in 
            //FloxMapComponent.
            double nodeArea = Math.abs(node.getValue())
                    * model.getNodeSizeScaleFactor();
            double nodeRadius = (Math.sqrt(nodeArea / Math.PI)) * refScale;
            nodeRadius = (nodeRadius + PIXEL_TOLERANCE) / mapScale;

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
            double v = (model.getNbrNodes() > 0) ? model.getMeanNodeValue() : 0;
            destinationNode = new Point(point.x, point.y);
        }

        // Set the value of newFlow to the mean of existing flow values.
        // If no other flows exist, set the value of newFlow to 1.
        final double value;
        if (model.getNbrFlows() < 1) {
            value = 1;
        } else {
            value = model.getMeanFlowValue();
        }
         // build a flow from the toNode and the fromNode, add it to the model
        Flow newFlow = new Flow(originNode, destinationNode, value);

        // Straighten the new flow.
        // This is done because when a newFlow is created, the control point
        // is assigned a strange, arbitrary location.
        newFlow.straighten();

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
     *
     * @param g2d
     */
    @Override
    public void draw(Graphics2D g2d) {

        if (originNodeCreated) {
            FloxRenderer.enableHighQualityRenderingHints(g2d, true);
            double refScale = model.getReferenceMapScale();
            
            double nodeArea = Math.abs(originNode.getValue()) * model.getNodeSizeScaleFactor();
            double r = (Math.sqrt(nodeArea / Math.PI)) * refScale;
            double x = mapComponent.xToPx(originNode.x);
            double y = mapComponent.yToPx(originNode.y);
            Ellipse2D circle = new Ellipse2D.Double(x - r, y - r, r * 2, r * 2);
            g2d.setColor(FloxRenderer.NODE_FILL_COLOR);
            g2d.fill(circle);
            g2d.setColor(FloxRenderer.SELECTION_COLOR);
            g2d.setStroke(new BasicStroke(model.getNodeStrokeWidthPx()));
            g2d.draw(circle);
        }
    }

    /**
     * Treat key events. The event can be consumed (return true) or be delegated
     * to other listeners (return false).
     *
     * @param keyEvent The new key event.
     * @return True if the key event has been consumed, false otherwise.
     */
    @Override
    public boolean keyEvent(KeyEvent keyEvent) {
        if (originNodeCreated) {

            // treat backspace and delete key release events
            boolean keyReleased = keyEvent.getID() == KeyEvent.KEY_RELEASED;
            boolean isDeleteKey = keyEvent.getKeyCode() == KeyEvent.VK_DELETE;
            boolean isBackspaceKey = keyEvent.getKeyCode() == KeyEvent.VK_BACK_SPACE;
            if (keyReleased && (isDeleteKey || isBackspaceKey)) {
                // delete new origin node
                originNode = null;
                originNodeCreated = false;
                // repaint the map
                mapComponent.refreshMap();
                return true;
            }
        }

        // default: delegate key event to other components
        return false;
    }
}
