package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxMapComponent;
import edu.oregonstate.cartography.flox.gui.FloxRenderer;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.geom.Point2D;
import java.awt.event.KeyEvent;
import java.awt.geom.GeneralPath;
import java.util.ArrayList;

/**
 * Tool for adding flows.
 *
 * @author Dan Stephen
 */
public class AddFlowTool extends DoubleBufferedTool {

    /**
     * The data model containing all flows.
     */
    private final Model model;

    /**
     * The origin node of the new flow being added. An originNode is assigned on
     * the first click of the AddFlowTool. If an existing node is clicked, that
     * Point is assigned to originNode. If an empty space is clicked, a new
     * Point is created and assigned to originNode.
     */
    Point originNode = null;

    /**
     * The destination node of the new flow being added. A destinationNode is
     * assigned on the second click of the AddFlowTool. If an existing node is
     * clicked, that Point is assigned to destinationNode. If an empty space is
     * clicked, a new Point is created and assigned to destinationNode.
     */
    Point destinationNode = null;

    /**
     * The distance from an existing node that a click must be within for that
     * node to be assigned to originNode or destinationNode. FIXME The clicking
     * distance should change with the size of the node.
     */
    private final double PIXEL_TOLERANCE = 3;

    /**
     * The default value for a new node.
     */
    private static final int DEFAULT_NODE_VALUE = 1;

    /**
     * The default value for a flow.
     */
    private static final int DEFAULT_FLOW_VALUE = 1;

    /**
     * current mouse position in world coordinates
     */
    private Point2D mouse = null;

    /**
     * the value of the flow that will be created
     */
    private double flowValue;

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
        if (originNode != null) {
            ArrayList<Point> nodes = model.getNodes();
            destinationNode = ((FloxMapComponent) mapComponent).getClickedNode(nodes, point, PIXEL_TOLERANCE);

            // If an existing node was NOT assigned to destinationNode, make a new
            // Point and assign it to destinationNode.
            if (destinationNode == null) {
                destinationNode = new Point(point.x, point.y, originNode.getValue());
            }

            // Add the new flow to the data model.
            if (destinationNode != originNode) {
                model.addFlow(new Flow(originNode, destinationNode, flowValue));
            }

            // Reinitialize flags, set origin and destination nodes to null. This
            // insures that new nodes will be assigned/created with successive
            originNode = null;
            destinationNode = null;

            releaseBackground();
            
            // update the force-based layout and add undo option
            ((FloxMapComponent) mapComponent).layout("Add Flow");

        } else {
            
            
            ArrayList<Point> nodes = model.getNodes();
            originNode = ((FloxMapComponent) mapComponent).getClickedNode(nodes, point, PIXEL_TOLERANCE);

            // If an existing node was NOT assigned to originNode, create a new one 
            if (originNode == null) {
                double v = (model.getNbrNodes() > 0) ? model.getMeanNodeValue() : DEFAULT_NODE_VALUE;
                originNode = new Point(point.x, point.y, v);
            }
            // select origin node if it exists in the map
            model.setSelectionOfAllFlowsAndNodes(false);
            originNode.setSelected(true);
            
            if (model.getNbrFlows() < 1) {
                flowValue = DEFAULT_FLOW_VALUE;
            } else {
                flowValue = model.getMeanFlowValue();
            }            
        }
        mapComponent.refreshMap();
    }

    private void conditionalCaptureBackground(Point2D.Double point) {
        // if this is the first time, capture the screen.
        if (originNode != null && !isCapturingBackground()) {
            mouse = (Point2D.Double) point.clone();
            captureBackground();
            mapComponent.repaint();
        } else {
            releaseBackground();
        }
    }
    @Override
    public void mouseMoved(Point2D.Double point, MouseEvent evt) {
        conditionalCaptureBackground(point);
    }

    @Override
    public void mouseEntered(Point2D.Double point, MouseEvent evt) {
        conditionalCaptureBackground(point);
    }

    @Override
    public void mouseExited(Point2D.Double point, MouseEvent evt) {
        mouse = null;
        releaseBackground();
        mapComponent.repaint();
    }

    /**
     * Draw a line from the originNode to the current mouse position.
     *
     * @param g2d
     */
    @Override
    public void draw(Graphics2D g2d) {

        if (isCapturingBackground() && originNode != null && mouse != null) {
            FloxRenderer.enableHighQualityRenderingHints(g2d, true);
            Point endNode = new Point(mouse.getX(), mouse.getY(), originNode.getValue());
            Flow flow = new Flow(originNode, endNode, flowValue);
            FloxMapComponent map = (FloxMapComponent) mapComponent;
            GeneralPath flowPath = flow.toGeneralPath(map.getScale(), map.getWest(), map.getNorth());
            double s = map.getScale() / model.getReferenceMapScale();
            double flowStrokeWidth = model.getFlowWidthPx(flow) * s;
            g2d.setStroke(new BasicStroke((float) flowStrokeWidth,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            g2d.draw(flowPath);
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
        if (originNode != null) {

            // treat backspace and delete key release events
            boolean keyReleased = keyEvent.getID() == KeyEvent.KEY_RELEASED;
            boolean isDeleteKey = keyEvent.getKeyCode() == KeyEvent.VK_DELETE;
            boolean isBackspaceKey = keyEvent.getKeyCode() == KeyEvent.VK_BACK_SPACE;
            if (keyReleased && (isDeleteKey || isBackspaceKey)) {
                // delete new origin node
                originNode = null;
                // repaint the map
                mapComponent.refreshMap();
                return true;
            }
        }

        // default: delegate key event to other components
        return false;
    }
}
