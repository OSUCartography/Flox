package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxMapComponent;
import edu.oregonstate.cartography.flox.gui.FloxRenderer;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.BasicStroke;
import java.awt.Color;
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
     * The distance from an existing node that a click must be within for that
     * node to be assigned to originNode or destinationNode.
     */
    private final double PIXEL_TOLERANCE = 3;

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

    private Flow makeFlow(double endX, double endY) {
        Point endNode = new Point(endX, endY, originNode.getValue());
        Flow flow = new Flow(originNode, endNode, flowValue);
        if (model.isClipFlowStarts()) {
            model.updateStartClipArea(flow);
        }
        if (model.isClipFlowEnds()) {
            model.updateEndClipArea(flow);
        }
        return flow;
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

            Point endNode = ((FloxMapComponent) mapComponent).getClickedNode(nodes, point, PIXEL_TOLERANCE);

            // create new end node if user did not click on existing node
            if (endNode == null) {
                endNode = new Point(point.x, point.y, originNode.getValue());
            }

            // Add the new flow to the data model.
            if (endNode != originNode) {
                model.addFlow(makeFlow(endNode.x, endNode.y));
            }

            originNode = null;

            releaseBackground();

            // update the force-based layout and add undo option
            ((FloxMapComponent) mapComponent).layout("Add Flow");

        } else {

            ArrayList<Point> nodes = model.getNodes();
            originNode = ((FloxMapComponent) mapComponent).getClickedNode(nodes, point, PIXEL_TOLERANCE);

            // If an existing node was NOT assigned to originNode, create a new one 
            if (originNode == null) {
                double v = (model.getNbrNodes() > 0) ? model.getMeanNodeValue() : Model.DEFAULT_NODE_VALUE;
                originNode = new Point(point.x, point.y, v);
            }
            // select origin node if it exists in the map
            model.setSelectionOfAllFlowsAndNodes(false);
            originNode.setSelected(true);

            if (model.getNbrFlows() < 1) {
                flowValue = Model.DEFAULT_FLOW_VALUE;
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
            FloxMapComponent map = (FloxMapComponent) mapComponent;
            double s = map.getScale() / model.getReferenceMapScale();
            FloxRenderer.enableHighQualityRenderingHints(g2d, true);
            Flow flow = makeFlow(mouse.getX(), mouse.getY());
            double flowStrokeWidth = model.getFlowWidthPx(flow) * s;
            g2d.setStroke(new BasicStroke((float) flowStrokeWidth,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));

            // draw the clipped flow section in gray
            boolean clipWithAreas = model.isClipFlowEnds() || model.isClipFlowStarts();
            if (clipWithAreas) {
                GeneralPath flowPath = flow.toGeneralPath(map.getScale(), map.getWest(), map.getNorth());
                g2d.setColor(Color.GRAY);
                g2d.draw(flowPath);
            }

            // draw clipped flow in black
            flow = model.clipFlow(flow, true);
            GeneralPath flowPath = flow.toGeneralPath(map.getScale(), map.getWest(), map.getNorth());
            g2d.setColor(Color.BLACK);
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
            boolean isEscapeKey = keyEvent.getKeyCode() == KeyEvent.VK_ESCAPE;
            if (keyReleased && (isDeleteKey || isBackspaceKey || isEscapeKey)) {
                // delete new origin node
                originNode = null;
                // repaint the map
                releaseBackground();
                mapComponent.refreshMap();
                return true;
            }
        }

        // default: delegate key event to other components
        return false;
    }

}
