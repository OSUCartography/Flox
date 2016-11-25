package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxMapComponent;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.event.MouseEvent;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Iterator;

/**
 *
 * @author danielstephen
 */
public class MoveTool extends DoubleBufferedTool implements CombinableTool {


    boolean dragging = false;

    // Stores the coordinates of the previous drag events.
    double previousDrag_x, previousDrag_y;

    // Constructor
    public MoveTool(AbstractSimpleFeatureMapComponent mapComponent) {
        super(mapComponent);
    }
    
    /**
     * The mouse was clicked, while this MapTool was active.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void mouseClicked(Point2D.Double point, MouseEvent evt) {
        Model model = ((FloxMapComponent) mapComponent).getModel();
        
        // If no dragging occured, deselect all nodes. If a node was clicked, 
        // select that one. If shift is held down, don't do anything.
        if (model.isNodeSelected() && !dragging && !(evt.isShiftDown())) {
            // There are selected nodes, nothing was dragged,
            // and shift isn't held down.
            // deselect all nodes
            ArrayList<Point> selectedNodes = model.getSelectedNodes();
            for (Point pt : selectedNodes) {
                pt.setSelected(false);
            }

            // Get an arraylist selectedNodes that were clicked
            int tolPx = SelectionTool.CLICK_PIXEL_TOLERANCE;
            FloxMapComponent map = (FloxMapComponent) mapComponent;
            Point clickedNode = map.getClickedNode(selectedNodes, point, tolPx);

            // If any nodes were clicked, select the first one
            if (clickedNode != null) {
                clickedNode.setSelected(true);
            }
        }
    }

    /**
     * Called at the start of a mouse drag event.
     *
     * @param point The location of the drag event.
     * @param evt The drag event.
     */
    @Override
    public void startDrag(Point2D.Double point, MouseEvent evt) {
        Model model = ((FloxMapComponent) mapComponent).getModel();
        if (model.isNodeSelected()) {
            // There is at least one selected node
            // Was one of them clicked?
            ArrayList<Point> selectedNodes = model.getSelectedNodes();
            int tolPx = SelectionTool.CLICK_PIXEL_TOLERANCE;
            FloxMapComponent map = (FloxMapComponent) mapComponent;
            Point clickedNode = map.getClickedNode(selectedNodes, point, tolPx);

            if (clickedNode != null) {
                // At least one selected node was clicked
                // Allow dragging
                dragging = true;
            }
        }

        if (model.isControlPtSelected()) {
            // A control point is currently selected.
            // Allow dragging
            dragging = true;
        }

        // Initializes the previousDrag coordinates to the location of this 
        // first drag event.
        previousDrag_x = point.x;
        previousDrag_y = point.y;
    }

    /**
     * Updates the location of the drag to the coordinates of the most recent
     * drag event.
     *
     * @param point
     * @param evt
     */
    @Override
    public void updateDrag(Point2D.Double point, MouseEvent evt) {
        updateLocation(point);
        ((FloxMapComponent)mapComponent).getMainWindow().updateCoordinateFields();
    }

    @Override
    public void endDrag(Point2D.Double point, MouseEvent evt) {
        // this calls mouseClicked
        super.endDrag(point, evt);
        
        Model model = ((FloxMapComponent) mapComponent).getModel();
        // deselect all Bezier control points
        if (model.isControlPtSelected()) {
            Iterator<Flow> iterator = model.flowIterator();
            while (iterator.hasNext()) {
                Flow flow = iterator.next();
                if (flow.isControlPointSelected()) {
                    flow.setControlPointSelected(false);
                }
            }
        }

        updateClippingAreas();
        mapComponent.refreshMap();
        
        // update the force-based layout and add undo option        
        if (dragging == true) {
            ((FloxMapComponent) mapComponent).layout("Move");
        }
        dragging = false;
    }

    /**
     * Updates the location of selected features to the current location of the
     * drag event.
     *
     */
    public void updateLocation(Point2D.Double point) {
        Model model = ((FloxMapComponent) mapComponent).getModel();
        if (model.isControlPtSelected()) {
            // If a control point is selected, move only the control point.
            Iterator<Flow> iterator = model.flowIterator();
            while (iterator.hasNext()) {
                Flow flow = iterator.next();
                if (flow.isSelected()) {
                    if (flow.isControlPointSelected()) {
                        double dx = (point.x - previousDrag_x);
                        double dy = (point.y - previousDrag_y);
                        flow.offsetCtrlPt(dx, dy);
                    }
                }
            }

        } else {
            // Move selected nodes
            Iterator<Point> nodes = model.nodeIterator();
            while (nodes.hasNext()) {
                Point node = nodes.next();
                if (node.isSelected()) {
                    node.x += (point.x - previousDrag_x);
                    node.y += (point.y - previousDrag_y);
                }
            }

            // move control points
            Iterator<Flow> flows = model.flowIterator();
            while (flows.hasNext()) {
                Flow flow = flows.next();
                // if start point is selected, move by half distance
                if (flow.getStartPt().isSelected()) {
                    double dx = (point.x - previousDrag_x) / 2;
                    double dy = (point.y - previousDrag_y) / 2;
                    flow.offsetCtrlPt(dx, dy);
                }
                // if end point is selected, move by half distance
                if (flow.getEndPt().isSelected()) {
                    double dx = (point.x - previousDrag_x) / 2;
                    double dy = (point.y - previousDrag_y) / 2;
                    flow.offsetCtrlPt(dx, dy);
                }
            }

            //calling updateClippingAreas() is possible, but a bit slow, so 
            // better update clip areas when the drag operation ends
        }

        // Redraw the map
        mapComponent.refreshMap();

        // Update the previousDrag coordinates.
        previousDrag_x = point.x;
        previousDrag_y = point.y;
    }

    private void updateClippingAreas() {
        Model model = ((FloxMapComponent) mapComponent).getModel();
        if (model.hasClipAreas() == false) {
            return;
        }
        
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            // only update areas for selected nodes, because only those are moved
            if (flow.getStartPt().isSelected()) {
                model.findStartClipAreaForFlow(flow);
            }
            if (flow.getEndPt().isSelected()) {
                model.findEndClipAreaForFlow(flow);
            }
        }
    }

    /**
     * Returns whether the tool is currently dragging.
     *
     * @return
     */
    @Override
    public boolean isDragging() {
        return dragging;
    }

    @Override
    public boolean adjustCursor(Point2D.Double point) {
        return true;
    }

}
