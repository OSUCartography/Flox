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
import javax.swing.JButton;
import javax.swing.JFormattedTextField;

/**
 *
 * @author danielstephen
 */
public class MoveTool extends DoubleBufferedTool implements CombinableTool {

    private final Model model;

    protected final JFormattedTextField xField;
    protected final JFormattedTextField yField;
    protected final JButton lockUnlockButton;

    boolean dragging = false;

    // Stores the coordinates of the previous drag events.
    double previousDrag_x, previousDrag_y;

    private void updateCoordinateFields() {
        ArrayList<Point> nodes = model.getSelectedNodes();
        int nbrNodes = nodes.size();
        xField.setEnabled(nbrNodes == 1);
        yField.setEnabled(nbrNodes == 1);

        if (nbrNodes != 1) {
            xField.setValue(null);
            yField.setValue(null);
        } else {
            xField.setValue(nodes.get(0).x);
            yField.setValue(nodes.get(0).y);
        }

    }

    /**
     * The mouse was clicked, while this MapTool was active.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void mouseClicked(Point2D.Double point, MouseEvent evt) {

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
            // FIXME, a magic number of 2 is passed in for the pixel tolorance
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
        updateCoordinateFields();
        model.computeArrowheadsAndClipping();
    }

    @Override
    public void endDrag(Point2D.Double point, MouseEvent evt) {
        // this calls mouseClicked
        super.endDrag(point, evt);

        // deselect all Bezier control points
        if (model.isControlPtSelected()) {
            Iterator<Flow> iterator = model.flowIterator();
            while (iterator.hasNext()) {
                Flow flow = iterator.next();
                Point cPt = flow.getCtrlPt();
                if (cPt.isSelected()) {
                    cPt.setSelected(false);
                }
                mapComponent.refreshMap();
            }
        }

        model.computeArrowheadsAndClipping();

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

        if (model.isControlPtSelected()) {
            // If a control point is selected, move only the control point.
            Iterator<Flow> iterator = model.flowIterator();
            while (iterator.hasNext()) {
                Flow flow = iterator.next();
                if (flow.isSelected()) {
                    Point cPt = flow.getCtrlPt();
                    if (cPt.isSelected()) {
                        cPt.x += (point.x - previousDrag_x);
                        cPt.y += (point.y - previousDrag_y);
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
                if (flow.getStartPt().isSelected()) {
                    flow.getCtrlPt().x += (point.x - previousDrag_x) / 2;
                    flow.getCtrlPt().y += (point.y - previousDrag_y) / 2;
                }
                if (flow.getEndPt().isSelected()) {
                    flow.getCtrlPt().x += (point.x - previousDrag_x) / 2;
                    flow.getCtrlPt().y += (point.y - previousDrag_y) / 2;
                }
            }
        }

        // Redraw the map
        mapComponent.refreshMap();

        // Update the previousDrag coordinates.
        previousDrag_x = point.x;
        previousDrag_y = point.y;
    }

    // Constructor
    public MoveTool(AbstractSimpleFeatureMapComponent mapComponent,
            JFormattedTextField xField, JFormattedTextField yField,
            JButton lockUnlockButton) {
        super(mapComponent);
        this.model = ((FloxMapComponent) mapComponent).getModel();
        this.xField = xField;
        this.yField = yField;
        this.lockUnlockButton = lockUnlockButton;
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
