/*
 * SelectionTool.java
 *
 * Created on April 7, 2005, 7:21 PM
 */
package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxMapComponent;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.event.MouseEvent;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;
import javax.swing.JButton;
import javax.swing.JFormattedTextField;

/**
 * SelectionTool - a tool to select map features by mouse clicks and mouse
 * drags.
 */
public class SelectionTool extends RectangleTool implements CombinableTool {

    /**
     * Tolerance for selection of objects by mouse clicks.
     */
    protected static final int CLICK_PIXEL_TOLERANCE = 2;

    /**
     * Model with elements to select.
     */
    private final Model model;

    protected final JFormattedTextField valueField;
    protected final JFormattedTextField xField;
    protected final JFormattedTextField yField;
    protected final JButton lockUnlockButton;

    /**
     * Create a new instance.
     *
     * @param mapComponent The MapComponent for which this MapTool provides its
     * services.
     * @param valueField Text field to display value of current node or flow.
     */
    public SelectionTool(AbstractSimpleFeatureMapComponent mapComponent,
            JFormattedTextField valueField, JFormattedTextField xField,
            JFormattedTextField yField, JButton lockUnlockButton) {
        super(mapComponent);
        this.valueField = valueField;
        this.xField = xField;
        this.yField = yField;
        this.lockUnlockButton = lockUnlockButton;
        this.model = ((FloxMapComponent) mapComponent).getModel();
    }

    /**
     * Update the valueField's value. If the value of all selected features is
     * the same, set the valueField to that value. Otherwise, set it to null.
     */
    private void updateValueField() {
        ArrayList<Flow> flows = model.getSelectedFlows();
        ArrayList<Point> nodes = model.getSelectedNodes();

        // get the number of selected features
        int nbrFlowsAndNodes = flows.size() + nodes.size();

        // enable the valueField if anything is selected
        valueField.setEnabled(nbrFlowsAndNodes > 0);

        if (nbrFlowsAndNodes == 0) { // If nothing is selected
            valueField.setValue(null);
        } else if (nbrFlowsAndNodes == 1) { // If just one feature is selected
            double value;
            if (flows.size() == 1) { // If the selected feature is a flow
                value = flows.get(0).getValue();
            } else { // The selected feature is a node
                value = nodes.get(0).getValue();
            }
            valueField.setValue(value);
        } else if (flows.size() + nodes.size() > 1) { // More than one thing is selected
            // Check to see if all values are the same.

            // Make an ArrayList of all values.
            ArrayList<Double> values = new ArrayList();
            for (Flow flow : flows) {
                values.add(flow.getValue());
            }
            for (Point node : nodes) {
                values.add(node.getValue());
            }

            // Get the first value in Values
            double v = values.get(0);

            // Compare v to all the other values.
            // If any are different, set valueField to null and exit the method
            for (double value : values) {
                if (v != value) {
                    valueField.setValue(null);
                    return;
                }
            }

            // All values are the same, set valueField to v
            valueField.setValue(v);

        }
    }

    private void updateCoordinateFields() {
        ArrayList<Point> selectedNodes = model.getSelectedNodes();
        int nbrNodes = selectedNodes.size();
        xField.setEnabled(nbrNodes == 1);
        yField.setEnabled(nbrNodes == 1);

        if (nbrNodes != 1) {
            xField.setValue(null);
            yField.setValue(null);
        } else {
            xField.setValue(selectedNodes.get(0).x);
            yField.setValue(selectedNodes.get(0).y);
        }

    }

    /**
     * Sets the icon of the lockUnlockButton to the appropriate icon for the
     * locked status of selected flows.
     */
    private void updateLockUnlockButton() {
        ArrayList<Flow> selectedFlows = model.getSelectedFlows();
        if (selectedFlows.size() > 0) {
            lockUnlockButton.setEnabled(true);

            int locked = 0;
            int unlocked = 0;
            for (Flow flow : selectedFlows) {
                if (flow.isLocked()) {
                    locked++;
                } else {
                    unlocked++;
                }
            }
            if (locked + unlocked == 0) {
                lockUnlockButton.setEnabled(false);
            } else if (locked > 0 && unlocked == 0) {
                lockUnlockButton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/edu/oregonstate/cartography/icons/Locked16x16.gif")));
            } else if (unlocked > 0 && locked == 0) {
                lockUnlockButton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/edu/oregonstate/cartography/icons/Unlocked16x16.gif")));
            } else {
                lockUnlockButton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/edu/oregonstate/cartography/icons/LockedUnlocked16x16.gif")));
            }
        } else {
            lockUnlockButton.setEnabled(false);
        }

    }

    /**
     * A drag ends, while this MapTool was the active one.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void endDrag(Point2D.Double point, MouseEvent evt) {

        Rectangle2D.Double rect = getRectangle();
        super.endDrag(point, evt);

        if (rect != null) {
            /*boolean selectionChanged = */
            selectByRectangle(rect, evt.isShiftDown());
            updateValueField();
            updateCoordinateFields();
            updateLockUnlockButton();
        }

        if (model.isControlPtSelected()) {
            // deselect all control points
            Iterator<Flow> iterator = model.flowIterator();
            while (iterator.hasNext()) {
                Flow flow = iterator.next();
                Point cPt = flow.getCtrlPt();
                cPt.setSelected(false);
            }
            mapComponent.refreshMap();
        }

        setDefaultCursor();
    }

    /**
     * The mouse was clicked, while this MapTool was the active one.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void mouseClicked(Point2D.Double point, MouseEvent evt) {
        super.mouseClicked(point, evt);

        if (model.isControlPtSelected()) {
            // deselect all control points
            Iterator<Flow> iterator = model.flowIterator();
            while (iterator.hasNext()) {
                Flow flow = iterator.next();
                Point cPt = flow.getCtrlPt();
                cPt.setSelected(false);
            }
        }
        mapComponent.refreshMap();
    }

    /**
     * The left mouse button was clicked down, while this MapTool was the active
     * one.
     *
     * @param point The location of the mouse in world coordinates
     * @param evt The original event.
     */
    @Override
    public void mouseDown(Point2D.Double point, MouseEvent evt) {

        selectByPoint(point, evt.isShiftDown(), SelectionTool.CLICK_PIXEL_TOLERANCE);

        updateValueField();
        updateCoordinateFields();
        updateLockUnlockButton();
    }

    public boolean selectByRectangle(Rectangle2D.Double rect, boolean shiftDown) {

        // Select nodes
        boolean nodeGotSelected = false;
        if (((FloxMapComponent) mapComponent).isDrawNodes()) {
            Iterator<Point> nodes = model.nodeIterator();
            while (nodes.hasNext()) {
                Point pt = nodes.next();

                if (((mapComponent.xToPx(pt.x) >= mapComponent.xToPx(rect.getMinX()) - 10)
                        && (mapComponent.xToPx(pt.x) <= mapComponent.xToPx(rect.getMaxX()) + 10))
                        && ((mapComponent.yToPx(pt.y) >= mapComponent.yToPx(rect.getMaxY()) - 10)
                        && (mapComponent.yToPx(pt.y) <= mapComponent.yToPx(rect.getMinY()) + 10))) {
                    pt.setSelected(true);
                    nodeGotSelected = true;
                } else if (shiftDown == false) {
                    pt.setSelected(false);
                }
            }
        }

        // Select flows
        boolean flowGotSelected = false;
        if (((FloxMapComponent) mapComponent).isDrawFlows()) {
            double deCasteljauTol = model.getDeCasteljauTolerance();

            Iterator<Flow> flows = model.flowIterator();
            if (rect.height == 0 || rect.width == 0) {
                rect.add(rect.getMaxX() + (1 / mapComponent.getScale()),
                        rect.getMaxY() + (1 / mapComponent.getScale()));
            }
            while (flows.hasNext()) {
                Flow flow = model.clipFlow(flows.next(), false);
                if (flow.getBoundingBox().intersects(rect)) {
                    ArrayList<Point> pts = flow.toUnclippedStraightLineSegments(deCasteljauTol);
                    for (int i = 0; i < pts.size() - 1; i++) {
                        // Get the points
                        Point pt1 = pts.get(i);
                        Point pt2 = pts.get(i + 1);
                        // Does the segment intersect rect?
                        if (rect.intersectsLine(pt1.x, pt1.y, pt2.x, pt2.y)) {
                            flow.setSelected(true);
                            flowGotSelected = true;
                            break;
                        }
                        // Else statement for deselecting flows? Doesn't seem to be
                        // needed because flows are deselected by the initial click.
                    }
                } else // flow bb does not intersect rect
                {
                    if (shiftDown == false) {
                        flow.setSelected(false);
                    }
                }
            }
        }

        mapComponent.refreshMap();
        return (flowGotSelected || nodeGotSelected);
    }

    /**
     * Select a map feature after the user clicked on the map.
     *
     * @param point click location in world coordinates.
     * @param shiftDown true if shift key was pressed when the click happened
     * @param pixelTolerance selection tolerance in pixels
     * @return
     */
    private boolean selectByPoint(Point2D.Double point, boolean shiftDown, int pixelTolerance) {

        boolean nodeGotSelected = false;
        boolean flowGotSelected = false;
        boolean controlPtGotSelected = false;

        // Select Control Point
        if (((FloxMapComponent) mapComponent).isDrawFlows()) {
            if (model.isFlowSelected()) {
                // Iterate througth the flows, checking to see if it is selected.
                Iterator<Flow> iterator = model.flowIterator();
                while (iterator.hasNext()) {
                    Flow flow = iterator.next();
                    if (flow.isSelected()) {
                        // See if the event point is near the control point.
                        Point cPt = flow.getCtrlPt();

                        // If a control point already got selected, exit the loop
                        if (controlPtGotSelected) {
                            cPt.setSelected(false);
                            continue;
                        }

                        if (((mapComponent.xToPx(cPt.x) >= mapComponent.xToPx(point.x) - 5)
                                && (mapComponent.xToPx(cPt.x) <= mapComponent.xToPx(point.x) + 5))
                                && ((mapComponent.yToPx(cPt.y) >= mapComponent.yToPx(point.y) - 5)
                                && (mapComponent.yToPx(cPt.y) <= mapComponent.yToPx(point.y) + 5))) {
                            cPt.setSelected(true);
                            // Lock the flow
                            flow.setLocked(true);
                            lockUnlockButton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/edu/oregonstate/cartography/icons/Locked16x16.gif")));
                            controlPtGotSelected = true;
                            mapComponent.refreshMap();
                            // A control point was selected, so exit the method to 
                            // avoid deselecting flows
                            return true;
                        } else {
                            cPt.setSelected(false);
                        }
                    }
                }
            }
        }

        // SELECT NODES
        // Get clicked nodes
        ArrayList<Point> nodes = model.getNodes();
        int tolPx = SelectionTool.CLICK_PIXEL_TOLERANCE;
        FloxMapComponent map = (FloxMapComponent) mapComponent;
        Point clickedNode = map.getClickedNode(nodes, point, tolPx);

        // If a node was clicked, select it. If it was already selected,
        // deselect it if shift is held down. Deselect all other nodes unless
        // shift is held down.
        if (clickedNode != null) { // At least one node was clicked.
            if (clickedNode.isSelected()) {// The first node is currently selected.
                // Set nodeGotSelected to true so that flows aren't selected later.
                nodeGotSelected = true;
                if (shiftDown) {
                    // Shift is held down.
                    // Deselect it
                    clickedNode.setSelected(false);
                }
                // set nodeGotSelected to true so that flows aren't 
                // selected later. FIXME, this is weird.
                nodeGotSelected = true;
            } else { // The first node in clickedNodes is not currently selected.
                if (!shiftDown) {
                    // shift is not held down.
                    // deselect all nodes
                    for (Point node : nodes) {
                        node.setSelected(false);
                    }
                }
                // select the first node in clickedNodes
                clickedNode.setSelected(true);
                nodeGotSelected = true;
            }
        } else // No nodes were clicked.
        {
            if (!shiftDown) {
                // Shift is not held down.
                // Deselect all nodes.
                for (Point node : nodes) {
                    node.setSelected(false);
                }
            }
        }

        // SELECT FLOWS
        if (((FloxMapComponent) mapComponent).isDrawFlows()) {

            Iterator<Flow> flows = model.flowIterator();

            // The distance tolerance the click needs to be within the flow in order
            // to be selected, scaled to the current map scale.
            double toleranceWorld = pixelTolerance / mapComponent.getScale();

            double[] xy = new double[2];
            double tol = 1d / model.getReferenceMapScale(); // 1 pixel in world coordinates

            while (flows.hasNext()) {
                Flow flow = flows.next();

                // flow width
                double flowWidthWorld = model.getFlowWidthPx(flow) / model.getReferenceMapScale();

                // Add half the width to tol, scaled to the map scale
                double maxDistWorld = toleranceWorld + flowWidthWorld / 2;

                // Add a little padding to the bounding box in the amount of tol
                Rectangle2D flowBB = flow.getBoundingBox();
                flowBB.add(flowBB.getMinX() - maxDistWorld, flowBB.getMinY() - maxDistWorld);
                flowBB.add(flowBB.getMaxX() + maxDistWorld, flowBB.getMaxY() + maxDistWorld);

                if (flowBB.contains(point)) {
                    // Get the distance of the click to the flow.
                    xy[0] = point.x;
                    xy[1] = point.y;
                    double distanceSqWorld = flow.distanceSq(xy, tol);
                    // If that distance is less than the tolerance, select it.
                    if (distanceSqWorld <= maxDistWorld * maxDistWorld && !nodeGotSelected) {
                        if (shiftDown) {
                            flow.setSelected(!flow.isSelected());
                        } else {
                            flow.setSelected(true);
                        }
                        flowGotSelected = true;
                    } else if (shiftDown == false) {
                        flow.setSelected(false);
                    }

                } else if (shiftDown == false) {
                    flow.setSelected(false);
                }
            }
        }

        mapComponent.refreshMap();
        return (flowGotSelected || nodeGotSelected);
    }

    @Override
    protected String getCursorName() {
        return "selectionarrow";
    }

    @Override
    public boolean adjustCursor(Point2D.Double point) {
        this.setDefaultCursor();
        return true;
    }
}
