/*
 * SelectionTool.java
 *
 * Created on April 7, 2005, 7:21 PM
 */
package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxMapComponent;
import edu.oregonstate.cartography.flox.gui.FloxRenderer;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.event.MouseEvent;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;

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

    /**
     * Create a new instance.
     *
     * @param mapComponent The MapComponent for which this MapTool provides its
     * services.
     */
    public SelectionTool(AbstractSimpleFeatureMapComponent mapComponent) {
        super(mapComponent);
       
        this.model = ((FloxMapComponent) mapComponent).getModel();
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
            selectByRectangle(rect, evt.isShiftDown());
            ((FloxMapComponent)mapComponent).getMainWindow().updateValueField();
            ((FloxMapComponent)mapComponent).getMainWindow().updateCoordinateFields();
            ((FloxMapComponent)mapComponent).getMainWindow().updateLockUnlockButtonIcon();
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

        // detect click on lock icon and unlock clicked flow
        Iterator<Flow> iterator = model.flowIterator();
        double tolWorldCoord = FloxRenderer.LOCK_ICON_RADIUS / mapComponent.getScale();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isLocked() && flow.getBoundingBox().contains(point)) {
                Point lockCenter = flow.pointOnCurve(0.5);
                double dist = lockCenter.distance(point.x, point.y);
                if (dist < tolWorldCoord) {
                    flow.setLocked(false);
                    mapComponent.refreshMap();
                    ((FloxMapComponent)mapComponent).getMainWindow().updateLockUnlockButtonIcon();
                    // compute new flow layout
                    ((FloxMapComponent) mapComponent).layout("Unlock");
                    return;
                }
            }
        }

        super.mouseClicked(point, evt);

        // deselect all control points
        iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            Point cPt = flow.getCtrlPt();
            cPt.setSelected(false);
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
        ((FloxMapComponent)mapComponent).getMainWindow().updateValueField();
        ((FloxMapComponent)mapComponent).getMainWindow().updateCoordinateFields();
        ((FloxMapComponent)mapComponent).getMainWindow().updateLockUnlockButtonIcon();
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
            double segmentLength = model.segmentLength();
            Iterator<Flow> flows = model.flowIterator();
            if (rect.height == 0 || rect.width == 0) {
                rect.add(rect.getMaxX() + (1 / mapComponent.getScale()),
                        rect.getMaxY() + (1 / mapComponent.getScale()));
            }
            while (flows.hasNext()) {
                Flow flow = flows.next();
                Flow clippedFlow = model.clipFlow(flow, false, false);
                if (clippedFlow.getBoundingBox().intersects(rect)) {
                    ArrayList<Point> pts = clippedFlow.regularIntervals(segmentLength);
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
                            ((FloxMapComponent)mapComponent).getMainWindow().updateLockUnlockButtonIcon();
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
                Flow clipppedFlow = model.clipFlow(flow, false, false);

                // flow width
                double flowWidthWorld = model.getFlowWidthPx(flow.getValue()) / model.getReferenceMapScale();

                // Add half the width to tol, scaled to the map scale
                double maxDistWorld = toleranceWorld + flowWidthWorld / 2;

                // Add a little padding to the bounding box in the amount of tol
                Rectangle2D flowBB = clipppedFlow.getBoundingBox();
                flowBB.add(flowBB.getMinX() - maxDistWorld, flowBB.getMinY() - maxDistWorld);
                flowBB.add(flowBB.getMaxX() + maxDistWorld, flowBB.getMaxY() + maxDistWorld);

                if (flowBB.contains(point)) {
                    // Get the distance of the click to the flow.
                    double distanceSqWorld = clipppedFlow.distanceSq(point.x, point.y, tol);
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
