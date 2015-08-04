/*
 * SelectionTool.java
 *
 * Created on April 7, 2005, 7:21 PM
 */
package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxMapComponent;
import edu.oregonstate.cartography.flox.model.CubicBezierFlow;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.event.MouseEvent;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;
import javax.swing.JFormattedTextField;

/**
 * SelectionTool - a tool to select map features by mouse clicks and mouse
 * drags.
 */
public class SelectionTool extends RectangleTool implements CombinableTool {

    /**
     * Tolerance for selection of objects by mouse clicks.
     */
    protected static final int CLICK_PIXEL_TOLERANCE = 3;

    /**
     * Model with elements to select.
     */
    private final Model model;

    protected final JFormattedTextField valueField;
    protected final JFormattedTextField xField;
    protected final JFormattedTextField yField;

    /**
     * Create a new instance.
     *
     * @param mapComponent The MapComponent for which this MapTool provides its
     * services.
     * @param valueField Text field to display value of current node or flow.
     */
    public SelectionTool(AbstractSimpleFeatureMapComponent mapComponent,
            JFormattedTextField valueField, JFormattedTextField xField, 
            JFormattedTextField yField) {
        super(mapComponent);
        this.valueField = valueField;
        this.xField = xField;
        this.yField = yField;
        this.model = ((FloxMapComponent) mapComponent).getModel();
    }

    /**
     * Update the text field for changing the value of flows and nodes.
     */
    private void updateValueField() {
        ArrayList<Flow> flows = model.getSelectedFlows();
        ArrayList<Point> nodes = model.getSelectedNodes();
        int nbrFlowsAndNodes = flows.size() + nodes.size();
        valueField.setEnabled(nbrFlowsAndNodes > 0);
        
        if (nbrFlowsAndNodes == 0) {
            valueField.setValue(null);
        } else if (nbrFlowsAndNodes == 1) {
            double value;
            if (flows.size() == 1) {
                value = flows.get(0).getValue();
            } else {
                value = nodes.get(0).getValue();
            }
            valueField.setValue(value);
        } else if (flows.size() + nodes.size() > 1) {
            // test whether all selected nodes and/or flows have the same value
            double v = Double.NaN;
            if (flows.size() > 0) {
                v = flows.get(0).getValue();
            }
            if (nodes.size() > 0) {
                v = nodes.get(0).getValue();
            }
            for (int i = 1; i < flows.size(); i++) {
                if (flows.get(i).getValue() != v) {
                    v = Double.NaN;
                    break;
                }
            }
            for (int i = 1; i < nodes.size(); i++) {
                if (nodes.get(i).getValue() != v) {
                    v = Double.NaN;
                    break;
                }
            }
            valueField.setValue(Double.isNaN(v) ? null : v);
        }
    }

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
        }

        if (model.isControlPtSelected()) {
            // deselect all control points
            Iterator flows = model.flowIterator();
            while (flows.hasNext()) {
                Flow flow = (Flow) flows.next();
                if (flow instanceof CubicBezierFlow) {
                    break;
                }

                Point cPt = ((QuadraticBezierFlow) flow).getCtrlPt();
                cPt.setSelected(false);
            }
            mapComponent.eraseBufferImage();
            mapComponent.repaint();
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
            Iterator flows = model.flowIterator();
            while (flows.hasNext()) {
                Flow flow = (Flow) flows.next();
                if (flow instanceof CubicBezierFlow) {
                    break;
                }
                Point cPt = ((QuadraticBezierFlow) flow).getCtrlPt();
                cPt.setSelected(false);
            }
        }
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

        /*boolean selectionChanged = */ selectByPoint(point, evt.isShiftDown(),
                SelectionTool.CLICK_PIXEL_TOLERANCE);

        updateValueField();
        updateCoordinateFields();
    }

    public boolean selectByRectangle(Rectangle2D.Double rect, boolean shiftDown) {

        // Select nodes
        boolean nodeGotSelected = false;
        Iterator<Point> nodes = model.nodeIterator();
        while (nodes.hasNext()) {
            Point pt = nodes.next();

            if (((mapComponent.xToPx(pt.x) >= mapComponent.xToPx(rect.getMinX()) - 10)
                    && (mapComponent.xToPx(pt.x) <= mapComponent.xToPx(rect.getMaxX()) + 10))
                    && ((mapComponent.yToPx(pt.y) >= mapComponent.yToPx(rect.getMaxY()) - 10)
                    && (mapComponent.yToPx(pt.y) <= mapComponent.yToPx(rect.getMinY()) + 10))) {
                pt.setSelected(true);
                nodeGotSelected = true;
            } else {
                if (shiftDown == false) {
                    pt.setSelected(false);
                }

            }

        }

        // Select flows
        double deCasteljauTol = model.getDeCasteljauTolerance();
        boolean flowGotSelected = false;
        Iterator<Flow> flows = model.flowIterator();
        while (flows.hasNext()) {
            Flow flow = flows.next();
            if (flow.getBoundingBox().intersects(rect)) {
                ArrayList<Point> pts = flow.toStraightLineSegments(deCasteljauTol);
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
            } else {
                // flow bb does not intersect rect
                if (shiftDown == false) {
                    flow.setSelected(false);
                }
            }
        }

        mapComponent.eraseBufferImage();
        mapComponent.repaint();
        return (flowGotSelected || nodeGotSelected);
    }

    private boolean selectByPoint(Point2D.Double point, boolean shiftDown, int pixelTolerance) {

        double scale = mapComponent.getScale();
        boolean nodeGotSelected = false;
        boolean flowGotSelected = false;
        boolean controlPtGotSelected = false;

        // Get the locked scale factor needed to calculate feature sizes
        double lockedScaleFactor;
        if (!model.isFlowWidthLocked()) {
            lockedScaleFactor = 1;
        } else {
            // compare the locked scale to the current scale
            double lockedMapScale = model.getLockedMapScale();
            lockedScaleFactor = scale / lockedMapScale;
        }

        // if the model says a flow is currently selected, check to see if a 
        // control point is nearby, and select it if so.
        if (model.isFlowSelected()
                || ((FloxMapComponent) mapComponent).isDrawControlPoints()) {
            // Iterate througth the flows, checking to see if it is selected.
            Iterator flows = model.flowIterator();
            while (flows.hasNext()) {
                Flow flow = (Flow) flows.next();
                if (flow instanceof CubicBezierFlow) {
                    break;
                }
                if (flow.isSelected()
                        || ((FloxMapComponent) mapComponent).isDrawControlPoints()) {
                    // See if the event point is near the control point.
                    Point cPt = ((QuadraticBezierFlow) flow).getCtrlPt();

                    if (controlPtGotSelected) {
                        cPt.setSelected(false);
                        continue;
                    }

                    if (((mapComponent.xToPx(cPt.x) >= mapComponent.xToPx(point.x) - 5)
                            && (mapComponent.xToPx(cPt.x) <= mapComponent.xToPx(point.x) + 5))
                            && ((mapComponent.yToPx(cPt.y) >= mapComponent.yToPx(point.y) - 5)
                            && (mapComponent.yToPx(cPt.y) <= mapComponent.yToPx(point.y) + 5))) {
                        cPt.setSelected(true);
                        controlPtGotSelected = true;
                        mapComponent.eraseBufferImage();
                        mapComponent.repaint();
                        return true;
                    } else {
                        cPt.setSelected(false);
                    }
                }
            }
        }

        // Iterate backwards through the nodes so that nodes drawn last (on top)
        // get selected first.
        ArrayList<Point> nodes = model.getNodes();
        for (int i = nodes.size() - 1; i >= 0; i--) {
            Point node = nodes.get(i);

            // If a node was selected stop checking.
            if (nodeGotSelected && (shiftDown == false)) {
                node.setSelected(false);
                continue;
            }

            // Get the radius of the node
            double nodeArea = Math.abs(node.getValue()
                    * model.getNodeSizeScaleFactor());
            double nodeRadius = (Math.sqrt(nodeArea / Math.PI)) * lockedScaleFactor;
            nodeRadius = (nodeRadius + pixelTolerance) / scale;

            // Calculate the distance of the click from the node center.
            double dx = node.x - point.x;
            double dy = node.y - point.y;
            double distSquared = (dx * dx + dy * dy);

            if (distSquared <= nodeRadius * nodeRadius) {
                node.setSelected(!node.isSelected());
                node.setSelected(true);
                nodeGotSelected = true;
            } else {
                if (!shiftDown) {
                    node.setSelected(false);
                }
            }
        }

        // Select flows
        Iterator<Flow> flows = model.flowIterator();

        // The distance tolerance the click needs to be within the flow in order
        // to be selected, scaled to the current map scale.
        double tol = pixelTolerance / scale;

        double[] xy = new double[2];
        while (flows.hasNext()) {
            QuadraticBezierFlow flow = (QuadraticBezierFlow) flows.next();

            // Get the flow's width.
            double width = Math.abs(flow.getValue()) * model.getFlowWidthScaleFactor()
                    * lockedScaleFactor;

            // Add half the width to tol, scaled to the map scale
            double totalTol = tol + ((width / 2) / scale);

            // Add a little padding to the bounding box in the amount of tol
            Rectangle2D flowBB = flow.getBoundingBox();

            flowBB.add(flowBB.getMinX() - totalTol, flowBB.getMinY() - totalTol);
            flowBB.add(flowBB.getMaxX() + totalTol, flowBB.getMaxY() + totalTol);

            if (flowBB.contains(point)) {

                // Get the distance of the click to the flow.
                xy[0] = point.x;
                xy[1] = point.y;
                double distance = flow.distance(xy);
                // If that distance is less than the tolerance, select it.
                if (distance <= totalTol && !nodeGotSelected) {
                    if (shiftDown) {
                        flow.setSelected(!flow.isSelected());
                    } else {
                        flow.setSelected(true);
                    }
                    flowGotSelected = true;
                } else {
                    if (shiftDown == false) {
                        flow.setSelected(false);
                    }
                }

            } else {
                if (shiftDown == false) {
                    flow.setSelected(false);
                }
            }
        }

        mapComponent.eraseBufferImage();
        mapComponent.repaint();
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
