/*
 * SelectionTool.java
 *
 * Created on April 7, 2005, 7:21 PM
 */
package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.model.CubicBezierFlow;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import edu.oregonstate.cartography.utils.GeometryUtils;
import java.awt.event.MouseEvent;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * SelectionTool - a tool to select GeoObjects by mouse clicks and mouse drags.
 */
public class SelectionTool extends RectangleTool implements CombinableTool {

    /**
     * Tolerance for selection of objects by mouse clicks.
     */
    protected static final int CLICK_PIXEL_TOLERANCE = 12;

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
    public SelectionTool(AbstractSimpleFeatureMapComponent mapComponent, Model model) {
        super(mapComponent);
        this.model = model;
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
            final boolean selectionChanged = selectByRectangle(rect, evt.isShiftDown());
        }

        if(model.isControlPtIsSelected()) {
            // deselect all control points
            Iterator flows = model.flowIterator();
            while(flows.hasNext()) {
                Flow flow = (Flow) flows.next();
                if(flow instanceof CubicBezierFlow) {
                    break;
                }
                
                Point cPt = ((QuadraticBezierFlow) flow).getCtrlPt();
                cPt.setSelected(false);
            }
            mapComponent.eraseBufferImage();
            mapComponent.repaint();
            model.setControlPtIsSelected(false);
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

        if(model.isControlPtIsSelected()) {
            // deselect all control points
            Iterator flows = model.flowIterator();
            while(flows.hasNext()) {
                Flow flow = (Flow) flows.next();
                if(flow instanceof CubicBezierFlow) {
                    break;
                }
                
                Point cPt = ((QuadraticBezierFlow) flow).getCtrlPt();
                cPt.setSelected(false);
                
            }
            model.setControlPtIsSelected(false);
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

        boolean selectionChanged = selectByPoint(point, evt.isShiftDown(),
                SelectionTool.CLICK_PIXEL_TOLERANCE);
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
        double deCasteljauTol = model.getShortestFlowLengthDividedByMinFlowNodes();
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
        if (flowGotSelected || nodeGotSelected) {
            model.setNodeIsSelected(nodeGotSelected);
            model.setFlowIsSelected(flowGotSelected);
            return true;
        } else {
            if (shiftDown == false) {
                model.setNodeIsSelected(false);
                model.setFlowIsSelected(false);
            }
            return false;
        }
    }

    public boolean selectByPoint(Point2D.Double point, boolean shiftDown, int pixelTolerance) {
        Iterator<Point> nodes = model.nodeIterator();

        boolean nodeGotSelected = false;
        boolean flowGotSelected = false;
        boolean controlPtGotSelected = false;

        // if the model says a flow is currently selected...
        if (model.isFlowIsSelected()) {
            // Iterate througth the flows, checking to see if it is selected.
            Iterator flows = model.flowIterator();
            while (flows.hasNext()) {
                Flow flow = (Flow) flows.next();
                if (flow instanceof CubicBezierFlow) {
                    break;
                }
                if (flow.isSelected()) {
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
                        model.setControlPtIsSelected(true);
                        mapComponent.eraseBufferImage();
                        mapComponent.repaint();
                        return true;
                    } else {
                        cPt.setSelected(false);
                        model.setControlPtIsSelected(false);
                    }
                }
            }
        }

        // Select nodes
        while (nodes.hasNext()) {
            Point pt = nodes.next();

            if (nodeGotSelected && (shiftDown == false)) {
                pt.setSelected(false);
                continue;
            }

            if (((mapComponent.xToPx(pt.x) >= mapComponent.xToPx(point.x) - pixelTolerance)
                    && (mapComponent.xToPx(pt.x) <= mapComponent.xToPx(point.x) + pixelTolerance))
                    && ((mapComponent.yToPx(pt.y) >= mapComponent.yToPx(point.y) - pixelTolerance)
                    && (mapComponent.yToPx(pt.y) <= mapComponent.yToPx(point.y) + pixelTolerance))) {
                pt.setSelected(true);
                nodeGotSelected = true;
            } else {
                if (shiftDown == false) {
                    pt.setSelected(false);
                }
            }

        }

        // Select flows
        Iterator<Flow> flows = model.flowIterator();

        double deCasteljauTol = model.getShortestFlowLengthDividedByMinFlowNodes();
        while (flows.hasNext()) {
            Flow flow = flows.next();
            if (flow.getBoundingBox().contains(point)) {

                // iterate through the points along the flow
                ArrayList<Point> pts = flow.toStraightLineSegments(deCasteljauTol);
                for (int i = 0; i < (pts.size() - 1); i++) {

                    Point pt1 = pts.get(i);
                    Point pt2 = pts.get(i + 1);

                    ArrayList<Point> segmentPts = new ArrayList();
                    segmentPts.add(pt1);
                    segmentPts.add(pt2);

                    // Does the bounding box of the segment contain the click?
                    if (GeometryUtils.getBoundingBoxOfPoints(segmentPts).contains(point)) {
                        // Convert the point coordinates to pixel coordinates
                        double x0px = mapComponent.xToPx(point.x);
                        double y0px = mapComponent.yToPx(point.y);
                        double x1px = mapComponent.xToPx(pt1.x);
                        double y1px = mapComponent.yToPx(pt1.y);
                        double x2px = mapComponent.xToPx(pt2.x);
                        double y2px = mapComponent.yToPx(pt2.y);

                        double dist = GeometryUtils.getDistanceToLine(x0px, y0px, x1px, y1px,
                                x2px, y2px);

                        // select the flow if dist is below a tolorance
                        // don't select a flow if a node got selected
                        if (dist <= 4 && nodeGotSelected != true) {
                            flow.setSelected(true);
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

                    if (flowGotSelected) {
                        break;
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
        if (flowGotSelected || nodeGotSelected) {
            model.setNodeIsSelected(nodeGotSelected);
            model.setFlowIsSelected(flowGotSelected);
            return true;
        } else {
            if (shiftDown == false) {
                model.setNodeIsSelected(false);
                model.setFlowIsSelected(false);
            }
            return false;
        }
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
