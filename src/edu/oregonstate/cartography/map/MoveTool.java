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
import java.util.Iterator;

/**
 *
 * @author danielstephen
 */
public class MoveTool extends DoubleBufferedTool implements CombinableTool {

    private final Model model;
    boolean dragging = false;


    // Stores the coordinates of the previous drag events.
    double previousDrag_x, previousDrag_y;

    /**
     * Called at the start of a mouse drag event. 
     * @param point The location of the drag event.
     * @param evt The drag event.
     */
    @Override
    public void startDrag(Point2D.Double point, MouseEvent evt) {

        if (model.isNodeSelected()) {
            dragging = true;
        }

        if (model.isControlPtIsSelected()) {
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
     * @param point
     * @param evt 
     */
    @Override
    public void updateDrag(Point2D.Double point, MouseEvent evt) {

        updateLocation(point);
    }

    @Override
    public void endDrag(Point2D.Double point, MouseEvent evt) {
        super.endDrag(point, evt);

        dragging = false;

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
                mapComponent.eraseBufferImage();
                mapComponent.repaint();
            }
            model.setControlPtIsSelected(false);
        }
    }

    /**
     * Captures the world coordinates of the location where the mouse was first
     * pressed down
     *
     * @param point
     * @param evt
     */
    @Override
    public void mouseDown(Point2D.Double point, MouseEvent evt) {

    }

    /**
     * Updates the location of selected features to the current location of
     * the drag event.
     * @param e
     */
    public void updateLocation(Point2D.Double point) {

        if (model.isControlPtIsSelected()) {
            // If a control point is selected, move only the control point.
            Iterator flows = model.flowIterator();
            while (flows.hasNext()) {
                QuadraticBezierFlow flow = ((QuadraticBezierFlow) flows.next());
                if (flow.isSelected() || 
                        ((FloxMapComponent) mapComponent).isDrawControlPoints()) {
                    Point cPt = flow.getCtrlPt();
                    if (cPt.isSelected()) {
                        cPt.x += (point.x - previousDrag_x);
                        cPt.y += (point.y - previousDrag_y);
                    }
                }

            }
            
        } else {
            // Move selected nodes
            Iterator nodes = model.nodeIterator();
            while (nodes.hasNext()) {
                Point node = (Point) nodes.next();
                if (node.isSelected()) {
                    node.x += (point.x - previousDrag_x);
                    node.y += (point.y - previousDrag_y);
                }
            }
        }

        // Update the longest flow field in the model.
        model.setLongestFlowLength();
        
        mapComponent.eraseBufferImage();
        mapComponent.repaint();

        // Update the previousDrax coordinates.
        previousDrag_x = point.x;
        previousDrag_y = point.y;
    }

    // Constructor
    public MoveTool(AbstractSimpleFeatureMapComponent mapComponent) {
        super(mapComponent);
        this.model = ((FloxMapComponent) mapComponent).getModel();
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
