package edu.oregonstate.cartography.map;

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
    private final static boolean VERBOSE = false;
    boolean dragging = false;

    //Holds the coordinates of the user's last mouseDown event
    double last_x, last_y;

    double previousDrag_x, previousDrag_y;

    @Override
    public void startDrag(Point2D.Double point, MouseEvent evt) {

        if (model.isNodeIsSelected()) {
            dragging = true;
        }

        if (model.isControlPtIsSelected()) {
            dragging = true;
        }
        
        previousDrag_x = point.x;
        previousDrag_y = point.y;

    }

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
     *
     * @param e
     */
    public void updateLocation(Point2D.Double point) {

        if (model.isControlPtIsSelected()) {
            Iterator flows = model.flowIterator();
            while (flows.hasNext()) {
                QuadraticBezierFlow flow = ((QuadraticBezierFlow) flows.next());
                if (flow.isSelected()) {
                    Point cPt = flow.getCtrlPt();
                    if (cPt.isSelected()) {
                        cPt.x += (point.x - previousDrag_x);
                        cPt.y += (point.y - previousDrag_y);
                    }
                }

            }
            
        } else {
            Iterator nodes = model.nodeIterator();
            while (nodes.hasNext()) {
                Point node = (Point) nodes.next();
                if (node.isSelected()) {
                    node.x += (point.x - previousDrag_x);
                    node.y += (point.y - previousDrag_y);
                }
            }
        }

        // Set the longestFlowLength needed to compute the maximum intermediate
        // nodes. This may slow things down a lot.
        model.setLongestFlowLength();
        
        mapComponent.eraseBufferImage();
        mapComponent.repaint();

        previousDrag_x = point.x;
        previousDrag_y = point.y;
    }

    // Constructor
    public MoveTool(AbstractSimpleFeatureMapComponent mapComponent, Model model) {
        super(mapComponent);
        this.model = model;
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
