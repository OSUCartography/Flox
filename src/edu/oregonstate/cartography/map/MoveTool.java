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
import java.util.ArrayList;
import java.util.Iterator;
import javax.swing.JFormattedTextField;

/**
 *
 * @author danielstephen
 */
public class MoveTool extends DoubleBufferedTool implements CombinableTool {

    private final Model model;
    
    protected final JFormattedTextField xField;
    protected final JFormattedTextField yField;
    
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
     * Called at the start of a mouse drag event. 
     * @param point The location of the drag event.
     * @param evt The drag event.
     */
    @Override
    public void startDrag(Point2D.Double point, MouseEvent evt) {

        if (model.isNodeSelected()) {
            dragging = true;
        }

        if (model.isControlPtSelected()) {
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
        updateCoordinateFields();
    }

    @Override
    public void endDrag(Point2D.Double point, MouseEvent evt) {
        super.endDrag(point, evt);

        dragging = false;

        if(model.isControlPtSelected()) {
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

        if (model.isControlPtSelected()) {
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
        
        
        mapComponent.eraseBufferImage();
        mapComponent.repaint();

        // Update the previousDrax coordinates.
        previousDrag_x = point.x;
        previousDrag_y = point.y;
    }

    // Constructor
    public MoveTool(AbstractSimpleFeatureMapComponent mapComponent, 
            JFormattedTextField xField, JFormattedTextField yField) {
        super(mapComponent);
        this.model = ((FloxMapComponent) mapComponent).getModel();
        this.xField = xField;
        this.yField = yField;
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
