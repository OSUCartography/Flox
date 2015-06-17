package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
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
    
    @Override
    public void startDrag(Point2D.Double point, MouseEvent evt) {
        
        if(VERBOSE) {
            System.out.println("MoveTool: 'Started a drag'");
        }
        
        if(model.isSomethingIsSelected()) {
            dragging = true;
        }
        
    }
    
    @Override
    public void updateDrag(Point2D.Double point, MouseEvent evt) {
        
        updateLocation(point);
        
        if(VERBOSE){
            System.out.println("dragging...");
        }
    }
    
    @Override
    public void endDrag(Point2D.Double point, MouseEvent evt) {
        super.endDrag(point, evt);
        if(VERBOSE) {
            System.out.println("Ended a drag");
        }
        
        dragging = false;
    }
    
    /**
     * Captures the world coordinates of the location where the mouse was first 
     * pressed down
     * @param point
     * @param evt 
     */
    @Override
    public void mouseDown(Point2D.Double point, MouseEvent evt) {
        last_x = point.x;
        last_y = point.y;
        
        if(VERBOSE) {
            System.out.println("mouseDown at: " + last_x + " " + last_y);
        }
    }
    
    /**
     * 
     * @param e 
     */
    public void updateLocation(Point2D.Double point) {
        Iterator nodes = model.nodeIterator();
        while (nodes.hasNext()){
            Point node = (Point) nodes.next();
            if (node.isSelected()){
                node.x = (point.x);
                node.y = (point.y);
            }
        }
        mapComponent.eraseBufferImage();
        mapComponent.repaint();
        
        if(VERBOSE) {
            System.out.println("Updating...");
        }
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
