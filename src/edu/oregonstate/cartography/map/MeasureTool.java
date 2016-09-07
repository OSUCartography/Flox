/*
 * PenTool.java
 *
 * Created on April 21, 2005, 6:10 PM
 */

package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxRenderer;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Cursor;
import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;


/**
 * MeasureTool - a tool to measure distances between two points.
 * @author Bernhard Jenny, Institute of Cartography, ETH Zurich.
 */
public class MeasureTool extends DoubleBufferedTool {
    
    /**
     * The location to start measuring.
     */
    private Point2D.Double dragStartPos;
    /**
     * The current end position of the measured track.
     */
    private Point2D.Double dragCurrentPos;
    
    /**
     * A set of MeasurToolListeners that will be informed when a new distance has
     * been computed.
     */
    private final Set listeners = new HashSet();
    
    /**
     * Create a new instance.
     * @param mapComponent The MapComponent for which this MapTool provides its services.
     */
    public MeasureTool(AbstractSimpleFeatureMapComponent mapComponent) {
        super(mapComponent);
    }
    
    @Override
    public void deactivate() {
        reportClearDistance();
        mapComponent.repaint();
    }
    
    /**
     * Adds a MeasureToolListener.
     * @param listener The MeasureToolListener to add.
     */
    public void addMeasureToolListener(MeasureToolListener listener) {
        if (listener == null) {
            throw new IllegalArgumentException();
        }
        listeners.add(listener);
    }
    
    /**
     * Removes a MeasureToolListener.
     * @param listener The MeasureToolListener to remove.
     */
    public void removeMeasureToolListener(MeasureToolListener listener) {
        listeners.remove(listener);
    }
    
    /**
     * Inform all registered MeasureToolListeners of a new distance.
     */
    private void reportDistance(boolean finalDistance) {
        if (dragStartPos == null || dragCurrentPos == null)
            return;
        
        double dx = dragCurrentPos.x - dragStartPos.x;
        double dy = dragCurrentPos.y - dragStartPos.y;
        double d = Math.sqrt(dx*dx+dy*dy);
        double angle = Math.atan2(dy, dx);
        
        Iterator iterator = listeners.iterator();
        while (iterator.hasNext()) {
            MeasureToolListener listener = (MeasureToolListener)iterator.next();
            if (finalDistance)
                listener.newDistance(d, angle, mapComponent);
            else
                listener.distanceChanged(d, angle, mapComponent);
        }
    }
    
    /**
     * Inform all registered MeasureToolListeners that the distance is not valid anymore.
     */
    private void reportClearDistance() {
        Iterator iterator = this.listeners.iterator();
        while (iterator.hasNext()) {
            MeasureToolListener listener = (MeasureToolListener)iterator.next();
            listener.clearDistance();
        }
    }
    
    /**
     * The mouse was pressed down, while this MapTool was the active one.
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void mouseDown(Point2D.Double point, MouseEvent evt) {
        setMeasureCursor();
        captureBackground();
    }
    
    /**
     * The mouse starts a drag, while this MapTool was the active one.
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void startDrag(Point2D.Double point, MouseEvent evt) {
        setMeasureCursor();
        this.dragStartPos = (Point2D.Double)point.clone();
    }
    
    /**
     * The mouse location changed during a drag, while this MapTool was the active one.
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void updateDrag(Point2D.Double point, MouseEvent evt) {
        // just in case we didn't get a mousePressed-Event
        if (dragStartPos == null) {
            dragStartPos = (Point2D.Double)point.clone();
            setMeasureCursor();
            return;
        }
        
        // if this is the first time mouseDragged is called, capture the screen.
        if (dragCurrentPos == null)
            captureBackground();
        
        dragCurrentPos = (Point2D.Double)point.clone();
        mapComponent.repaint();
        
        reportDistance(false);
    }
    
    /**
     * A drag ends, while this MapTool was the active one.
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void endDrag(Point2D.Double point, MouseEvent evt) {
        /*
        dragStartPos = null;
        dragCurrentPos = null;
         */
        reportDistance(true);
        releaseBackground();
        mapComponent.refreshMap();
        setDefaultCursor();
    }
    
    /**
     * Treat escape key events. Stop drawing the rectangle and revert to the
     * previous state, without doing anything.
     * The event can be consumed (return true) or be delegated to other
     * listeners (return false).
     * @param keyEvent The new key event.
     * @return True if the key event has been consumed, false otherwise.
     */
    @Override
    public boolean keyEvent(KeyEvent keyEvent) {
        boolean keyReleased = keyEvent.getID() == KeyEvent.KEY_RELEASED;
        boolean isEscapeKey = keyEvent.getKeyCode() == KeyEvent.VK_ESCAPE;
        if (keyReleased && isEscapeKey) {
            dragStartPos = null;
            dragCurrentPos = null;
            releaseBackground();
            mapComponent.repaint();
            setDefaultCursor();
            reportClearDistance();
        }
        return false;
    }
    
    /**
     * Draw the interface elements of this MapTool.
     */
    @Override
    public void draw(Graphics2D g2d) {
        if (dragStartPos == null || dragCurrentPos == null) {
            return;
        }
        
        FloxRenderer.enableHighQualityRenderingHints(g2d, true);
        
        g2d.setColor(Color.black);
        BasicStroke stroke = new BasicStroke(1);
        g2d.setStroke(stroke);
        double x1 = mapComponent.xToPx(dragStartPos.getX());
        double y1 = mapComponent.yToPx(dragStartPos.getY());
        double x2 = mapComponent.xToPx(dragCurrentPos.getX());
        double y2 = mapComponent.yToPx(dragCurrentPos.getY());
        
        Line2D line = new Line2D.Double(x1, y1, x2, y2);
        g2d.draw(line);
    }
    
    /**
     * Utility method to change the cursor to a cross-hair cursor.
     */
    private void setMeasureCursor() {
        mapComponent.setCursor(new Cursor(Cursor.CROSSHAIR_CURSOR));
    }
}
