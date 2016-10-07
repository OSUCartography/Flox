/*
 * RectangleTool.java
 *
 * Created on April 8, 2005, 2:47 PM
 */
package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxRenderer;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

/**
 * RectangleTool - an abstract tool that draws a rectangle when dragging the
 * mouse.
 *
 * @author Bernhard Jenny, Institute of Cartography, ETH Zurich.
 */
public abstract class RectangleTool extends DoubleBufferedTool {

    /**
     * The start position of the drag.
     */
    protected Point2D.Double dragStartPos;

    /**
     * The current position of the drag.
     */
    protected Point2D.Double dragCurrentPos;

    /**
     * The dash length used for drawing the rectangle with dashes.
     */
    protected static final int DASH_LENGTH = 3;

    /**
     * MIN_RECT_DIM_PX is used by isRectangleLargeEnough to test whether the
     * currently drawn rectangle is considered to be large enough. E.g. with the
     * magnifier tool, if the user only draws a rectangle of 1x1 pixel, the view
     * will not zoom to such a small area, which would be confusing. Unit:
     * screen pixel
     */
    protected static final int MIN_RECT_DIM_PX = 3;

    /**
     * Create a new instance.
     *
     * @param mapComponent The map component for which this MapTool provides
     * services.
     */
    protected RectangleTool(AbstractSimpleFeatureMapComponent mapComponent) {
        super(mapComponent);
    }

    /**
     * The mouse starts a drag, while this MapTool was the active one.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void startDrag(Point2D.Double point, MouseEvent evt) {
        dragStartPos = (Point2D.Double) point.clone();
    }

    /**
     * The mouse location changed during a drag, while this MapTool was the
     * active one.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void updateDrag(Point2D.Double point, MouseEvent evt) {
        // just in case we didn't get a mousePressed-Event
        if (dragStartPos == null) {
            dragStartPos = (Point2D.Double) point.clone();
            return;
        }

        // if this is the first time mouseDragged is called, capture the screen.
        if (dragCurrentPos == null) {
            captureBackground();
        }

        dragCurrentPos = (Point2D.Double) point.clone();
        mapComponent.repaint();
    }

    /**
     * A drag ends, while this MapTool was the active one.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void endDrag(Point2D.Double point, MouseEvent evt) {
        // release the corner points of the drag rectangle
        this.dragStartPos = this.dragCurrentPos = null;

        releaseBackground();
        mapComponent.repaint();
    }

    /**
     * Returns whether the tool is currently dragging.
     *
     * @return
     */
    @Override
    public boolean isDragging() {
        return dragStartPos != null;
    }

    /**
     * The mouse was clicked, while this MapTool was the active one.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void mouseClicked(Point2D.Double point, MouseEvent evt) {
        this.dragStartPos = this.dragCurrentPos = null;
        releaseBackground();
        mapComponent.repaint();
        mapComponent.requestFocus();
    }

    /**
     * Treat escape key events. Stop drawing the rectangle and revert to the
     * previous state, without doing anything. The event can be consumed (return
     * true) or be delegated to other listeners (return false).
     *
     * @param keyEvent The new key event.
     * @return True if the key event has been consumed, false otherwise.
     */
    @Override
    public boolean keyEvent(KeyEvent keyEvent) {
        boolean keyReleased = keyEvent.getID() == KeyEvent.KEY_RELEASED;
        boolean isEscapeKey = keyEvent.getKeyCode() == KeyEvent.VK_ESCAPE;

        if (keyReleased && isEscapeKey) {
            // release the corner points of the drag rectangle
            this.dragStartPos = this.dragCurrentPos = null;

            // repaint the map
            releaseBackground();
            mapComponent.repaint();
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

        // disable antialiasing
        FloxRenderer.enableFastRenderingHints(g2d);
        
        // setup stroke
        g2d.setColor(Color.black);
        float[] dashArray = new float[]{DASH_LENGTH, DASH_LENGTH};
        BasicStroke stroke = new BasicStroke((float) 1,
                BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, dashArray, 0);
        g2d.setStroke(stroke);

        // draw 4 lines forming a rectangle.
        // drawing individual lines reduces the "marching ants effect".
        Rectangle2D.Double rect = this.getRectangle();
        Line2D line = new Line2D.Double();
        double west = mapComponent.xToPx(rect.getMinX());
        double east = mapComponent.xToPx(rect.getMaxX());
        double south = mapComponent.yToPx(rect.getMinY());
        double north = mapComponent.yToPx(rect.getMaxY());
        line.setLine(west, south, east, south);
        g2d.draw(line);
        line.setLine(east, north, east, south);
        g2d.draw(line);
        line.setLine(west, north, east, north);
        g2d.draw(line);
        line.setLine(west, north, west, south);
        g2d.draw(line);
    }

    /**
     * Returns the rectangle formed by the start location and the current drag
     * location.
     *
     * @return The rectangle.
     */
    protected Rectangle2D.Double getRectangle() {
        if (dragStartPos == null || dragCurrentPos == null) {
            return null;
        }
        double x = Math.min(dragStartPos.getX(), dragCurrentPos.getX());
        double y = Math.min(dragStartPos.getY(), dragCurrentPos.getY());
        double w = Math.abs(dragCurrentPos.getX() - dragStartPos.getX());
        double h = Math.abs(dragCurrentPos.getY() - dragStartPos.getY());
        return new Rectangle2D.Double(x, y, w, h);
    }

    /**
     * Returns whether the the currently drawn rectangle is considered to be
     * large enough. E.g. with the magnifier tool, if the user only draws a
     * rectangle of 1x1 pixel, the view will not zoom to such a small area,
     * which would be confusing.
     * @return 
     */
    protected boolean isRectangleLargeEnough() {
        Rectangle2D.Double rect = getRectangle();
        if (rect == null) {
            return false;
        }
        double minRectDim = MIN_RECT_DIM_PX / mapComponent.getScale();
        return (rect.width >= minRectDim && rect.height >= minRectDim);
    }
}
