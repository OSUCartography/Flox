/*
 * SelectionTool.java
 *
 * Created on April 7, 2005, 7:21 PM
 */
package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import edu.oregonstate.cartography.utils.CursorUtils;
import java.awt.geom.*;
import java.awt.event.*;

/**
 * PanTool - a tool to pan a map with the mouse.
 *
 * @author Bernhard Jenny, Institute of Cartography, ETH Zurich.
 */
public class PanTool extends MapTool {

    /**
     * Store the start position of the panning.
     */
    private Point2D.Double dragStartPos;

    /**
     * Creates a new instance of SelectionTool
     *
     * @param mapComponent The MapComponent for which this MapTool provides its
     * services.
     */
    public PanTool(AbstractSimpleFeatureMapComponent mapComponent) {
        super(mapComponent);
    }

    /**
     * Utility method to set the cursor icon to a closed hand.
     */
    private void setClosedHandCursor() {
        final String iconName = "panclicked";
        CursorUtils.setCursor(iconName, mapComponent);
    }

    /**
     * The mouse was pressed down, while this MapTool was the active one.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void mouseDown(Point2D.Double point, MouseEvent evt) {
        // change cursor to closed hand
        setClosedHandCursor();
    }

    /**
     * The mouse was clicked, while this MapTool was the active one.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void mouseClicked(Point2D.Double point, MouseEvent evt) {
        if (evt.getClickCount() == 2) {
            mapComponent.zoomIn(point);
        }
        setDefaultCursor();
    }

    /**
     * The mouse starts a drag, while this MapTool is the active one.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void startDrag(Point2D.Double point, MouseEvent evt) {
        // store the start location of the drag.
        dragStartPos = (Point2D.Double) point.clone();
    }

    /**
     * The mouse location changed during a drag, while this MapTool was the
     * active tool.
     *
     * @param point The location of the mouse in world coordinates.
     * @param evt The original event.
     */
    @Override
    public void updateDrag(Point2D.Double point, MouseEvent evt) {
        // just in case we didn't get a mousePressed event
        if (dragStartPos == null) {
            dragStartPos = (Point2D.Double) point.clone();
        } else {
            double dx = dragStartPos.getX() - point.getX();
            double dy = dragStartPos.getY() - point.getY();
            mapComponent.offsetVisibleArea(dx, dy);
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
        this.dragStartPos = null;
        // change cursor to open hand
        setDefaultCursor();
    }

    @Override
    protected String getCursorName() {
        return "pan";
    }
}
