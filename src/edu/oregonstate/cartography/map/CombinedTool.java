/*
 * CombinedTool.java
 *
 * Created on June 11, 2006, 1:54 PM
 *
 */

package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Iterator;


/**
 * A combination of various map tools.
 * @author jenny
 */
public class CombinedTool extends MapTool{
    
    ArrayList tools = new ArrayList();
    
    String name = "";
    
    /** Creates a new instance of CombinedTool
     * @param mapComponent
     * @param name */
    public CombinedTool(AbstractSimpleFeatureMapComponent mapComponent, String name) {
        super(mapComponent);
        this.name = name;
    }
    
    public void addMapTool(CombinableTool mapTool) {
        this.tools.add(mapTool);
    }
    
    @Override
    public void deactivate() {
        try {
            super.deactivate();
        } finally {
            
            Iterator iterator = tools.iterator();
            while (iterator.hasNext()) {
                ((MapTool)iterator.next()).deactivate();
            }
        }
    }
    
    @Override
    public void pause(){
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            ((MapTool)iterator.next()).pause();
        }
    }
    
    @Override
    public void resume(){
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            ((MapTool)iterator.next()).resume();
        }
    }
    
    
    @Override
    public void mouseClicked(Point2D.Double point, MouseEvent evt) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            ((MapTool)iterator.next()).mouseClicked(point, evt);
        }
    }
    
    @Override
    public void mouseDown(Point2D.Double point, MouseEvent evt) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            ((MapTool)iterator.next()).mouseDown(point, evt);
        }
    }
    
    @Override
    public void mouseMoved(Point2D.Double point, MouseEvent evt) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            MapTool tool = ((MapTool)iterator.next());
            tool.mouseMoved(point, evt);
        }
        
        // adjust the cursor
        iterator = tools.iterator();
        boolean cursorAdjusted = false;
        while (iterator.hasNext()) {
            CombinableTool tool = ((CombinableTool)iterator.next());
            if (cursorAdjusted |= tool.adjustCursor(point))
                break;
        }
        if (!cursorAdjusted)
            this.setDefaultCursor();
    }
    
    @Override
    public void mouseEntered(Point2D.Double point, MouseEvent evt) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            ((MapTool)iterator.next()).mouseEntered(point, evt);
        }
    }
    
    @Override
    public void mouseExited(Point2D.Double point, MouseEvent evt) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            ((MapTool)iterator.next()).mouseExited(point, evt);
        }
    }
    
    @Override
    public void startDrag(Point2D.Double point, MouseEvent evt) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            MapTool mapTool = (MapTool)iterator.next();
            mapTool.startDrag(point, evt);
            if (mapTool.isDragging())
                break;
        }
    }
    
    @Override
    public void updateDrag(Point2D.Double point, MouseEvent evt) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            MapTool mapTool = (MapTool)iterator.next();
            if (mapTool.isDragging()) {
                mapTool.updateDrag(point, evt);
                break;
            }
        }
    }
    
    @Override
    public void endDrag(Point2D.Double point, MouseEvent evt) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            MapTool mapTool = (MapTool)iterator.next();
            if (mapTool.isDragging()) {
                mapTool.endDrag(point, evt);
                break;
            }
        }
    }
    
    /**
     * Treat key events.
     * The event can be consumed (return true) or be delegated to other
     * listeners (return false).
     * @param keyEvent The new key event.
     * @return True if the key event has been consumed, false otherwise.
     */
    @Override
    public boolean keyEvent(KeyEvent keyEvent) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            final MapTool mapTool = (MapTool)iterator.next();
            final boolean consumed = mapTool.keyEvent(keyEvent);
            if (consumed)
                return true;
        }
        return false;
    }
    
    @Override
    public void draw(Graphics2D g2d) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            ((MapTool)iterator.next()).draw(g2d);
        }
    }
    
    @Override
    public boolean drawBackground(java.awt.Graphics2D g2d) {
        Iterator iterator = tools.iterator();
        while (iterator.hasNext()) {
            MapTool mapTool = (MapTool)iterator.next();
            if (mapTool.drawBackground(g2d))
                return true;
        }
        return false;
    }
}
