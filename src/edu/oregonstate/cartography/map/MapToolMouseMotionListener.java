/*
 * MapToolMouseMotionListener.java
 *
 * Created on May 15, 2005, 6:47 PM
 */

package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.geom.Point2D;

/**
 *
 * @author jenny
 */
public interface MapToolMouseMotionListener {
    public void mouseMoved(Point2D.Double point, AbstractSimpleFeatureMapComponent mapComponent);    
}
