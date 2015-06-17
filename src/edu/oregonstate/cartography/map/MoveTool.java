package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.geom.Point2D;

/**
 *
 * @author danielstephen
 */
public class MoveTool extends MapTool implements CombinableTool {

    
    
    public MoveTool(AbstractSimpleFeatureMapComponent mapComponent) {
        super(mapComponent);
    }

    @Override
    public boolean adjustCursor(Point2D.Double point) {

        return true;
    }
    
}
