/*
 * RotateScaleMoveSelectionTool.java
 *
 * Created on May 31, 2006, 11:15 AM
 *
 */

package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;


/**
 * A tool to select objects (by clicking or by rectangular dragging), to move,
 * scale and rotate objects.
 * @author Bernhard Jenny, Institute of Cartography, ETH Zurich.
 */
public class ScaleMoveSelectionTool extends CombinedTool {
    
    /**
     * Creates a new instance of RotateScaleMoveSelectionTool
     */
    public ScaleMoveSelectionTool(AbstractSimpleFeatureMapComponent mapComponent) {
        super(mapComponent, "Select - Move - Scale");
        
        SelectionTool selectionTool = new SelectionTool(this.mapComponent);
        MoveTool moveTool = new MoveTool(this.mapComponent);
    
        
        this.addMapTool(moveTool);
        // selection tool must be added last
        this.addMapTool(selectionTool);
    }
}
