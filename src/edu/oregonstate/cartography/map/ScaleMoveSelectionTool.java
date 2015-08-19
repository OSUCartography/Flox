/*
 * RotateScaleMoveSelectionTool.java
 *
 * Created on May 31, 2006, 11:15 AM
 *
 */

package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.Undo;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import javax.swing.JButton;
import javax.swing.JFormattedTextField;


/**
 * A tool to select objects (by clicking or by rectangular dragging), to move,
 * scale and rotate objects.
 * @author Bernhard Jenny, Institute of Cartography, ETH Zurich.
 */
public class ScaleMoveSelectionTool extends CombinedTool {
    
    /**
     * Creates a new instance of RotateScaleMoveSelectionTool
     */
    public ScaleMoveSelectionTool(AbstractSimpleFeatureMapComponent mapComponent, 
            JFormattedTextField valueField, JFormattedTextField xField, 
            JFormattedTextField yField, JButton lockUnlockButton, Undo undo) {
        super(mapComponent, "Select - Move - Scale");
        
        SelectionTool selectionTool = new SelectionTool(this.mapComponent, 
                valueField, xField, yField, lockUnlockButton);
        
        MoveTool moveTool = new MoveTool(this.mapComponent, xField, yField, 
                lockUnlockButton, undo);
    
        this.addMapTool(moveTool);
        // selection tool must be added last
        this.addMapTool(selectionTool);
    }
}
