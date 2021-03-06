/*
 * DoubleBufferedTool.java
 *
 * Created on April 11, 2005, 3:19 PM
 */

package edu.oregonstate.cartography.map;

import edu.oregonstate.cartography.flox.gui.FloxRenderer;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import edu.oregonstate.cartography.utils.ImageUtils;
import java.awt.image.*;
import java.awt.*;

/**
 * DoubleBuffereTool - an abstract tool that provides quick drawing of the map
 * background based on a buffered image.
 * 
 * The DoubleBufferedTool does not store its own copy of an image, but simply
 * uses the MapComponents double buffer.
 * @author Bernhard Jenny, Institute of Cartography, ETH Zurich.
 */
public abstract class DoubleBufferedTool extends MapTool {
    
    /**
     * Static flag to toggle debugging information on and off.
     */
    private static final boolean VERBOSE = false;
    
    /**
     * A VolatileImage offers hardware supported drawing. Should be faster.
     */
    private VolatileImage volatileImage = null;
    
    /**
     * Flag to remember whether this tool captured the background.
     */
    private boolean backgroundCaptured = false;
    
    /** Creates a new instance of DoubleBufferedTool
     * @param mapComponent The MapComponent for which this MapTool provides its services.
     */
    protected DoubleBufferedTool(AbstractSimpleFeatureMapComponent mapComponent) {
        super(mapComponent);
    }
    
    
    /**
     * Capture the background. Until releaseBackground() is called, 
     * isCapturingBackground() will return true.
     */
    protected void captureBackground() {
        this.backgroundCaptured = true;
        
        if (VERBOSE)
            System.out.println("captured background");
    }
    
    /**
     * Releases the previously captured background.
     * Also release the volatile image used for fast drawing of the background.
     */
    protected void releaseBackground() {
        this.backgroundCaptured = false;
        
        if (volatileImage != null) {
            volatileImage.flush();
        }
        volatileImage = null;
        
        if (VERBOSE){
            System.out.println("released background");
        }
    }
    
    /**
     * Returns whether this MapTool is currently drawing a double buffered background image.
     * @return True if double buffered background image is used for drawing.
     */
    public boolean isCapturingBackground() {
        return this.backgroundCaptured;
    }
    
    /**
     * Draws the previously captured double-buffered background image.
     * @param g2d The destination to draw to.
     * @return Always returns true to indicate that the map was completely painted.
     */
    @Override
    public boolean drawBackground(Graphics2D g2d) {
        if (!this.backgroundCaptured) {
            return false;
        }
        FloxRenderer.enableFastRenderingHints(g2d);
        
        Insets insets = mapComponent.getInsets();
        BufferedImage backImg = mapComponent.getBufferImage();
        volatileImage = ImageUtils.drawVolatileImage(g2d, volatileImage, 
                insets.left, insets.top, backImg);
        
        if (VERBOSE) {
            System.out.println("drawing captured background");
        }
        
        return true;
    }
    
}
