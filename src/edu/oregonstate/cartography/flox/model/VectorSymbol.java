package edu.oregonstate.cartography.flox.model;

import java.awt.Color;

/**
 * A symbol for drawing vector lines and polygons.
 * 
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class VectorSymbol {
     /**
     * color of stroked lines
     */
    private Color strokeColor = Color.LIGHT_GRAY;
    
    /**
     * If true geometry is stroked.
     */
    private boolean stroked = true;
    
    /**
     * color of filling
     */
    private Color fillColor = Color.decode("#F3F3F3");
    
    /**
     * If true, geometry is filled.
     */
    private boolean filled = true;

    /**
     * @return the strokeColor
     */
    public Color getStrokeColor() {
        return strokeColor;
    }

    /**
     * @param strokeColor the strokeColor to set
     */
    public void setStrokeColor(Color strokeColor) {
        this.strokeColor = strokeColor;
    }

    /**
     * @return the stroked
     */
    public boolean isStroked() {
        return stroked;
    }

    /**
     * @param stroked the stroked to set
     */
    public void setStroked(boolean stroked) {
        this.stroked = stroked;
    }

    /**
     * @return the fillColor
     */
    public Color getFillColor() {
        return fillColor;
    }

    /**
     * @param fillColor the fillColor to set
     */
    public void setFillColor(Color fillColor) {
        this.fillColor = fillColor;
    }

    /**
     * @return the filled
     */
    public boolean isFilled() {
        return filled;
    }

    /**
     * @param filled the filled to set
     */
    public void setFilled(boolean filled) {
        this.filled = filled;
    }
}
