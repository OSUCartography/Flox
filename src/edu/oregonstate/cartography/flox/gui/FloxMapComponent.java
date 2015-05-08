package edu.oregonstate.cartography.flox.gui;

import com.vividsolutions.jts.geom.GeometryCollection;
import edu.oregonstate.cartography.flox.model.Layer;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.VectorSymbol;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.geom.Rectangle2D;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class FloxMapComponent extends AbstractSimpleFeatureMapComponent {

    /**
     * The model to draw.
     */
    private Model model;

    public FloxMapComponent() {
    }

    /**
     * Returns the bounding box of the geometry that is drawn by the map.
     *
     * @return The bounding box.
     */
    @Override
    protected Rectangle2D getBoundingBox() {
        return model.getBoundingBox();
    }

    /**
     * Override paintComponent of JComponent for custom drawing.
     *
     * @param g The destination to draw to.
     */
    @Override
    protected void paintComponent(Graphics g) {
        if (model == null) {
            return;
        }

        Graphics2D g2d = getGraphics2DBuffer();
        FloxRenderer renderer = new FloxRenderer(model, g2d, west, north, scale);
        renderer.setStrokeWidth(1f);
        
        // draw background map
        int nbrLayers = model.getNbrLayers();
        for (int i = nbrLayers - 1; i >= 0; i--) {
            Layer layer = model.getLayer(i);
            GeometryCollection geometry = layer.getGeometryCollection();
            VectorSymbol symbol = layer.getVectorSymbol();
            Color fillColor = symbol.isFilled() ? layer.getVectorSymbol().getFillColor() : null;
            Color strokeColor = symbol.isStroked() ? layer.getVectorSymbol().getStrokeColor() : null;
            if (fillColor != null || strokeColor != null) {
                renderer.draw(geometry, fillColor, strokeColor);
            }
        }

        // draw flows and nodes
        renderer.drawFlows();
        renderer.drawNodes();
        if (model.isDrawControlPoints()) {
            renderer.drawControlPoints();
        }
        if (model.isDrawLineSegments()) {
            renderer.drawStraightLinesSegments();
        }

        if (model.isDrawReconstructedBezier()) {
            renderer.drawRebuiltBezierCurve();
        }

        // copy double buffer image to JComponent
        Insets insets = getInsets();
        ((Graphics2D) g).drawImage(bufferImage, insets.left, insets.top, this);
    }

    /**
     * Set the model with data to draw.
     *
     * @param model the model to set
     */
    public void setModel(Model model) {
        this.model = model;
    }

}
