package edu.oregonstate.cartography.flox.gui;

import com.vividsolutions.jts.geom.GeometryCollection;
import edu.oregonstate.cartography.flox.model.Layer;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.VectorSymbol;
import edu.oregonstate.cartography.map.MapTool;
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

    /**
     * flag for drawing control points
     */
    private boolean drawControlPoints = false;

    /**
     * flag for drawing line segments
     */
    private boolean drawLineSegments = false;

    /**
     * flag for drawing canvas outline
     */
    private boolean drawCanvas = false;

    /**
     * flag for drawing flow rangebox
     */
    private boolean drawFlowRangebox = false;

    /**
     * flag for drawing clip areas around ends of flows
     */
    private boolean drawEndClipAreas = false;

    /**
     * flag for drawing clip areas around starts of flows
     */
    private boolean drawStartClipAreas = false;

    /**
     * Flag to indicate when the flow width is locked to the current map scale.
     */
    private boolean flowWidthLocked = false;

    /**
     * The map scale at the time it was locked.
     */
    private double lockedScale;

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

        // Give the current MapTool a chance to draw some background drawing.
        // Returns true if the the tool also painted the map, i.e. there is no
        // need to paint the map.
        MapTool mapTool = getMapTool();
        boolean toolPaintedMap = mapTool == null ? false : mapTool.drawBackground(g2d);

        // paint the map if this has not been done by the current MapTool
        if (toolPaintedMap == false) {
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

            if (isDrawFlowRangebox()) {
                renderer.drawFlowRangebox();
            }

            if (isDrawCanvas()) {
                renderer.drawCanvasBorder();
            }
            if (isDrawControlPoints()) {
                renderer.drawControlPoints();
            }
            if (isDrawLineSegments()) {
                renderer.drawStraightLinesSegments();
            }

            if (drawStartClipAreas || drawEndClipAreas) {
                renderer.drawClipAreas(drawStartClipAreas, drawEndClipAreas);
            }
        }

        // copy double buffer image to JComponent
        Insets insets = getInsets();
        ((Graphics2D) g).drawImage(bufferImage, insets.left, insets.top, this);

        // Give the current MapTool a chance to draw some custom graphics.
        if (mapTool != null) {
            mapTool.draw((Graphics2D) g);
        }
    }

    /**
     * Set the model with data to draw.
     *
     * @param model the model to set
     */
    public void setModel(Model model) {
        this.model = model;
    }

    public Model getModel() {
        return model;
    }
    
    /**
     * Delete selected nodes and flows
     *
     * @return True if at least one flow or one node was deleted.
     */
    @Override
    public boolean deleteSelected() {
        if (model.deleteSelectedFlowsAndNodes() > 0) {
            eraseBufferImage();
            repaint();
            return true;
        } else {
            return false;
        }
    }

    /**
     * @return the drawControlPoints
     */
    public boolean isDrawControlPoints() {
        return drawControlPoints;
    }

    public boolean isDrawCanvas() {
        return drawCanvas;
    }

    public void setDrawCanvas(boolean drawCanvas) {
        this.drawCanvas = drawCanvas;
    }

    public boolean isDrawFlowRangebox() {
        return drawFlowRangebox;
    }

    /**
     *
     * @param drawControlPoints
     */
    public void setDrawControlPoints(boolean drawControlPoints) {
        this.drawControlPoints = drawControlPoints;
    }

    /**
     * @return the drawLineSegments
     */
    public boolean isDrawLineSegments() {
        return drawLineSegments;
    }

    public void setDrawLineSegments(boolean drawLineSegments) {
        this.drawLineSegments = drawLineSegments;
    }

    /**
     * @param drawFlowRangebox the drawFlowRangebox to set
     */
    public void setDrawFlowRangebox(boolean drawFlowRangebox) {
        this.drawFlowRangebox = drawFlowRangebox;
    }

    /**
     * @return the drawEndClipAreas
     */
    public boolean isDrawEndClipAreas() {
        return drawEndClipAreas;
    }

    /**
     * @param drawEndClipAreas the drawEndClipAreas to set
     */
    public void setDrawEndClipAreas(boolean drawEndClipAreas) {
        this.drawEndClipAreas = drawEndClipAreas;
    }

    /**
     * @return the drawStartClipAreas
     */
    public boolean isDrawStartClipAreas() {
        return drawStartClipAreas;
    }

    /**
     * @param drawStartClipAreas the drawStartClipAreas to set
     */
    public void setDrawStartClipAreas(boolean drawStartClipAreas) {
        this.drawStartClipAreas = drawStartClipAreas;
    }
}
