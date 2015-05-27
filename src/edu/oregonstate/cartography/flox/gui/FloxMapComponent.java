package edu.oregonstate.cartography.flox.gui;

import com.vividsolutions.jts.geom.GeometryCollection;
import edu.oregonstate.cartography.flox.model.Layer;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.VectorSymbol;
import edu.oregonstate.cartography.map.MapTool;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;

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
     * flag for drawing reconstructed BŽzier curves
     */
    private boolean drawReconstructedBezier = false;

    /**
     * flag for drawing canvas padding border
     */
    private boolean drawCanvasPadding = false;

    /**
     * flag for drawing flow rangebox
     */
    private boolean drawFlowRangebox = false;

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
            if (model.isDrawArrows()) {
                renderer.drawFlowsWithArrows();
            } else {
                renderer.drawFlows();
            }
            renderer.drawNodes();

            if (isDrawFlowRangebox()) {
                renderer.drawFlowRangebox();
            }

            if (isDrawCanvasPadding()) {
                renderer.drawCanvasPadding();
            }
            if (isDrawControlPoints()) {
                renderer.drawControlPoints();
            }
            if (isDrawLineSegments()) {
                renderer.drawStraightLinesSegments();
            }

            if (isDrawCanvasPadding()) {
                renderer.drawCanvasPadding();
            }
            if (isDrawControlPoints()) {
                renderer.drawControlPoints();
            }
            if (isDrawLineSegments()) {
                renderer.drawStraightLinesSegments();
            }

            if (isDrawReconstructedBezier()) {
                renderer.drawRebuiltBezierCurve();
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

    /**
     * @return the drawControlPoints
     */
    public boolean isDrawControlPoints() {
        return drawControlPoints;
    }

    public boolean isDrawCanvasPadding() {
        return drawCanvasPadding;
    }

    public void setDrawCanvasPadding(boolean drawCanvasPadding) {
        this.drawCanvasPadding = drawCanvasPadding;
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
     * @return the drawReconstructedBezier
     */
    public boolean isDrawReconstructedBezier() {
        return drawReconstructedBezier;
    }

    /**
     * @param drawReconstructedBezier the drawReconstructedBezier to set
     */
    public void setDrawReconstructedBezier(boolean drawReconstructedBezier) {
        this.drawReconstructedBezier = drawReconstructedBezier;
    }

    /**
     * @param drawFlowRangebox the drawFlowRangebox to set
     */
    public void setDrawFlowRangebox(boolean drawFlowRangebox) {
        this.drawFlowRangebox = drawFlowRangebox;
    }

    @Override
    public boolean selectByRectangle(Rectangle2D.Double rect, boolean shiftDown) {
        System.out.println("You drew a rectangle!");
        Iterator<Point> nodes = model.nodeIterator();
        while (nodes.hasNext()) {
            Point pt = nodes.next();

            if ((pt.x >= rect.getMinX() && pt.x <= rect.getMaxX())
                    && (pt.y >= rect.getMinY() && pt.y <= rect.getMaxY())) {
                pt.setSelected(true);
            } else {
                pt.setSelected(false);
            }
        }
        repaint();
        return false;
    }
}
