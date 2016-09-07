package edu.oregonstate.cartography.flox.gui;

import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.map.MapTool;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

/**
 *
 * @beaninfo attribute: isContainer false
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class FloxMapComponent extends AbstractSimpleFeatureMapComponent {

    /**
     * The model to draw.
     */
    private Model model;

    /**
     * Flag for drawing flows
     */
    private boolean drawFlows = true;

    /**
     * Flag for drawing nodes
     */
    private boolean drawNodes = true;

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
     * flag for drawing obstacles (areas around nodes and arrowheads)
     */
    private boolean drawObstacles = false;

    /**
     * Flag to indicate when the flow width is locked to the current map scale.
     */
    private boolean flowWidthLocked = false;

    private MainWindow mainWindow;

    public FloxMapComponent() {
    }

    public void setMainWindow(MainWindow mainWindow) {
        this.mainWindow = mainWindow;
    }
    
    public void layout(String undoMessage) {
        mainWindow.layout(undoMessage);
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
        super.paintComponent(g);
        if (model == null) {
            return;
        }

        Graphics2D g2d = getGraphics2DBuffer();
        
        // Give the current MapTool a chance to draw some background drawing.
        // Returns true if the the tool also painted the map, i.e. there is no
        // need to paint the map.
        MapTool mapTool = getMapTool();
        boolean toolPaintedMap = (mapTool == null) ? false : mapTool.drawBackground(g2d);

        // paint the map if this has not been done by the current MapTool
        if (toolPaintedMap == false) {
            FloxRenderer renderer = new FloxRenderer(model, g2d,
                    west, north, scale, bufferImage.getWidth(), bufferImage.getHeight());
            renderer.setStrokeWidth(1f);
            renderer.render(
                    true, // renderBackgroundLayers
                    isDrawCanvas(),
                    isDrawFlows(),
                    isDrawNodes(),
                    isDrawFlowRangebox(),
                    true, // draw control points
                    isDrawLineSegments(),
                    true, // draw symbol for locked flows
                    true, // highlight selected flows and nodes
                    isDrawStartClipAreas(),
                    isDrawEndClipAreas(),
                    isDrawObstacles());
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
            layout("Delete");
            refreshMap();
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns an ArrayList of nodes that were clicked.
     *
     * @param nodes An ArrayList of nodes from which clicked nodes will be
     * returned
     * @param click a Point2D.Double at the location of the click in world coordinates.
     * @param pixelTolerance If the click is within this pixel tolerance of the
     * node, it will be returned. This is to account for the the stroke width of
     * the node drawing. In pixels.
     * @return An ArrayList of nodes that were clicked.
     */
    public ArrayList<Point> getClickedNodes(ArrayList<Point> nodes,
            Point2D.Double click, double pixelTolerance) {
        // Create an empty ArrayList to store clicked nodes
        ArrayList<Point> clickedNodes = new ArrayList<>();

        if (isDrawNodes()) {
            for (int i = nodes.size() - 1; i >= 0; i--) {
                Point node = nodes.get(i);

                double rRefPx = model.getNodeRadiusPx(node) + model.getNodeStrokeWidthPx() / 2;
                double rWorld = rRefPx / model.getReferenceMapScale();
                rWorld += pixelTolerance / scale;
                
                // Calculate the distance of the click from the node center.
                double dx = node.x - click.x;
                double dy = node.y - click.y;
                double distSquared = (dx * dx + dy * dy);

                if (distSquared <= rWorld * rWorld) {
                    // this node was clicked.
                    // add it to clickedNodes
                    clickedNodes.add(node);
                }
            }
        }

        return clickedNodes;
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

    /**
     * @param drawFlows the drawFlows to set
     */
    public void setDrawFlows(boolean drawFlows) {
        this.drawFlows = drawFlows;
    }

    /**
     * @param drawNodes the drawNodes to set
     */
    public void setDrawNodes(boolean drawNodes) {
        this.drawNodes = drawNodes;
    }

    /**
     * @return the drawFlows
     */
    public boolean isDrawFlows() {
        return drawFlows;
    }

    /**
     * @return the drawNodes
     */
    public boolean isDrawNodes() {
        return drawNodes;
    }

    /**
     * @return the drawObstacles
     */
    public boolean isDrawObstacles() {
        return drawObstacles;
    }

    /**
     * @param drawObstacles the drawObstacles to set
     */
    public void setDrawObstacles(boolean drawObstacles) {
        this.drawObstacles = drawObstacles;
    }

}
