package edu.oregonstate.cartography.flox.gui;

import edu.oregonstate.cartography.flox.model.Arrow;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.ForceLayouter;
import edu.oregonstate.cartography.flox.model.Layer;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.RangeboxEnforcer;
import edu.oregonstate.cartography.flox.model.VectorSymbol;
import edu.oregonstate.cartography.simplefeature.SimpleFeatureRenderer;
import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.Transparency;
import java.awt.geom.AffineTransform;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;

/**
 * A renderer for the Flox data model
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class FloxRenderer extends SimpleFeatureRenderer {

    /**
     * Color for drawing selected flows.
     */
    public static final Color SELECTION_COLOR = Color.decode("#59A4FF");

    /**
     * Color for drawing stroke line of nodes.
     */
    public static final Color NODE_STROKE_COLOR = Color.BLACK;

    /**
     * Color for drawing interior of nodes.
     */
    public static final Color NODE_FILL_COLOR = Color.WHITE;

    /**
     * Radius of circles for control points (in pixels)
     */
    private static final double CR = 3;

    /**
     * The model containing all the map data
     */
    private final Model model;

    /**
     * the width of the drawable area of the map component
     */
    private final int canvasWidth;

    /**
     * the height of the drawable area of the map component
     */
    private final int canvasHeight;

    /**
     * Creates a new renderer.
     *
     * @param model The model to render.
     * @param g2d The graphics context to render to.
     * @param west The left image border corresponds to this world coordinate
     * position.
     * @param north The top image border corresponds to this world coordinate
     * position.
     * @param scale The scale factor to apply when drawing.
     * @param canvasWidth width of the drawable area of the map component
     * @param canvasHeight height of the drawable area of the map component
     */
    public FloxRenderer(Model model, Graphics2D g2d,
            double west, double north, double scale,
            int canvasWidth, int canvasHeight) {
        super(g2d, west, north, scale);
        this.model = model;
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
    }

    public void render(boolean renderBackgroundLayers,
            boolean drawCanvas,
            boolean drawFlows,
            boolean drawNodes,
            boolean drawFlowRangebox,
            boolean drawControlPoints,
            boolean drawLineSegments,
            boolean drawLocks,
            boolean highlightSelected,
            boolean drawStartClipAreas,
            boolean drawEndClipAreas,
            boolean drawObstacles) {

        if (renderBackgroundLayers) {
            int nbrLayers = model.getNbrLayers();
            for (int i = nbrLayers - 1; i >= 0; i--) {
                Layer layer = model.getLayer(i);
                GeometryCollection geometry = layer.getGeometryCollection();
                VectorSymbol symbol = layer.getVectorSymbol();
                Color fillColor = symbol.isFilled() ? layer.getVectorSymbol().getFillColor() : null;
                Color strokeColor = symbol.isStroked() ? layer.getVectorSymbol().getStrokeColor() : null;
                if (fillColor != null || strokeColor != null) {
                    draw(geometry, fillColor, strokeColor);
                }
            }
        }

        if (drawCanvas) {
            drawCanvas();
        }

        if (drawFlows) {
            drawFlows(highlightSelected, drawLocks);
        }

        if (drawNodes) {
            drawNodes(highlightSelected);
        }

        if (drawFlowRangebox) {
            drawFlowRangebox();
        }

        if (drawControlPoints) {
            drawControlPoints();
        }
        if (drawLineSegments) {
            drawStraightLinesSegments();
        }

        if (drawStartClipAreas || drawEndClipAreas) {
            drawClipAreas(drawStartClipAreas, drawEndClipAreas);
        }

        if (drawObstacles) {
            drawObstacles();
        }
    }

    /**
     * Renders the flows to an image.
     *
     * @param model The flows to render.
     * @param maxDim The width or the height of the image will have this size.
     * @param bb The bounding box of the map area that will be visible in the
     * image
     * @param antialias If true anti-aliasing is applied.
     * @param drawBackgroundLayers If true, map layers are rendered.
     * @param fillNodes If true, nodes are filled.
     * @param drawSelectedFlows If false, selected flows are not drawn.
     * @param drawFlows If true, flows are rendered.
     * @param drawNodes If true, nodes are rendered.
     * @return The new image.
     */
    public static BufferedImage renderToImage(Model model, int maxDim,
            Rectangle2D bb,
            boolean antialias,
            boolean drawBackgroundLayers,
            boolean fillNodes,
            boolean drawSelectedFlows,
            boolean drawFlows,
            boolean drawNodes) {

        // find size of fitting image
        //Rectangle2D bb = model.getFlowsBoundingBox();
        double scale = maxDim / Math.max(bb.getWidth(), bb.getHeight());
        int w, h;
        if (bb.getWidth() > bb.getHeight()) {
            w = maxDim;
            h = (int) Math.ceil(bb.getHeight() * scale);
        } else {
            w = (int) Math.ceil(bb.getWidth() * scale);
            h = maxDim;
        }

        // create image
        BufferedImage bufferImage = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2d = bufferImage.createGraphics();

        // set default appearance of vector elements
        g2d.setStroke(new BasicStroke(1));
        g2d.setColor(Color.black);
        // erase background
        g2d.setBackground(Color.WHITE);
        g2d.clearRect(0, 0, w, h);

        // setup renderer
        FloxRenderer renderer = new FloxRenderer(model, g2d,
                bb.getMinX(), bb.getMaxY(), scale, w, h);
        FloxRenderer.enableHighQualityRenderingHints(g2d, antialias);
        renderer.render(drawBackgroundLayers,
                false, // drawCanvas
                drawFlows,
                drawNodes,
                false, // drawFlowRangebox, 
                false, // drawControlPoints 
                false, // drawLineSegments
                false, // drawLocks
                false, // highlightSelected
                false, // drawStartClipAreas
                false, // drawEndClipAreas
                false // drawObstacles
        );

        return bufferImage;
    }

    /**
     * Generate a GeneralPath of the outline of the arrowhead.
     *
     * @return GeneralPath of the arrowhead.
     */
    private GeneralPath getArrowPath(Arrow a) {
        GeneralPath arrowPath = new GeneralPath();
        arrowPath.moveTo(xToPx(a.getBasePt().x), yToPx(a.getBasePt().y));
        arrowPath.lineTo((xToPx(a.getCorner1Pt().x)), (yToPx(a.getCorner1Pt().y)));
        arrowPath.quadTo(xToPx(a.getCorner1cPt().x), yToPx(a.getCorner1cPt().y),
                xToPx(a.getTipPt().x), yToPx(a.getTipPt().y));
        arrowPath.quadTo(xToPx(a.getCorner2cPt().x), yToPx(a.getCorner2cPt().y),
                xToPx(a.getCorner2Pt().x), yToPx(a.getCorner2Pt().y));
        arrowPath.lineTo(xToPx(a.getBasePt().x), yToPx(a.getBasePt().y));
        return arrowPath;
    }

    /**
     * Draws the flows to the Graphics2D context. Retrieves settings from the
     * model to determine flow width, length, as well as determining whether to
     * apply any clipping or add arrowheads.
     *
     * @param highlightSelected If true, selected flows are drawn with
     * SELECTION_COLOR.
     */
    private void drawFlows(boolean highlightSelected, boolean drawLocks) {

        double s = scale / model.getReferenceMapScale();
        boolean colorVaries = !model.getMinFlowColor().equals(model.getMaxFlowColor());

        // Iterate through the flows
        Iterator<Flow> iterator = colorVaries
                ? model.sortedFlowIterator(false) : model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();

            g2d.setColor(highlightSelected && flow.isSelected()
                    ? SELECTION_COLOR : model.getFlowColor(flow));

            // draw the arrow head
            if (model.isDrawArrowheads()) {
                Arrow arrow = flow.getEndArrow();
                if (arrow != null) {
                    g2d.fill(getArrowPath(arrow));
                }
            }

            // draw flow line
            Flow clippedFlow = model.clipFlow(flow, true);
            GeneralPath flowPath = clippedFlow.toGeneralPath(scale, west, north);
            double flowStrokeWidth = model.getFlowWidthPx(flow) * s;
            drawFlowLine(g2d, flow, flowPath, flowStrokeWidth, highlightSelected);

            // draw symbol for locked flow
            if (drawLocks && flow.isLocked()) {
                Point pt = flow.pointOnCurve(0.5);
                drawCross(pt.x, pt.y);
            }
        }
    }

    /**
     * Returns the geometry of an in-line arrow for one flow.
     *
     * @param flow the flow
     * @param flowStrokeWidth the width of the flow line in pixels
     * @return the geometry of the arrow in pixel coordinates.
     */
    private Shape getInlineFlowArrow(Flow flow, double flowStrokeWidth) {
        // default placement is at T + 0.5
        // TODO optimize placement to avoid conflicts with overlapping flows
        double t = 0.5;

        // location on flow line
        Point pt = flow.pointOnCurve(t);
        double x = xToPx(pt.x);
        double y = yToPx(pt.y);

        // distance from dip to center of line on flow line
        double d = flowStrokeWidth / 4;

        // half line width
        double w = flowStrokeWidth / 2;
        // make slightly wider to avoid visual artifacts along border of flow line
        w *= 1.05;
        // displacement in flow direction of end of arrow. If 0, a line
        // perpendicular to the flow line is drawn.
        double s = w;

        // construct right-pointing larger-than sign
        Path2D path = new Path2D.Double();
        // start at tip
        path.moveTo(d, 0);
        path.lineTo(d - s, -w);
        path.lineTo(-d - s, -w);
        path.lineTo(-d, 0);
        path.lineTo(-d - s, w);
        path.lineTo(d - s, w);
        path.closePath();

        // rotate and move
        double rot = flow.getSlope(t);
        AffineTransform trans = AffineTransform.getTranslateInstance(x, y);
        trans.concatenate(AffineTransform.getRotateInstance(-rot));
        path.transform(trans);

        return path;
    }

    /**
     * Draws a flow line
     *
     * @param g2d destination to draw to.
     * @param flow the flow to draw.
     * @param flowPath the geometry of the flow to draw.
     * @param flowStrokeWidth the width of the flow in pixels.
     * @param highlightSelected If true and the flow is selected, it is drawn
     * with SELECTION_COLOR.
     */
    private void drawFlowLine(Graphics2D g2d, Flow flow, GeneralPath flowPath,
            double flowStrokeWidth, boolean highlightSelected) {

        if (model.isDrawInlineArrows()) {
            // draw lines with inline arrows

            // TODO this image should be allocated once and be reused for all flows
            BufferedImage mask = g2d.getDeviceConfiguration().createCompatibleImage(
                    canvasWidth, canvasHeight, Transparency.TRANSLUCENT);
            Graphics2D mask2D = (Graphics2D) mask.getGraphics();
            enableHighQualityRenderingHints(mask2D, true);

            mask2D.setStroke(new BasicStroke((float) flowStrokeWidth,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            mask2D.setColor(highlightSelected && flow.isSelected()
                    ? SELECTION_COLOR : model.getFlowColor(flow));
            mask2D.draw(flowPath);

            // draw to alpha channel to make inner flow area transparent
            mask2D.setComposite(AlphaComposite.getInstance(AlphaComposite.DST_IN));
            mask2D.setColor(new Color(0, true));
            mask2D.fill(getInlineFlowArrow(flow, flowStrokeWidth));

            // draw flow image onto map image
            mask2D.dispose();
            g2d.drawRenderedImage(mask, null);
        } else {
            // will draw plain arrows elsewhere
            g2d.setStroke(new BasicStroke((float) flowStrokeWidth,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            g2d.draw(flowPath);
        }

        // uncomment to test parallel lines
        // double offset = flow.getBaselineLength() / 10;
        // g2d.draw(flow.toGeneralPath(scale, west, north, offset));
    }

    /**
     * Draw all nodes to a Graphics2D context.
     *
     */
    private void drawNodes(boolean highlightSelected) {
        double s = scale / model.getReferenceMapScale();

        double strokeWidthPx = model.getNodeStrokeWidthPx();

        // same stroke width for all nodes
        g2d.setStroke(new BasicStroke((float) (strokeWidthPx * s)));

        ArrayList<Point> nodes = model.getOrderedNodes(false);
        for (Point node : nodes) {
            double nodeRadiusPx = model.getNodeRadiusPx(node);
            double r = nodeRadiusPx * s;
            Color strokeColor;
            if (strokeWidthPx == 0) {
                strokeColor = null;
            } else if (highlightSelected && node.isSelected()) {
                strokeColor = SELECTION_COLOR;
            } else {
                strokeColor = NODE_STROKE_COLOR;
            }
            // if the stroke width is larger than the radius of the circle, the
            // drawing engine does not fill the circle entirely. This fix fills
            // the circle with the stroke color.
            Color fillColor = strokeWidthPx > nodeRadiusPx ? strokeColor : NODE_FILL_COLOR;
            drawCircle(node.x, node.y, r, fillColor, strokeColor);
        }
    }

    /**
     * Draws the canvas.
     */
    private void drawCanvas() {
        g2d.setStroke(new BasicStroke(1));
        Rectangle2D canvas = model.getNodesBoundingBox();

        double cWidth = canvas.getWidth();
        double cHeight = canvas.getHeight();

        // Get the additional padding around the canvas, which is a
        // percentage of the current canvas specified by the model.
        double xPad = cWidth * model.getCanvasPadding();
        double yPad = cHeight * model.getCanvasPadding();

        // Calculate the points of the canvas rectangle, adding the 
        // canvasPadding.
        Point b1 = new Point(
                xToPx(canvas.getX() - xPad),
                yToPx(canvas.getY() - yPad));
        Point b2 = new Point(
                xToPx(canvas.getMaxX() + xPad),
                yToPx(canvas.getY() - yPad));
        Point b3 = new Point(
                xToPx(canvas.getX() - xPad),
                yToPx(canvas.getMaxY() + yPad));
        Point b4 = new Point(
                xToPx(canvas.getMaxX() + xPad),
                yToPx(canvas.getMaxY() + yPad));

        // Construct a GeneralPath from the canvas points
        GeneralPath canvasPath = new GeneralPath();
        canvasPath.moveTo(b1.x, b1.y);
        canvasPath.lineTo(b2.x, b2.y);
        canvasPath.lineTo(b4.x, b4.y);
        canvasPath.lineTo(b3.x, b3.y);
        canvasPath.lineTo(b1.x, b1.y);

        // Draw the canvas
        g2d.setColor(Color.BLACK);
        g2d.draw(canvasPath);
        g2d.setColor(new Color(245, 245, 245));
        g2d.fill(canvasPath);
    }

    private void drawFlowRangebox() {
        g2d.setStroke(new BasicStroke(1f));
        g2d.setColor(Color.GRAY);
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            if (flow.isSelected()) {
                RangeboxEnforcer enforcer = new RangeboxEnforcer(model);
                Point[] box = enforcer.computeRangebox(flow);

                Line2D line1 = new Line2D.Double(
                        xToPx(box[0].x), yToPx(box[0].y),
                        xToPx(box[1].x), yToPx(box[1].y));
                Line2D line2 = new Line2D.Double(
                        xToPx(box[1].x), yToPx(box[1].y),
                        xToPx(box[2].x), yToPx(box[2].y));
                Line2D line3 = new Line2D.Double(
                        xToPx(box[2].x), yToPx(box[2].y),
                        xToPx(box[3].x), yToPx(box[3].y));
                Line2D line4 = new Line2D.Double(
                        xToPx(box[3].x), yToPx(box[3].y),
                        xToPx(box[0].x), yToPx(box[0].y));

                g2d.draw(line1);
                g2d.draw(line2);
                g2d.draw(line3);
                g2d.draw(line4);
            }
        }
    }

    /**
     * Draw the control points of selected flows.
     */
    private void drawControlPoints() {
        // draw just the control points of selected flows.
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isSelected()) {
                drawControlPoints(flow);
            }
        }
    }

    /**
     * Draw the control point of a Bezier curve and lines connecting the control
     * point to the start and end points.
     */
    private void drawControlPoints(Flow flow) {
        g2d.setStroke(new BasicStroke(1f));
        Point startPt = flow.getStartPt();
        Point endPt = flow.getEndPt();
        Point cpt = flow.getCtrlPt();
        g2d.setColor(Color.GRAY);
        Line2D line1 = new Line2D.Double(xToPx(startPt.x),
                yToPx(startPt.y), xToPx(cpt.x), yToPx(cpt.y));
        g2d.draw(line1);
        Line2D line2 = new Line2D.Double(xToPx(endPt.x),
                yToPx(endPt.y), xToPx(cpt.x), yToPx(cpt.y));
        g2d.draw(line2);
        if (cpt.isSelected()) {
            drawCircle(cpt.x, cpt.y, CR, Color.CYAN, Color.GRAY);
        } else {
            drawCircle(cpt.x, cpt.y, CR, Color.ORANGE, Color.GRAY);
        }
    }

    private void drawCross(double x, double y) {
        final double L = 4;

        Line2D line1 = new Line2D.Double(xToPx(x) - L, yToPx(y) - L,
                xToPx(x) + L, yToPx(y) + L);
        Line2D line2 = new Line2D.Double(xToPx(x) - L, yToPx(y) + L,
                xToPx(x) + L, yToPx(y) - L);

        g2d.setStroke(new BasicStroke(2f));
        g2d.setColor(Color.WHITE);
        g2d.draw(line1);
        g2d.draw(line2);

        g2d.setStroke(new BasicStroke(1f));
        g2d.setColor(Color.BLACK);
        g2d.draw(line1);
        g2d.draw(line2);
    }

    /**
     * Draw straight line segments for a Bezier curve. Useful for debugging.
     */
    private void drawStraightLinesSegments() {

        Iterator<Flow> iter = model.flowIterator();

        // If there are no flows, stop
        if (!iter.hasNext()) {
            return;
        }

        setStrokeWidth(2f);
        double deCasteljauTol = model.getDeCasteljauTolerance();

        while (iter.hasNext()) {
            Flow flow = iter.next();
            Flow clippedFlow = model.clipFlow(flow, false);
            ArrayList<Point> points = clippedFlow.toUnclippedStraightLineSegments(deCasteljauTol);
            for (Point point : points) {
                drawCircle(point.x, point.y, CR, Color.pink, Color.white);
            }
        }
    }

    private void drawClipAreas(boolean drawStartClipAreas, boolean drawEndClipAreas) {
        HashSet<Geometry> endClipAreas = new HashSet<>();
        HashSet<Geometry> startClipAreas = new HashSet<>();
        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            endClipAreas.add(flow.getEndClipArea());
            startClipAreas.add(flow.getStartClipArea());
        }

        if (drawStartClipAreas) {
            for (Geometry geometry : startClipAreas) {
                draw(geometry, null, Color.GRAY);
            }
        }

        if (drawEndClipAreas) {
            for (Geometry geometry : endClipAreas) {
                draw(geometry, null, Color.GRAY);
            }
        }
    }

    private void drawObstacles() {
        g2d.setStroke(new BasicStroke(model.getNodeStrokeWidthPx()));
        ForceLayouter layouter = new ForceLayouter(model);
        List<ForceLayouter.Obstacle> obstacles = layouter.getObstacles();
        double s = scale / model.getReferenceMapScale();
        Color fillColor = new Color(200, 0, 0, 80);
        for (ForceLayouter.Obstacle obstacle : obstacles) {
            drawCircle(obstacle.x, obstacle.y, obstacle.r * s, fillColor, null);
        }
    }
}
