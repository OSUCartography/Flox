package edu.oregonstate.cartography.flox.gui;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import edu.oregonstate.cartography.flox.model.CubicBezierFlow;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Layer;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import edu.oregonstate.cartography.flox.model.RangeboxEnforcer;
import edu.oregonstate.cartography.flox.model.VectorSymbol;
import edu.oregonstate.cartography.simplefeature.SimpleFeatureRenderer;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;

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
    private final Color SELECTION_COLOR = Color.decode("#59A4FF");

    /**
     * Width of stroke line for nodes
     */
    private final float NODE_STROKE_WIDTH = 2;

    /**
     * Radius of circles for control points (in pixels)
     */
    private final double CR = 3;

    /**
     * The model containing all the map data
     */
    private final Model model;

    /**
     * Flag to indicate when the flow width is locked to the current map scale.
     */
    private boolean flowWidthLocked = false;

    /**
     * If true GUI elements to indicate selection or locked status are drawn.
     */
    private boolean drawGUIElements = true;

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
     * @param drawGUIElements If true GUI elements to indicate selection or
     * locked status are drawn.
     */
    public FloxRenderer(Model model, Graphics2D g2d,
            double west, double north, double scale,
            boolean drawGUIElements) {
        super(g2d, west, north, scale);
        this.model = model;
        this.drawGUIElements = drawGUIElements;
    }

    /**
     * Renders the flows to an image.
     *
     * @param model The flows to render.
     * @param maxDim The width or the height of the image will have this size.
     * @param bb The bounding box of the map area that will be visible in the
     * image
     * @param antialias If true anti-aliasing is applied.
     * @return The new image.
     */
    public static BufferedImage renderToImage(Model model, int maxDim,
            Rectangle2D bb, boolean antialias, boolean drawGUIElements) {
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

        // enable antialiasing
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                antialias ? RenderingHints.VALUE_ANTIALIAS_ON : RenderingHints.VALUE_ANTIALIAS_OFF);
        // enable high quality rendering
        g2d.setRenderingHint(RenderingHints.KEY_RENDERING,
                RenderingHints.VALUE_RENDER_QUALITY);
        g2d.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,
                RenderingHints.VALUE_STROKE_PURE);
        // enable bicubic interpolation of images
        g2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                RenderingHints.VALUE_INTERPOLATION_BICUBIC);
        // set default appearance of vector elements
        g2d.setStroke(new BasicStroke(1));
        g2d.setColor(Color.black);
        // erase background
        g2d.setBackground(Color.WHITE);
        g2d.clearRect(0, 0, w, h);

        // setup renderer
        FloxRenderer renderer = new FloxRenderer(model, g2d,
                bb.getMinX(), bb.getMaxY(), scale, drawGUIElements);

        // render background layers
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

        // render flows and nodes
        renderer.drawFlows();
        renderer.drawNodes();
        return bufferImage;
    }

    /**
     * Constructs a GeneralPath object for drawing from a CubicBezierFlow
     *
     * @param flow The flow to convert.
     * @return A GeneralPath for drawing.
     */
    private GeneralPath flowToGeneralPath(CubicBezierFlow flow) {
        GeneralPath path = new GeneralPath();
        Point startPt = flow.getStartPt();
        path.moveTo(xToPx(startPt.x), yToPx(startPt.y));
        Point cPt1 = flow.getcPt1();
        Point cPt2 = flow.getcPt2();
        Point endPt = flow.getEndPt();
        path.curveTo(xToPx(cPt1.x), yToPx(cPt1.y),
                xToPx(cPt2.x), yToPx(cPt2.y),
                xToPx(endPt.x), yToPx(endPt.y));
        return path;
    }

    /**
     * Constructs a GeneralPath object for drawing from a QuadraticBezierFlow
     *
     * @param flow The flow to convert.
     * @return A GeneralPath for drawing.
     */
    private GeneralPath flowToGeneralPath(QuadraticBezierFlow flow) {
        if (flow == null) {
            return null;
        }
        GeneralPath path = new GeneralPath();
        Point startPt = flow.getStartPt();
        path.moveTo(xToPx(startPt.x), yToPx(startPt.y));
        Point cPt = flow.getCtrlPt();
        Point endPt = flow.getEndPt();
        path.quadTo(xToPx(cPt.x), yToPx(cPt.y),
                xToPx(endPt.x), yToPx(endPt.y));
        return path;
    }

    /**
     * Draws the flows to the Graphics2D context. Retrieves settings from the
     * model to determine flow width, length, as well as determining whether to
     * apply any clipping or add arrowheads.
     */
    public void drawFlows() {

        // Create an ArrayList to store the flows
        ArrayList<Flow> flows = model.getFlows();

        // Determine location of the end point of flows based on the model
        double r = model.getFlowDistanceFromEndPoint() / scale
                * getLockedScaleFactor();

        // Iterate through the flows
        for (Flow flow : flows) {

            // Create a GeneralPath for the flow
            GeneralPath flowPath;

            // Create and initialize a GeneralPath for the arrowhead
            GeneralPath arrowPath = new GeneralPath();

            // Calculate the stroke width of the flow based on its value.
            // This is done here so that the Arrow class will have access to 
            // the stroke width.
            double flowStrokeWidth = Math.abs(flow.getValue()) * model.getFlowWidthScaleFactor()
                    * getLockedScaleFactor();

            // If flow is a CubicBezierFlow, just set the flowPath to 
            // the flow without any changes. 
            if (flow instanceof CubicBezierFlow) {
                flowPath = flowToGeneralPath((CubicBezierFlow) flow);

            } else {
                QuadraticBezierFlow f = (QuadraticBezierFlow) flow;

                // Draw arrows if the model says so
                if (model.isDrawArrows()) {

                    // Clip the flow by the end node
                    f = clipFlowByEndNode(f);

                    // Clip the flow by the clipping area
                    f = getClippedFlow(f);

                    // Clip the flow by distance from endpoint
                    f = f.split(f.getIntersectionTWithCircleAroundEndPoint(r))[0];

                    // Add the arrow
                    // Instantiate an Arrow object, passing it the first of
                    // the split flows and the model. And everything else.
                    Arrow arrow = new Arrow(f, model, flowStrokeWidth, scale,
                            west, north);

                    // Get the GeneralPath needed to draw the arrowhead
                    arrowPath = arrow.getArrowPath();

                    // Get the clipped flow generated by the Arrow class. The 
                    // Arrow class shortens the flow it was passed based on the
                    // length of the arrowhead.
                    flowPath = flowToGeneralPath(arrow.getOutFlow());

                } else {

                    // Clip the flow by clipping area
                    f = getClippedFlow(f);

                    // Clip the flow by distance from endpoint
                    f = f.split(f.getIntersectionTWithCircleAroundEndPoint(r))[0];

                    // If the model does not specify the drawing of arrows, set
                    // flowPath to the first of splitFlows
                    flowPath = flowToGeneralPath(f);
                }

            }

            g2d.setColor(drawGUIElements && flow.isSelected()
                    ? SELECTION_COLOR : model.getFlowColor());

            // draw the arrow heads
            if (model.isDrawArrows()) {
                g2d.fill(arrowPath);
            }

            // draw the flow
            g2d.setStroke(new BasicStroke((float) flowStrokeWidth,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            g2d.draw(flowPath);

            // draw symbol for locked flows
            if (drawGUIElements && flow.isLocked()) {
                Point pt = flow.pointOnCurve(0.5);
                drawCross(pt.x, pt.y);
            }

        }

        // Iterate through the flows again to draw control points for selected
        // flows. This is done in a separate iteration to prevent control points
        // from being drawn under flows.
        if (drawGUIElements && model.isFlowSelected()) {
            for (Flow flow : flows) {
                if (flow.isSelected()) {
                    drawControlPoints(flow);
                }
            }
        }

    }

    /**
     * Draw all nodes to a Graphics2D context.
     */
    public void drawNodes() {
        g2d.setStroke(new BasicStroke(NODE_STROKE_WIDTH));
        ArrayList<Point> nodes = model.getOrderedNodes(false);
        for (Point node : nodes) {
            double r = getNodeRadius(node);
            Color color = drawGUIElements && node.isSelected()
                    ? SELECTION_COLOR : model.getFlowColor();
            drawCircle(node.x, node.y, r, Color.WHITE, color);
        }
    }

    /**
     * Draws the canvas.
     */
    public void drawCanvas() {

        assert (model.getCanvas() != null);

        g2d.setStroke(new BasicStroke(1));
        Rectangle2D canvas = model.getCanvas();

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

    public void drawFlowRangebox() {
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
                        xToPx(box[2].x), yToPx(box[2].y),
                        xToPx(box[3].x), yToPx(box[3].y));
                Line2D line3 = new Line2D.Double(
                        xToPx(box[0].x), yToPx(box[0].y),
                        xToPx(box[2].x), yToPx(box[2].y));
                Line2D line4 = new Line2D.Double(
                        xToPx(box[1].x), yToPx(box[1].y),
                        xToPx(box[3].x), yToPx(box[3].y));

                g2d.draw(line1);
                g2d.draw(line2);
                g2d.draw(line3);
                g2d.draw(line4);
            }

        }
    }

    /**
     * Call drawControlPoint for each flow in the model.
     */
    public void drawControlPoints() {
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            drawControlPoints(flow);
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
        if (flow instanceof CubicBezierFlow) {
            Point cpt1 = ((CubicBezierFlow) flow).getcPt1();
            Point cpt2 = ((CubicBezierFlow) flow).getcPt2();
            g2d.setColor(Color.GRAY);
            Line2D line1 = new Line2D.Double(xToPx(startPt.x),
                    yToPx(startPt.y), xToPx(cpt1.x), yToPx(cpt1.y));
            g2d.draw(line1);
            Line2D line2 = new Line2D.Double(xToPx(endPt.x),
                    yToPx(endPt.y), xToPx(cpt2.x), yToPx(cpt2.y));
            g2d.draw(line2);
            drawCircle(cpt1.x, cpt1.y, CR, Color.ORANGE, Color.GRAY);
            drawCircle(cpt2.x, cpt2.y, CR, Color.ORANGE, Color.GRAY);
        } else {
            Point cpt = ((QuadraticBezierFlow) flow).getCtrlPt();
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
    public void drawStraightLinesSegments() {

        Iterator<Flow> iter = model.flowIterator();

        // If there are no flows, stop
        if (!iter.hasNext()) {
            return;
        }

        setStrokeWidth(2f);
        double deCasteljauTol = model.getDeCasteljauTolerance();

        while (iter.hasNext()) {
            Flow flow = iter.next();
            ArrayList<Point> points = flow.toStraightLineSegments(deCasteljauTol);
            for (Point point : points) {
                drawCircle(point.x, point.y, CR, Color.pink, Color.white);
            }
        }
    }

    public void drawClipAreas(boolean drawStartClipAreas, boolean drawEndClipAreas) {
        ArrayList<Flow> flows = model.getFlows();
        HashSet<Geometry> endClipAreas = new HashSet<>();
        HashSet<Geometry> startClipAreas = new HashSet<>();
        for (Flow flow : flows) {
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

    private double getLockedScaleFactor() {
        if (!model.isFlowWidthLocked()) {
            return 1;
        } else {
            // compare the locked scale to the current scale
            double lockedMapScale = model.getLockedMapScale();
            return scale / lockedMapScale;
        }
    }

    private QuadraticBezierFlow getClippedFlow(QuadraticBezierFlow flow) {
        double deCasteljauTol = model.getDeCasteljauTolerance();
        flow = flow.getClippedFlow(deCasteljauTol);
        return flow;
    }

    private QuadraticBezierFlow clipFlowByEndNode(QuadraticBezierFlow flow) {

        // Scale the node's radius + stroke/2 distance to world distance
        double nodeR = ((NODE_STROKE_WIDTH / 2) + getNodeRadius(flow.getEndPt())) / scale;

        // Clip the flow by that distance.
        double t = flow.getIntersectionTWithCircleAroundEndPoint(nodeR);
        return flow.split(t)[0];
    }

    /**
     * Get a node's radius in pixels for drawing.
     *
     * @param node
     * @return
     */
    private double getNodeRadius(Point node) {
        double area = Math.abs(node.getValue()
                * model.getNodeSizeScaleFactor());

        return (Math.sqrt(area / Math.PI)) * getLockedScaleFactor();
    }

}
