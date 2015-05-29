package edu.oregonstate.cartography.flox.gui;

import com.vividsolutions.jts.geom.Geometry;
import edu.oregonstate.cartography.flox.model.CubicBezierFlow;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import edu.oregonstate.cartography.flox.model.RangeboxEnforcer;
import edu.oregonstate.cartography.flox.model.bezier.Bezier;
import edu.oregonstate.cartography.flox.model.bezier.BezierPath;
import edu.oregonstate.cartography.simplefeature.SimpleFeatureRenderer;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
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
     * White border along flows
     */
    private final float WHITE_BORDER = 2;

    /**
     * Width of stroke line for nodes
     */
    private final float NODE_STROKE_WIDTH = 2;

    /**
     * Radius of circles for start and end points (in pixels)
     */
    private final double R = 10;

    /**
     * Radius of circles for control points (in pixels)
     */
    private final double CR = 3;

    private final Model model;

    /**
     * Renders the flows to an image.
     *
     * @param model The flows to render.
     * @param maxDim The width or the height of the image will have this size.
     * @param antialias If true anti-aliasing is applied.
     * @return The new image.
     */
    public static BufferedImage renderToImage(Model model, int maxDim, boolean antialias) {
        // find size of fitting image
        Rectangle2D bb = model.getFlowsBoundingBox();
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
        FloxRenderer renderer = new FloxRenderer(model, g2d, bb.getMinX(), bb.getMaxY(), scale);

        // render to image
        renderer.drawFlows();
        renderer.drawNodes();
        return bufferImage;
    }

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
     */
    public FloxRenderer(Model model, Graphics2D g2d, double west, double north, double scale) {
        super(g2d, west, north, scale);
        this.model = model;
    }

    /**
     * Constructs a GeneralPath object for drawing from a flow
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
     * Constructs a GeneralPath object for drawing from a flow
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
     * Draw all flows to a Graphics2D context.
     *
     */
    public void drawFlows() {
        ArrayList<Flow> flows;
        switch (model.getFlowOrder()) {
            case DECREASING:
                flows = model.getOrderedFlows(false);
                break;
            case INCREASING:
                flows = model.getOrderedFlows(true);
                break;
            default:
                flows = model.getFlows();
        }

        for (Flow flow : flows) {
            GeneralPath path;
            if (flow instanceof CubicBezierFlow) {
                path = flowToGeneralPath((CubicBezierFlow) flow);
            } else {
                path = flowToGeneralPath(((QuadraticBezierFlow) flow).getClippedFlow());
            }
            if (path == null) {
                continue;
            }
            double strokeWidth = Math.abs(flow.getValue()) * model.getFlowWidthScale();
            g2d.setStroke(new BasicStroke((float) strokeWidth + WHITE_BORDER * 2,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            g2d.setColor(Color.WHITE);
            g2d.draw(path);
            g2d.setStroke(new BasicStroke((float) strokeWidth,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            if (flow.isSelected()) {
                g2d.setColor(Color.CYAN);
            } else {
                g2d.setColor(Color.BLACK);
            }

            g2d.draw(path);
        }
    }

    public void drawFlowsWithArrows() {
        ArrayList<Flow> flows;
        double r = model.getFlowArrowEndPointRadius();
        switch (model.getFlowOrder()) {
            case DECREASING:
                flows = model.getOrderedFlows(false);
                break;
            case INCREASING:
                flows = model.getOrderedFlows(true);
                break;
            default:
                flows = model.getFlows();
        }

        for (Flow flow : flows) {
            GeneralPath path;
            GeneralPath arrowPath = new GeneralPath();
            if (flow instanceof CubicBezierFlow) {
                path = flowToGeneralPath((CubicBezierFlow) flow);

            } else {
                QuadraticBezierFlow f = (QuadraticBezierFlow) flow;
                f = f.getClippedFlow();
                if (f == null) {
                    continue;
                }
                double t = f.getIntersectionTWithCircleAroundEndPoint(r);
                QuadraticBezierFlow[] splitFlows = f.split(t);

                // Instantiate an Arrow object, make a GeneralPath from its
                // vertices.
                Arrow arrow = new Arrow((QuadraticBezierFlow) splitFlows[0], model);
                arrowPath.moveTo(xToPx(arrow.base.x), yToPx(arrow.base.y));
                arrowPath.lineTo(xToPx(arrow.corner1.x), yToPx(arrow.corner1.y));

                //arrowPath.lineTo(xToPx(arrow.tip.x), yToPx(arrow.tip.y));
                //arrowPath.lineTo(xToPx(arrow.corner2.x), yToPx(arrow.corner2.y));
                arrowPath.quadTo(xToPx(arrow.corner1cPt.x), yToPx(arrow.corner1cPt.y), xToPx(arrow.tip.x), yToPx(arrow.tip.y));
                arrowPath.quadTo(xToPx(arrow.corner2cPt.x), yToPx(arrow.corner2cPt.y), xToPx(arrow.corner2.x), yToPx(arrow.corner2.y));

                arrowPath.lineTo(xToPx(arrow.base.x), yToPx(arrow.base.y));
                path = flowToGeneralPath(arrow.getFlow());
            }

            // Draw the flow border
            double strokeWidth = Math.abs(flow.getValue()) * model.getFlowWidthScale();
            g2d.setStroke(new BasicStroke((float) strokeWidth + WHITE_BORDER * 2,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            g2d.setColor(Color.WHITE);
            g2d.draw(path);

            //Draw the arrow
            g2d.setStroke(new BasicStroke(3));
            g2d.setColor(Color.WHITE);
            g2d.draw(arrowPath);
            if (flow.isSelected()) {
                g2d.setColor(Color.CYAN);
            } else {
                g2d.setColor(Color.BLACK);
            }
            g2d.fill(arrowPath);

            // Draw the flow fill
            g2d.setStroke(new BasicStroke((float) strokeWidth,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            if (flow.isSelected()) {
                g2d.setColor(Color.CYAN);
            } else {
                g2d.setColor(Color.BLACK);
            }
            g2d.draw(path);
        }
    }

    /**
     * Draw all nodes to a Graphics2D context.
     */
    public void drawNodes() {
        g2d.setStroke(new BasicStroke(NODE_STROKE_WIDTH));
        Iterator<Point> iter = model.nodeIterator();
        while (iter.hasNext()) {
            Point pt = iter.next();
            if (pt.isSelected()) {
                drawCircle(pt.x, pt.y, R, Color.WHITE, Color.CYAN);
            } else {
                drawCircle(pt.x, pt.y, R, Color.WHITE, Color.BLACK);
            }

        }
    }

    public void drawCanvasPadding() {

        g2d.setStroke(new BasicStroke(1));
        Rectangle2D canvas = model.getCanvas();

        if (canvas == null) {
            System.out.println("No Canvas!");
        } else {
            System.out.println("We have a canvas!");

            double cWidth = canvas.getWidth();
            double cHeight = canvas.getHeight();

            // Is a percentage of the canvas size
            double xPad = cWidth * model.getCanvasPadding();
            double yPad = cHeight * model.getCanvasPadding();

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

            Line2D line1 = new Line2D.Double(b1.x, b1.y, b2.x, b2.y);
            Line2D line2 = new Line2D.Double(b2.x, b2.y, b4.x, b4.y);
            Line2D line3 = new Line2D.Double(b3.x, b3.y, b4.x, b4.y);
            Line2D line4 = new Line2D.Double(b1.x, b1.y, b3.x, b3.y);

            g2d.draw(line1);
            g2d.draw(line2);
            g2d.draw(line3);
            g2d.draw(line4);

        }

    }

    public void drawFlowRangebox() {
        g2d.setStroke(new BasicStroke(1f));
        g2d.setColor(Color.GRAY);
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
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

    /**
     * Draw control points of Bezier curves and lines connecting control points
     * to start and end points.
     */
    public void drawControlPoints() {
        g2d.setStroke(new BasicStroke(1f));
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
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
                drawCircle(cpt.x, cpt.y, CR, Color.ORANGE, Color.GRAY);
            }
        }
    }

    /**
     * Draw straight line segments for a Bezier curve. Useful for debugging.
     */
    public void drawStraightLinesSegments() {
        setStrokeWidth(2f);
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            ArrayList<Point> points = flow.toStraightLineSegments(0.01);
            for (Point point : points) {
                drawCircle(point.x, point.y, CR, Color.pink, Color.white);
            }
        }
    }

    /**
     * Testing algorithm for reconstructing Bezier curves from straight line
     * segments.
     */
    public void drawRebuiltBezierCurve() {
        // FIXME
        double tol = 0.3;

        setStrokeWidth(1f);
        g2d.setColor(Color.RED);

        Iterator<Flow> flowIterator = model.flowIterator();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            ArrayList<Point> points = flow.toStraightLineSegments(0.01);
            ArrayList<Point2D.Double> points2D = new ArrayList<>();
            for (Point pt : points) {
                points2D.add(new Point2D.Double(pt.x, pt.y));
            }
            BezierPath rebuilt = Bezier.fitBezierPath(points2D, tol);
            PathIterator pathIterator = rebuilt.getPathIterator(null);

            GeneralPath generalPath = new GeneralPath();
            double[] coords = new double[6];
            while (!pathIterator.isDone()) {
                int id = pathIterator.currentSegment(coords);
                switch (id) {
                    case PathIterator.SEG_CLOSE:
                        generalPath.closePath();
                        break;
                    case PathIterator.SEG_LINETO:
                        generalPath.lineTo(xToPx(coords[0]), yToPx(coords[1]));
                        break;
                    case PathIterator.SEG_MOVETO:
                        generalPath.moveTo(xToPx(coords[0]), yToPx(coords[1]));
                        break;
                    case PathIterator.SEG_QUADTO:
                        generalPath.quadTo(xToPx(coords[0]), yToPx(coords[1]),
                                xToPx(coords[2]), yToPx(coords[3]));
                        break;
                    case PathIterator.SEG_CUBICTO:
                        generalPath.curveTo(xToPx(coords[0]), yToPx(coords[1]),
                                xToPx(coords[2]), yToPx(coords[3]),
                                xToPx(coords[4]), yToPx(coords[5]));
                        break;

                }
                pathIterator.next();
            }

            g2d.draw(generalPath);
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
}
