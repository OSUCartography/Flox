package edu.oregonstate.cartography.flox.gui;

import edu.oregonstate.cartography.flox.model.CubicBezierFlow;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
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
import java.util.Iterator;

/**
 * A renderer for the Flox data model
 * 
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class FloxRenderer extends SimpleFeatureRenderer {

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
        Rectangle2D bb = model.getBoundingBox();
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
        g2d.setColor(Color.BLACK);
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            GeneralPath path;
            if (flow instanceof CubicBezierFlow) {
                path = flowToGeneralPath((CubicBezierFlow) flow);
            } else {
                path = flowToGeneralPath((QuadraticBezierFlow) flow);
            }
            double strokeWidth = Math.abs(flow.getValue()) * model.getFlowWidthScale();
            g2d.setStroke(new BasicStroke((float) strokeWidth, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            g2d.draw(path);
        }
    }

    /**
     * Draw all nodes to a Graphics2D context.
     */
    public void drawNodes() {
        Iterator<Point> iter = model.nodeIterator();
        while (iter.hasNext()) {
            Point pt = iter.next();
            drawCircle(pt.x, pt.y, R, Color.WHITE, Color.BLACK);
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
     * Testing algorithm for reconstructing Bezier curves from straight line segments.
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
}
