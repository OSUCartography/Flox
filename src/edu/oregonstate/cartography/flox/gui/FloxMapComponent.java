package edu.oregonstate.cartography.flox.gui;

import com.vividsolutions.jts.geom.GeometryCollection;
import edu.oregonstate.cartography.flox.model.bezier.Bezier;
import edu.oregonstate.cartography.flox.model.bezier.BezierPath;
import edu.oregonstate.cartography.flox.model.CubicBezierFlow;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Layer;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import edu.oregonstate.cartography.flox.model.VectorSymbol;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import edu.oregonstate.cartography.simplefeature.SimpleFeatureRenderer;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.geom.Ellipse2D;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
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
     * Radius of circles for start and end points (in pixels)
     */
    private final double R = 10;

    /**
     * Radius of circles for control points (in pixels)
     */
    private final double CR = 3;

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
        SimpleFeatureRenderer renderer = new SimpleFeatureRenderer(g2d, west, north, scale);
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
        drawFlows(renderer);
        drawNodes(renderer);
        if (model.isDrawControlPoints()) {
            drawControlPoints(renderer);
        }
        if (model.isDrawLineSegments()) {
            drawStraightLinesSegments(renderer);
        }

        if (model.isDrawReconstructedBezier()) {
            drawRebuiltBezierCurve(renderer);
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

    /**
     * Constructs a GeneralPath object for drawing from a flow
     *
     * @param flow The flow to convert.
     * @return A GeneralPath for drawing.
     */
    private GeneralPath flowToGeneralPath(CubicBezierFlow flow, SimpleFeatureRenderer renderer) {
        GeneralPath path = new GeneralPath();
        Point startPt = flow.getStartPt();
        path.moveTo(renderer.xToPx(startPt.x), renderer.yToPx(startPt.y));
        Point cPt1 = flow.getcPt1();
        Point cPt2 = flow.getcPt2();
        Point endPt = flow.getEndPt();
        path.curveTo(renderer.xToPx(cPt1.x), renderer.yToPx(cPt1.y),
                renderer.xToPx(cPt2.x), renderer.yToPx(cPt2.y),
                renderer.xToPx(endPt.x), renderer.yToPx(endPt.y));
        return path;
    }

    /**
     * Constructs a GeneralPath object for drawing from a flow
     *
     * @param flow The flow to convert.
     * @return A GeneralPath for drawing.
     */
    private GeneralPath flowToGeneralPath(QuadraticBezierFlow flow, SimpleFeatureRenderer renderer) {
        GeneralPath path = new GeneralPath();
        Point startPt = flow.getStartPt();
        path.moveTo(renderer.xToPx(startPt.x), renderer.yToPx(startPt.y));
        Point cPt = flow.getcPt();
        Point endPt = flow.getEndPt();
        path.quadTo(renderer.xToPx(cPt.x), renderer.yToPx(cPt.y),
                renderer.xToPx(endPt.x), renderer.yToPx(endPt.y));
        return path;
    }

    /**
     * Draw all pt lines to a Graphics2D context.
     *
     * @param g2d The graphics context.
     */
    private void drawFlows(SimpleFeatureRenderer renderer) {
        Graphics2D g2d = renderer.getGraphics2D();
        g2d.setColor(Color.BLACK);
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            GeneralPath path;
            if (flow instanceof CubicBezierFlow) {
                path = flowToGeneralPath((CubicBezierFlow) flow, renderer);
            } else {
                path = flowToGeneralPath((QuadraticBezierFlow) flow, renderer);
            }
            double strokeWidth = Math.abs(flow.getValue()) * model.getFlowWidthScale();
            g2d.setStroke(new BasicStroke((float) strokeWidth, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER));
            g2d.draw(path);
        }
    }

    /**
     * Draw all nodes to a Graphics2D context.
     *
     * @param g2d The graphics context.
     */
    private void drawNodes(SimpleFeatureRenderer renderer) {
        Iterator<Point> iter = model.nodeIterator();
        while (iter.hasNext()) {
            Point pt = iter.next();
            renderer.drawCircle(pt.x, pt.y, R, Color.WHITE, Color.BLACK);
        }
    }

    /**
     * Draw control points of Bezier curves and lines connecting control points
     * to start and end points.
     *
     * @param g2d
     */
    private void drawControlPoints(SimpleFeatureRenderer renderer) {
        Graphics2D g2d = renderer.getGraphics2D();
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
                Line2D line1 = new Line2D.Double(renderer.xToPx(startPt.x),
                        renderer.yToPx(startPt.y), renderer.xToPx(cpt1.x), renderer.yToPx(cpt1.y));
                g2d.draw(line1);
                Line2D line2 = new Line2D.Double(renderer.xToPx(endPt.x),
                        renderer.yToPx(endPt.y), renderer.xToPx(cpt2.x), renderer.yToPx(cpt2.y));
                g2d.draw(line2);
                renderer.drawCircle(cpt1.x, cpt1.y, CR, Color.ORANGE, Color.GRAY);
                renderer.drawCircle(cpt2.x, cpt2.y, CR, Color.ORANGE, Color.GRAY);
            } else {
                Point cpt = ((QuadraticBezierFlow) flow).getcPt();
                g2d.setColor(Color.GRAY);
                Line2D line1 = new Line2D.Double(renderer.xToPx(startPt.x),
                        renderer.yToPx(startPt.y), renderer.xToPx(cpt.x), renderer.yToPx(cpt.y));
                g2d.draw(line1);
                Line2D line2 = new Line2D.Double(renderer.xToPx(endPt.x),
                        renderer.yToPx(endPt.y), renderer.xToPx(cpt.x), renderer.yToPx(cpt.y));
                g2d.draw(line2);
                renderer.drawCircle(cpt.x, cpt.y, CR, Color.ORANGE, Color.GRAY);
            }
        }
    }

    /**
     * Draw straight line segments for a Bezier curve. Useful for debugging.
     *
     * @param g2d
     */
    private void drawStraightLinesSegments(SimpleFeatureRenderer renderer) {
        renderer.setStrokeWidth(2f);
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            ArrayList<Point> points = flow.toStraightLineSegments(0.01);
            for (Point point : points) {
                renderer.drawCircle(point.x, point.y, CR, Color.pink, Color.white);
            }
        }
    }

    private void drawRebuiltBezierCurve(SimpleFeatureRenderer renderer) {
        // FIXME
        double tol = 0.3;

        renderer.setStrokeWidth(1f);
        renderer.setColor(Color.RED);

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
            int counter = 0;
            while (!pathIterator.isDone()) {
                int id = pathIterator.currentSegment(coords);
                switch (id) {
                    case PathIterator.SEG_CLOSE:
                        generalPath.closePath();
                        break;
                    case PathIterator.SEG_LINETO:
                        generalPath.lineTo(renderer.xToPx(coords[0]), renderer.yToPx(coords[1]));
                        break;
                    case PathIterator.SEG_MOVETO:
                        generalPath.moveTo(renderer.xToPx(coords[0]), renderer.yToPx(coords[1]));
                        break;
                    /*case PathIterator.SEG_QUADTO:
                     generalPath.quadTo(coords[0], coords[1],
                     coords[2], coords[3]);
                     break;
                     */
                    case PathIterator.SEG_CUBICTO:
                        generalPath.curveTo(renderer.xToPx(coords[0]), renderer.yToPx(coords[1]),
                                renderer.xToPx(coords[2]), renderer.yToPx(coords[3]),
                                renderer.xToPx(coords[4]), renderer.yToPx(coords[5]));
                        break;

                }
                System.out.println(id);
                pathIterator.next();
                counter++;
            }

            if (counter != 2) {
                System.out.println("total: " + counter);
            }
            System.out.println();

            renderer.getGraphics2D().draw(generalPath);
        }
    }
}
