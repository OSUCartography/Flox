package edu.oregonstate.cartography.flox.gui;

import com.vividsolutions.jts.geom.GeometryCollection;
import edu.oregonstate.cartography.flox.model.BezierFlow;
import edu.oregonstate.cartography.flox.model.Flow;
import edu.oregonstate.cartography.flox.model.Layer;
import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.flox.model.Point;
import edu.oregonstate.cartography.flox.model.QuadraticBezierFlow;
import edu.oregonstate.cartography.flox.model.VectorSymbol;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.geom.Ellipse2D;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.Iterator;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class FloxMapComponent extends AbstractSimpleFeatureMapComponent {

    /**
     * Radius of circles for start and end points
     */
    private final double R = 10;

    /**
     * Radius of circles for control points
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
        g2d.setStroke(new BasicStroke(1));

        // draw background map
        int nbrLayers = model.getNbrLayers();
        for (int i = nbrLayers - 1; i >= 0; i--) {
            Layer layer = model.getLayer(i);
            GeometryCollection geometry = layer.getGeometryCollection();
            VectorSymbol symbol = layer.getVectorSymbol();
            Color fillColor = symbol.isFilled() ? layer.getVectorSymbol().getFillColor() : null;
            Color strokeColor = symbol.isStroked() ? layer.getVectorSymbol().getStrokeColor() : null;
            if (fillColor != null || strokeColor != null) {
                draw(geometry, g2d, fillColor, strokeColor);
            }
        }

        // draw flows and nodes
        g2d.setColor(Color.BLACK);
        drawFlows(g2d);
        drawNodes(g2d);
        drawControlPoints(g2d);

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

    private GeneralPath flowToPath(BezierFlow flow) {
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

    private GeneralPath flowToPath(QuadraticBezierFlow flow) {
        GeneralPath path = new GeneralPath();
        Point startPt = flow.getStartPt();
        path.moveTo(xToPx(startPt.x), yToPx(startPt.y));
        Point cPt = flow.getcPt();
        Point endPt = flow.getEndPt();
        path.quadTo(xToPx(cPt.x), yToPx(cPt.y), xToPx(endPt.x), yToPx(endPt.y));
        return path;
    }

    /**
     * Draw all pt lines to a Graphics2D context.
     *
     * @param g2d The graphics context.
     */
    private void drawFlows(Graphics2D g2d) {
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            GeneralPath path;
            if (flow instanceof BezierFlow) {
                path = flowToPath((BezierFlow) flow);
            } else {
                path = flowToPath((QuadraticBezierFlow) flow);
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
    private void drawNodes(Graphics2D g2d) {
        g2d.setStroke(new BasicStroke(2));
        Iterator<Point> iter = model.nodeIterator();
        while (iter.hasNext()) {
            Point pt = iter.next();
            drawCircle(g2d, xToPx(pt.x), yToPx(pt.y), R, Color.WHITE, Color.BLACK);
        }
    }

    /**
     * Draw control points of Bezier curves and lines connecting control points
     * to start and end points.
     * @param g2d 
     */
    private void drawControlPoints(Graphics2D g2d) {
        g2d.setStroke(new BasicStroke(1f));
        Iterator<Flow> iter = model.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            Point startPt = flow.getStartPt();
            Point endPt = flow.getEndPt();
            if (flow instanceof BezierFlow) {
                Point cpt1 = ((BezierFlow) flow).getcPt1();
                Point cpt2 = ((BezierFlow) flow).getcPt2();
                g2d.setColor(Color.GRAY);        
                Line2D line1 = new Line2D.Double(xToPx(startPt.x), yToPx(startPt.y), xToPx(cpt1.x), yToPx(cpt1.y));
                g2d.draw(line1);
                Line2D line2 = new Line2D.Double(xToPx(endPt.x), yToPx(endPt.y), xToPx(cpt2.x), yToPx(cpt2.y));
                g2d.draw(line2);
                drawCircle(g2d, xToPx(cpt1.x), yToPx(cpt1.y), CR, Color.ORANGE, Color.GRAY);
                drawCircle(g2d, xToPx(cpt2.x), yToPx(cpt2.y), CR, Color.ORANGE, Color.GRAY);
            } else {
                Point cpt = ((QuadraticBezierFlow) flow).getcPt();
                g2d.setColor(Color.GRAY);        
                Line2D line1 = new Line2D.Double(xToPx(startPt.x), yToPx(startPt.y), xToPx(cpt.x), yToPx(cpt.y));
                g2d.draw(line1);
                Line2D line2 = new Line2D.Double(xToPx(endPt.x), yToPx(endPt.y), xToPx(cpt.x), yToPx(cpt.y));
                g2d.draw(line2);
                drawCircle(g2d, xToPx(cpt.x), yToPx(cpt.y), CR, Color.ORANGE, Color.GRAY);
            }
        }
    }

    private static void drawCircle(Graphics2D g2d, double x, double y, double r,
            Color fill, Color stroke) {
        Ellipse2D circle = new Ellipse2D.Double(x - r, y - r, r * 2, r * 2);
        g2d.setColor(fill);
        g2d.fill(circle);
        g2d.setColor(stroke);
        g2d.draw(circle);
    }
}
