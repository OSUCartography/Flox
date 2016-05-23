package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.flox.gui.FloxMapComponent;
import static edu.oregonstate.cartography.flox.gui.FloxRenderer.NODE_STROKE_WIDTH;
import edu.oregonstate.cartography.simplefeature.SVGExporter;
import java.awt.Color;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

/**
 * Extension of SVGExporter to export Flows.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class SVGFlowExporter extends SVGExporter {

    private final Model model;
    private final FloxMapComponent mapComponent;

    private final double NODE_STROKE_WIDTH = 2;

    public SVGFlowExporter(Model model, FloxMapComponent mapComponent) {
        super(model.getLayerGeometry(), "OSU Cartography Group", "Flox");
        this.model = model;
        Rectangle2D b = mapComponent.getVisibleArea();
        bb.init(b.getMinX(), b.getMaxX(), b.getMinY(), b.getMaxY());
        this.mapComponent = mapComponent;
    }

    private String flowToPath(Flow flow) {

        StringBuilder str = new StringBuilder();
        str.append("M");
        str.append(df.format(xToPagePx(flow.getStartPt().x)));
        str.append(" ");
        str.append(df.format(yToPagePx(flow.getStartPt().y)));

        str.append(" Q");
        str.append(df.format(xToPagePx(flow.getCtrlPt().x)));
        str.append(" ");
        str.append(df.format(yToPagePx(flow.getCtrlPt().y)));
        str.append(", ");
        str.append(df.format(xToPagePx(flow.getEndPt().x)));
        str.append(", ");
        str.append(df.format(yToPagePx(flow.getEndPt().y)));
        return str.toString();
    }

    private String arrowToPath(Arrow arrow) {

        StringBuilder str = new StringBuilder();

        // Start at the base point
        str.append("M");
        str.append(df.format(xToPagePx(arrow.getBasePt().x)));
        str.append(" ");
        str.append(df.format(yToPagePx(arrow.getBasePt().y)));

        // Line to the first corner
        str.append(" L");
        str.append(df.format(xToPagePx(arrow.getCorner1Pt().x)));
        str.append(" ");
        str.append(df.format(yToPagePx(arrow.getCorner1Pt().y)));

        // Quad to the tip
        str.append(" Q");
        str.append(df.format(xToPagePx(arrow.getCorner1cPt().x)));
        str.append(" ");
        str.append(df.format(yToPagePx(arrow.getCorner1cPt().y)));
        str.append(", ");
        str.append(df.format(xToPagePx(arrow.getTipPt().x)));
        str.append(", ");
        str.append(df.format(yToPagePx(arrow.getTipPt().y)));

        // Quad to the other corner
        str.append(" Q");
        str.append(df.format(xToPagePx(arrow.getCorner2cPt().x)));
        str.append(" ");
        str.append(df.format(yToPagePx(arrow.getCorner2cPt().y)));
        str.append(", ");
        str.append(df.format(xToPagePx(arrow.getCorner2Pt().x)));
        str.append(", ");
        str.append(df.format(yToPagePx(arrow.getCorner2Pt().y)));

        // Line back to the base
        str.append(" L");
        str.append(df.format(xToPagePx(arrow.getBasePt().x)));
        str.append(" ");
        str.append(df.format(yToPagePx(arrow.getBasePt().y)));

        return str.toString();
    }

    private double getLockedScaleFactor() {
        if (!model.isScaleLocked()) {
            return 1;
        } else {
            // compare the locked scale to the current scale
            double lockedMapScale = model.getLockedMapScale();
            return mapComponent.getScale() / lockedMapScale;
        }
    }

    private double getFlowWidth(Flow flow) {
        return Math.abs(flow.getValue()) * model.getFlowWidthScaleFactor()
                * getLockedScaleFactor();

    }

    private double getNodeRadius(Point node) {
        double area = Math.abs(node.getValue()
                * model.getNodeSizeScaleFactor());

        return (Math.sqrt(area / Math.PI)) * getLockedScaleFactor();
    }

    /**
     * Computes clipping radius for end node. Takes size of node and distance to
     * end node into account.
     *
     * @param endNode The end node of the flow.
     * @return Clipping radius in world coordinates.
     */
    private double endClipRadius(Point endNode) {
        double s = 1000 * MM2PX / scale;
        // distance between end of flows and their end points
        double gapDistanceToEndNodes = model.getFlowDistanceFromEndPointPixel() / s
                * getLockedScaleFactor();
        // Compute the radius of the end node (add stroke width / 2 to radius)
        double endNodeRadius = (NODE_STROKE_WIDTH / 2 + getNodeRadius(endNode)) / s;
        return gapDistanceToEndNodes + endNodeRadius;
    }
    
    private double startClipRadius(Point startNode) {
        double s = 1000 * MM2PX / scale;
        // distance between end of flows and their end points
        double gapDistanceToEndNodes = model.getFlowDistanceFromStartPointPixel() / s
                * getLockedScaleFactor();
        // Compute the radius of the end node (add stroke width / 2 to radius)
        double endNodeRadius = (NODE_STROKE_WIDTH / 2 + getNodeRadius(startNode)) / s;
        return gapDistanceToEndNodes + endNodeRadius;
    }
   
    
    
    private Flow getClippedFlow(Flow flow, double startClipRadius, double endClipRadius) {
        double deCasteljauTol = model.getDeCasteljauTolerance();
        return flow.getClippedFlow(startClipRadius, endClipRadius, deCasteljauTol);
    }

    /**
     * Add the flows to a SVG document
     *
     * @param svgRootElement The SVG root element.
     * @param document The SVG document.
     */
    @Override
    protected void append(Element svgRootElement, Document document) {

        // export map layers
        int nbrLayers = model.getNbrLayers();
        for (int i = nbrLayers - 1; i >= 0; i--) {
            Layer layer = model.getLayer(i);
            Element g = appendGeometryCollection(layer.getGeometryCollection(),
                    svgRootElement, document);
            g.setAttribute("id", layer.getName());
            VectorSymbol symbol = layer.getVectorSymbol();
            Color fillColor = symbol.isFilled()
                    ? layer.getVectorSymbol().getFillColor() : null;
            Color strokeColor = symbol.isStroked()
                    ? layer.getVectorSymbol().getStrokeColor() : null;
            setVectorStyle(g, strokeColor, 1, fillColor);
        }

        // export flows and nodes
        Element g = (Element) document.createElementNS(SVGNAMESPACE, "g");
        setVectorStyle(g, model.getFlowColor(), 1, null);
        svgRootElement.appendChild(g);

        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            double flowWidth = getFlowWidth(flow);

            if (model.isDrawArrowheads()) {
                // Compute radius of clipping circle around end point.
                // Clip the flow with the clipping area and a circle around the end node
                double rs = model.getFlowDistanceFromStartPointPixel() > 0 ? startClipRadius(flow.getStartPt()) : 0;
                flow = getClippedFlow(flow, rs, endClipRadius(flow.getEndPt()));
                
                // Create an arrowhead
                flow.configureArrow(model, flowWidth, model.endClipRadius(flow.getEndPt(), mapComponent.getScale()));
                
                // get the arrow
                Arrow arrow = flow.getEndArrow();

                // Get the flow SVG
                Element pathElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
                pathElement.setAttribute("d", flowToPath(arrow.getOutFlow()));
                pathElement.setAttribute("stroke-width", Double.toString(flowWidth));
                g.appendChild(pathElement);

                // get the arrow
                Element arrowPathElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
                arrowPathElement.setAttribute("d", arrowToPath(arrow));
                arrowPathElement.setAttribute("fill", "black");
                arrowPathElement.setAttribute("stroke-width", Double.toString(0));
                g.appendChild(arrowPathElement);

            } else {
                // Clip the flow with the clipping area
                double rs = model.getFlowDistanceFromStartPointPixel() > 0 ? startClipRadius(flow.getStartPt()) : 0;
                double re = model.getFlowDistanceFromEndPointPixel() > 0 ? endClipRadius(flow.getEndPt()) : 0;
                
                flow = getClippedFlow(flow, rs, re);
                
                Element pathElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
                pathElement.setAttribute("d", flowToPath(flow));
                pathElement.setAttribute("stroke-width", Double.toString(flowWidth));
                g.appendChild(pathElement);
            }
        }

        ArrayList<Point> nodes = model.getOrderedNodes(false);
        for (Point node : nodes) {
            Element circleElement = (Element) document.createElementNS(SVGNAMESPACE, "circle");
            circleElement.setAttribute("cx", df.format(xToPagePx(node.x)));
            circleElement.setAttribute("cy", df.format(yToPagePx(node.y)));
            circleElement.setAttribute("r", Double.toString(getNodeRadius(node)));
            circleElement.setAttribute("stroke-width", Double.toString(NODE_STROKE_WIDTH));
            circleElement.setAttribute("fill", "white");
            g.appendChild(circleElement);
        }
    }
}
