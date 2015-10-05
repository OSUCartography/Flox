package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.flox.gui.Arrow;
import edu.oregonstate.cartography.flox.gui.FloxMapComponent;
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

        QuadraticBezierFlow qFlow = (QuadraticBezierFlow) flow;
        StringBuilder str = new StringBuilder();
        str.append("M");
        str.append(df.format(xToPagePx(flow.getStartPt().x)));
        str.append(" ");
        str.append(df.format(yToPagePx(flow.getStartPt().y)));

        str.append(" Q");
        str.append(df.format(xToPagePx(qFlow.getCtrlPt().x)));
        str.append(" ");
        str.append(df.format(yToPagePx(qFlow.getCtrlPt().y)));
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

    private QuadraticBezierFlow clipFlowByEndNode(QuadraticBezierFlow flow) {
        // Scale the node's radius + stroke/2 distance to world distance
        double nodeR = ((NODE_STROKE_WIDTH / 2) + getNodeRadius(flow.getEndPt())) / mapComponent.getScale();

        // Clip the flow by that distance.
        double t = flow.getIntersectionTWithCircleAroundEndPoint(nodeR);
        return flow.split(t)[0];
    }

    private QuadraticBezierFlow getClippedFlow(QuadraticBezierFlow flow) {
        double deCasteljauTol = model.getDeCasteljauTolerance();
        flow = flow.getClippedFlow(deCasteljauTol);
        return flow;
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

        double r = model.getFlowDistanceFromEndPointPixel() / mapComponent.getScale()
                * getLockedScaleFactor();

        Iterator<Flow> iterator = model.flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            double flowWidth = getFlowWidth(flow);

            QuadraticBezierFlow f = (QuadraticBezierFlow) flow;

            if (model.isDrawArrows()) {

                f = getClippedFlow(f);
                f = clipFlowByEndNode(f);
                f = f.split(f.getIntersectionTWithCircleAroundEndPoint(r))[0];

                // make the arrow
                Arrow arrow = new Arrow(f, model, flowWidth, mapComponent.getScale(),
                        mapComponent.getWest(), mapComponent.getNorth());

                // Get the flow SVG
                Element pathElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
                pathElement.setAttribute("d", flowToPath(arrow.getOutFlow()));
                pathElement.setAttribute("stroke-width", Double.toString(flowWidth));
                g.appendChild(pathElement);

                // get the arrow
                Element arrowPathElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
                arrowPathElement.setAttribute("d", arrowToPath(arrow));
                arrowPathElement.setAttribute("fill", "black");
                g.appendChild(arrowPathElement);

            } else {
                //FIXME clip the flow
                f = getClippedFlow(f);
                f = f.split(f.getIntersectionTWithCircleAroundEndPoint(r))[0];
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
