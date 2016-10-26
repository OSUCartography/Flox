package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.simplefeature.SVGExporter;
import java.awt.Color;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

/**
 * Extension of SVGExporter to export flows.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class SVGFlowExporter extends SVGExporter {

    private final Model model;

    public SVGFlowExporter(Model model) {
        super(model.getCanvas(), "", "Flox");
        this.model = model;
        scale = 1d / model.getReferenceMapScale();
        
        Rectangle2D canvas = model.getCanvas();
        canvasWidth = distToSVGCanvas(canvas.getWidth());
        canvasHeight = distToSVGCanvas(canvas.getHeight());
    }

    private String flowToPath(Flow flow) {
        StringBuilder str = new StringBuilder();
        str.append("M");
        str.append(df.format(xToSVGCanvas(flow.getStartPt().x)));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(flow.getStartPt().y)));

        str.append(" Q");
        str.append(df.format(xToSVGCanvas(flow.getCtrlPt().x)));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(flow.getCtrlPt().y)));
        str.append(", ");
        str.append(df.format(xToSVGCanvas(flow.getEndPt().x)));
        str.append(", ");
        str.append(df.format(yToSVGCanvas(flow.getEndPt().y)));
        return str.toString();
    }

    private String arrowToPath(Arrow arrow) {
        StringBuilder str = new StringBuilder();

        // Start at the base point
        str.append("M ");
        str.append(df.format(xToSVGCanvas(arrow.getBasePt().x)));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(arrow.getBasePt().y)));

        // Line to the first corner
        str.append(" L ");
        str.append(df.format(xToSVGCanvas(arrow.getCorner1Pt().x)));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(arrow.getCorner1Pt().y)));

        // Quad to the tip
        str.append(" Q ");
        str.append(df.format(xToSVGCanvas(arrow.getCorner1cPt().x)));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(arrow.getCorner1cPt().y)));
        str.append(" ");
        str.append(df.format(xToSVGCanvas(arrow.getTipPt().x)));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(arrow.getTipPt().y)));

        // Quad to the other corner
        str.append("  Q ");
        str.append(df.format(xToSVGCanvas(arrow.getCorner2cPt().x)));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(arrow.getCorner2cPt().y)));
        str.append(" ");
        str.append(df.format(xToSVGCanvas(arrow.getCorner2Pt().x)));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(arrow.getCorner2Pt().y)));

        // Line back to the base
        str.append(" L ");
        str.append(df.format(xToSVGCanvas(arrow.getBasePt().x)));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(arrow.getBasePt().y)));

        return str.toString();
    }
    
    private Element flowToDOMElement(Flow flow, Document document) {
        flow = model.clipFlow(flow, true, false);
            Element flowElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
            flowElement.setAttribute("id", Double.toString(flow.getValue()));
            flowElement.setAttribute("d", flowToPath(flow));
            double flowWidth = model.getFlowWidthPx(flow);
            setVectorStyle(flowElement, model.getFlowColor(flow), flowWidth, null);
            return flowElement;
    }

    /**
     * Add flows, nodes and map layers to the SVG document
     *
     * @param svgRootElement The SVG root element.
     * @param document The SVG document.
     */
    @Override
    protected void append(Element svgRootElement, Document document) {

        // background rectangle
        Element rect = (Element) document.createElementNS(SVGNAMESPACE, "rect");
        rect.setAttribute("x", "0");
        rect.setAttribute("y", "0");
        rect.setAttribute("width", df.format(canvasWidth));
        rect.setAttribute("height", df.format(canvasHeight));
        rect.setAttribute("id", "Canvas");
        setVectorStyle(rect, null, 0, model.getBackgroundColor());
        svgRootElement.appendChild(rect);

        // map layers
        int nbrLayers = model.getNbrLayers();
        for (int i = nbrLayers - 1; i >= 0; i--) {
            Layer layer = model.getLayer(i);
            Element g = appendGeometryCollection(layer.getGeometryCollection(),
                    svgRootElement, svgRootElement, document);
            g.setAttribute("id", layer.getName());
            VectorSymbol symbol = layer.getVectorSymbol();
            Color fillColor = symbol.isFilled()
                    ? layer.getVectorSymbol().getFillColor() : null;
            Color strokeColor = symbol.isStroked()
                    ? layer.getVectorSymbol().getStrokeColor() : null;
            setVectorStyle(g, strokeColor, 1, fillColor);
        }

        // flows
        Element flowsGroup = (Element) document.createElementNS(SVGNAMESPACE, "g");
        flowsGroup.setAttribute("id", "Flows");
        svgRootElement.appendChild(flowsGroup);
        Iterator<Flow> iterator = model.sortedFlowIteratorForDrawing(false);
        while (iterator.hasNext()) {
            Flow flow = iterator.next();

            // arrowhead
            if (model.isDrawArrowheads()) {
                Element arrowElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
                Arrow arrow = flow.getArrow(model);
                arrowElement.setAttribute("d", arrowToPath(arrow));
                setVectorStyle(arrowElement, null, 0, model.getFlowColor(flow));
                flowsGroup.appendChild(arrowElement);
            }

            // flow line
            flowsGroup.appendChild(flowToDOMElement(flow, document));
        }

        // nodes
        Element nodesGroup = (Element) document.createElementNS(SVGNAMESPACE, "g");
        svgRootElement.appendChild(nodesGroup);
        nodesGroup.setAttribute("id", "Nodes");
        ArrayList<Point> nodes = model.getSortedNodes(false);
        for (Point node : nodes) {
            Element circleElement = (Element) document.createElementNS(SVGNAMESPACE, "circle");
            circleElement.setAttribute("cx", df.format(xToSVGCanvas(node.x)));
            circleElement.setAttribute("cy", df.format(yToSVGCanvas(node.y)));
            circleElement.setAttribute("r", df.format(model.getNodeRadiusPx(node)));
            double strokeWidth = model.getNodeStrokeWidthPx();
            setVectorStyle(circleElement, model.getNodeStrokeColor(),
                    strokeWidth, model.getNodeFillColor());
            nodesGroup.appendChild(circleElement);
        }
    }
}
