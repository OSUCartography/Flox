package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Envelope;
import edu.oregonstate.cartography.simplefeature.SVGExporter;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
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

    public SVGFlowExporter(Model model) {
        super(model.getLayerGeometry(), "OSU Cartography Group", "Flox");
        this.model = model;
        Rectangle2D b = model.getFlowsBoundingBox();
        bb.init(b.getMinX(), b.getMaxX(), b.getMinY(), b.getMaxY());
    }

    private String toPath(Flow flow) {
        if (flow instanceof CubicBezierFlow) {
            CubicBezierFlow cFlow = (CubicBezierFlow) flow;
            StringBuilder str = new StringBuilder();
            str.append("M");
            str.append(df.format(xToPagePx(flow.getStartPt().x)));
            str.append(" ");
            str.append(df.format(yToPagePx(flow.getStartPt().y)));

            str.append(" C");
            str.append(df.format(xToPagePx(cFlow.getcPt1().x)));
            str.append(" ");
            str.append(df.format(yToPagePx(cFlow.getcPt1().y)));
            str.append(", ");

            str.append(df.format(xToPagePx(cFlow.getcPt2().x)));
            str.append(" ");
            str.append(df.format(yToPagePx(cFlow.getcPt2().y)));
            str.append(", ");

            str.append(df.format(xToPagePx(flow.getEndPt().x)));
            str.append(", ");
            str.append(df.format(yToPagePx(flow.getEndPt().y)));
            return str.toString();
        } else {
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
    }

    /**
     * Add the flows to a SVG document
     *
     * @param svgRootElement The SVG root element.
     * @param document The SVG document.
     */
    @Override
    protected void append(Element svgRootElement, Document document) {

        Element g = (Element) document.createElementNS(SVGNAMESPACE, "g");
        setVectorStyle(g);
        svgRootElement.appendChild(g);

        ArrayList<Flow> flows = model.getFlows();
        for (Flow flow : flows) {
            Element pathElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
            pathElement.setAttribute("d", toPath(flow));
            g.appendChild(pathElement);
        }
    }
}
