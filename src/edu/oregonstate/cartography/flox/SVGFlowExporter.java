package edu.oregonstate.cartography.flox;

import com.vividsolutions.jts.geom.GeometryCollection;
import java.util.Collection;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

/**
 * Extension of SVGExporter to export Flows.
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class SVGFlowExporter extends SVGExporter {

    /**
     * The Flow objects to export.
     */
    private final Collection<BezierFlow> flows;
    
    public SVGFlowExporter(GeometryCollection collection, Collection<BezierFlow> flows, String authorName, String applicationName) {
        super(collection,  authorName, applicationName);
        this.flows = flows;
    }
    
    /**
     * Add the flows to a SVG document
     * @param svgRootElement The SVG root element.
     * @param document The SVG document.
     */
    @Override
    protected void append(Element svgRootElement, Document document) {

        Element g = (Element) document.createElementNS(SVGNAMESPACE, "g");
        setVectorStyle(g);
        svgRootElement.appendChild(g);
        
        for (BezierFlow flow : flows) {
            StringBuilder str = new StringBuilder();
            str.append("M");
            str.append(df.format(xToPagePx(flow.getStartPt().x)));
            str.append(" ");
            str.append(df.format(yToPagePx(flow.getStartPt().y)));

            str.append(" C");
            str.append(df.format(xToPagePx(flow.getcPt1().x)));
            str.append(" ");
            str.append(df.format(yToPagePx(flow.getcPt1().y)));
            str.append(", ");
            
            str.append(df.format(xToPagePx(flow.getcPt2().x)));
            str.append(" ");
            str.append(df.format(yToPagePx(flow.getcPt2().y)));
            str.append(", ");
            
            str.append(df.format(xToPagePx(flow.getEndPt().x)));
            str.append(", ");
            str.append(df.format(yToPagePx(flow.getEndPt().y)));

            Element pathElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
            pathElement.setAttribute("d", str.toString());
            g.appendChild(pathElement);
        }
    }
}
