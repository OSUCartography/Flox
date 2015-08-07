package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Envelope;
import edu.oregonstate.cartography.flox.gui.Arrow;
import edu.oregonstate.cartography.flox.gui.FloxMapComponent;
import edu.oregonstate.cartography.flox.gui.FloxRenderer;
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
    private final FloxMapComponent mapComponent;

    private final double NODE_STROKE_WIDTH = 2;

    public SVGFlowExporter(Model model, FloxMapComponent mapComponent) {
        super(model.getLayerGeometry(), "OSU Cartography Group", "Flox");
        this.model = model;
        Rectangle2D b = model.getFlowsBoundingBox();
        bb.init(b.getMinX(), b.getMaxX(), b.getMinY(), b.getMaxY());
        this.mapComponent = mapComponent;
    }

    private String flowToPath(Flow flow) {
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
        if (!model.isFlowWidthLocked()) {
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

        Element g = (Element) document.createElementNS(SVGNAMESPACE, "g");
        setVectorStyle(g);
        svgRootElement.appendChild(g);

        double r = model.getFlowDistanceFromEndPoint() / mapComponent.getScale()
                * getLockedScaleFactor();

        ArrayList<Flow> flows = model.getFlows();
        for (Flow flow : flows) {

            double flowWidth = getFlowWidth(flow);

            if (flow instanceof CubicBezierFlow) {
                Element pathElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
                pathElement.setAttribute("d", flowToPath(flow));
                pathElement.setAttribute("stroke-width", Double.toString(flowWidth));
                g.appendChild(pathElement);
            } else {
                
                QuadraticBezierFlow f = (QuadraticBezierFlow) flow;
                
                if (model.isDrawArrows()) {

                    f = clipFlowByEndNode(f);
                    f = getClippedFlow(f);
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
