package edu.oregonstate.cartography.flox.model;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * Conversion of flows and nodes to CSV text format.
 *
 * @author Daniel Stephen and Bernie Jenny
 */
public class CSV {

    public enum BezierType {
        quadratic, cubic
    };

    private CSV() {
    }

    /**
     * Write flows to string.
     *
     * @param iterator for flows
     * @param bezierType if quadratic, a single control point for a quadratic
     * Bezier curve is exporter, if cubic two control points for a cubic Bezier
     * curve are exported.
     * @return CSV string with start point, end point, control point(s), and
     * flow value.
     */
    public static String flowsToCSV(Iterator<Flow> iterator, BezierType bezierType) {
        StringBuilder str = new StringBuilder();

        while (iterator.hasNext()) {

            ArrayList<Flow> flows = new ArrayList<>();
            Flow f = iterator.next();

            if (f instanceof FlowPair) {
                FlowPair biFlow = (FlowPair) f;
                flows.add(biFlow.createFlow1());
                flows.add(biFlow.createFlow2());
            } else {
                flows.add(f);
            }

            for (Flow flow : flows) {
                str.append(flow.getStartPt().x);
                str.append(",");
                str.append(flow.getStartPt().y);
                str.append(",");

                str.append(flow.getEndPt().x);
                str.append(",");
                str.append(flow.getEndPt().y);
                str.append(",");

                switch (bezierType) {
                    case quadratic:
                        str.append(flow.cPtX());
                        str.append(",");
                        str.append(flow.cPtY());
                        str.append(",");
                        break;
                    case cubic:
                        double x0 = flow.getStartPt().x;
                        double y0 = flow.getStartPt().y;
                        double x1 = flow.cPtX();
                        double y1 = flow.cPtY();
                        double x2 = flow.getEndPt().x;
                        double y2 = flow.getEndPt().y;

                        double c1x = x0 + (x1 - x0) * 2 / 3;
                        double c1y = y0 + (y1 - y0) * 2 / 3;
                        double c2x = x2 + (x1 - x2) * 2 / 3;
                        double c2y = y2 + (y1 - y2) * 2 / 3;

                        str.append(c1x);
                        str.append(",");
                        str.append(c1y);
                        str.append(",");
                        str.append(c2x);
                        str.append(",");
                        str.append(c2y);
                        str.append(",");
                        break;
                }

                str.append(flow.getValue());
//                str.append(",");

//                str.append(flow.getStartPt().getValue());
//                str.append(",");
//
//                str.append(flow.getEndPt().getValue());
//                str.append(",");
//
//                if (flow.isLocked()) {
//                    str.append(1);
//                } else {
//                    str.append(0);
//                }
                str.append("\n");
            }

        }

        return str.toString();
    }

    /**
     * Write nodes to string.
     *
     * @param iterator iterator for nodes
     * @return CSV string with start coordinates and value.
     */
    public static String nodesToCSV(Iterator<Point> iterator) {
        StringBuilder str = new StringBuilder();

        while (iterator.hasNext()) {
            Point node = iterator.next();
            str.append(node.x);
            str.append(",");
            str.append(node.y);
            str.append(",");
            str.append(node.getValue());
            str.append("\n");
        }

        return str.toString();
    }

}
