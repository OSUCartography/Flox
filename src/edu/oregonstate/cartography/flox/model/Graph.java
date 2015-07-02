package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Set;
import org.jgrapht.graph.DirectedMultigraph;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Graph extends DirectedMultigraph<Point, Flow> {

    public Graph() {
        super(Flow.class);
    }

    /**
     * Add a flow.
     *
     * @param flow The flow to add.
     */
    public void addFlow(Flow flow) {
        Point startPoint = findNodeInGraph(flow.getStartPt());
        Point endPoint = findNodeInGraph(flow.getEndPt());
        flow.setStartPt(startPoint);
        flow.setEndPt(endPoint);
        addVertex(startPoint);
        addVertex(endPoint);
        addEdge(startPoint, endPoint, flow);
    }

    /**
     * Searches for a point in the graph with the specified coordinates
     *
     * @param target A point with the coordinates to search.
     * @return The point with coordinates x and y in the graph or the passed
     * point if no point with the same coordinates exist in the graph.
     */
    public Point findNodeInGraph(Point target) {
        Iterator<Point> iter = vertexSet().iterator();
        while (iter.hasNext()) {
            Point pt = iter.next();
            if (pt.x == target.x && pt.y == target.y) {
                return pt;
            }
        }
        return target;
    }

    /**
     * Remove all flows.
     */
    public void clearFlows() {
        removeAllEdges();
        removeAllVertices();
    }

    public void removeAllEdges() {
        LinkedList<Flow> copy = new LinkedList<>();
        for (Flow e : edgeSet()) {
            copy.add(e);
        }
        removeAllEdges(copy);
    }

    public void removeAllVertices() {
        LinkedList<Point> copy = new LinkedList<>();
        for (Point v : vertexSet()) {
            copy.add(v);
        }
        removeAllVertices(copy);
    }

    /**
     * Returns the bounding box of all flows, excluding the other geometry.
     *
     * @return
     */
    public Rectangle2D getFlowsBoundingBox() {
        int nFlows = edgeSet().size();
        if (nFlows < 1) {
            return null;
        }
        Iterator<Flow> iter = edgeSet().iterator();
        Rectangle2D bb = iter.next().getBoundingBox();
        while (iter.hasNext()) {
            bb = bb.createUnion(iter.next().getBoundingBox());
        }
        return bb;
    }

    /**
     * Returns an iterator for the flows.
     *
     * @return The iterator.
     */
    public Iterator<Flow> flowIterator() {
        return edgeSet().iterator();
    }

    /**
     * Returns all flows in the graph. The flows are not ordered.
     *
     * @return All flows.
     */
    public ArrayList<Flow> getFlows() {
        return new ArrayList<>(edgeSet());
    }

    /**
     * Get an ordered list of all flows.
     *
     * @param increasing Increasing or decreasing list.
     * @return A list with all ordered flows.
     */
    public ArrayList<Flow> getOrderedFlows(boolean increasing) {
        ArrayList<Flow> flows = new ArrayList<>(edgeSet());
        java.util.Collections.sort(flows, new Comparator<Flow>() {
            @Override
            public int compare(Flow flow1, Flow flow2) {
                if (increasing) {
                    return Double.compare(flow1.getValue(), flow2.getValue());
                } else {
                    return Double.compare(flow2.getValue(), flow1.getValue());
                }
            }
        });
        return flows;
    }

    /**
     * Returns an iterator for the nodes.
     *
     * @return The iterator.
     */
    public Iterator<Point> nodeIterator() {
        return vertexSet().iterator();
    }

    /**
     * Returns all nodes in the graph.
     *
     * @return All nodes.
     */
    public ArrayList<Point> getNodes() {
        return new ArrayList<>(vertexSet());
    }

    /**
     * Returns the maximum flow value.
     *
     * @return The maximum flow value.
     */
    public double getMaxFlowValue() {
        int nFlows = edgeSet().size();
        if (nFlows < 1) {
            return 0;
        }
        Iterator<Flow> iter = flowIterator();
        double max = iter.next().value;
        while (iter.hasNext()) {
            double v = iter.next().getValue();
            if (v > max) {
                max = v;
            }
        }
        return max;
    }
    
    public double getMaxNodeValue() {
        int nNodes = vertexSet().size();
        if (nNodes < 1) {
            return 0;
        }
        Iterator<Point> iter = nodeIterator();
        double max = iter.next().getValue();
        while(iter.hasNext()) {
            double v = iter.next().getValue();
            if (v > max) {
                max = v;
            }
        }
        return max;
    }
}
