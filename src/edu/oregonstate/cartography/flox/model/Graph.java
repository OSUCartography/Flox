package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import org.jgrapht.DirectedGraph;
import org.jgrapht.graph.DirectedMultigraph;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Graph {
    private DirectedGraph<Point, Flow> graph = new DirectedMultigraph<>(Flow.class);
    
    public Graph() {
    }

    /**
     * Returns the number of flows.
     *
     * @return The number of flows.
     */
    public int getNbrFlows() {
        return graph.edgeSet().size();
    }

    /**
     * Returns the number of nodes in the graph. This is different from
     * getNbrFlows() * 2.
     *
     * @return The number of nodes.
     */
    public int getNbrNodes() {
        return graph.vertexSet().size();
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
        graph.addVertex(startPoint);
        graph.addVertex(endPoint);
        graph.addEdge(startPoint, endPoint, flow);
    }
    
    public void deleteFlow(Flow flow) {
        graph.removeEdge(flow);
    }

    public void deleteNode(Point node) {
        graph.removeVertex(node);
    }

    /**
     * Searches for a point in the graph with the specified coordinates
     *
     * @param target A point with the coordinates to search.
     * @return The point with coordinates x and y in the graph or the passed
     * point if no point with the same coordinates exist in the graph.
     */
    public Point findNodeInGraph(Point target) {
        Iterator<Point> iter = graph.vertexSet().iterator();
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
         graph = new DirectedMultigraph<>(Flow.class);
    }
    
    /**
     * Returns the bounding box of all flows, excluding the other geometry.
     *
     * @return
     */
    public Rectangle2D getFlowsBoundingBox() {
        int nFlows = graph.edgeSet().size();
        if (nFlows < 1) {
            return null;
        }
        Iterator<Flow> iter = graph.edgeSet().iterator();
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
        return graph.edgeSet().iterator();
    }

    /**
     * Returns all flows in the graph. The flows are not ordered.
     *
     * @return All flows.
     */
    public ArrayList<Flow> getFlows() {
        return new ArrayList<>(graph.edgeSet());
    }

    /**
     * Get an ordered list of all flows.
     *
     * @param increasing Increasing or decreasing list.
     * @return A list with all ordered flows.
     */
    public ArrayList<Flow> getOrderedFlows(boolean increasing) {
        ArrayList<Flow> flows = new ArrayList<>(graph.edgeSet());
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
        return graph.vertexSet().iterator();
    }

    /**
     * Returns all nodes in the graph.
     *
     * @return All nodes.
     */
    public ArrayList<Point> getNodes() {
        return new ArrayList<>(graph.vertexSet());
    }

    /**
     * Returns the maximum flow value.
     *
     * @return The maximum flow value.
     */
    public double getMaxFlowValue() {
        int nFlows = graph.edgeSet().size();
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
}
