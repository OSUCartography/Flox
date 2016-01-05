package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import org.jgrapht.graph.DirectedMultigraph;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public final class Graph extends DirectedMultigraph<Point, Flow> {

    public Graph() {
        super(Flow.class);
    }

    /**
     * Copy constructor.
     *
     * @param original
     */
    public Graph(Graph original) {
        super(Flow.class);
        
        Iterator<Flow> flowIterator = original.flowIterator();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            addFlow(flow);
        }
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
     * Add a node
     *
     * @param node The node to add
     */
    public void addNode(Point node) {
        Point newNode = findNodeInGraph(node);
        addVertex(newNode);
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

    public int getNbrFlows() {
        return edgeSet().size();
    }
    
    public int getNbrNodes() {
        return vertexSet().size();
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

    public ArrayList<Point> getOrderedNodes(boolean increasing) {
        ArrayList<Point> nodes = new ArrayList<>(vertexSet());
        java.util.Collections.sort(nodes, new Comparator<Point>() {
            @Override
            public int compare(Point node1, Point node2) {
                if (increasing) {
                    return Double.compare(node1.getValue(), node2.getValue());
                } else {
                    return Double.compare(node2.getValue(), node1.getValue());
                }
            }
        });
        return nodes;
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
        double max = iter.next().getValue();
        while (iter.hasNext()) {
            double v = iter.next().getValue();
            if (v > max) {
                max = v;
            }
        }
        return max;
    }

    public double getMinFlowValue() {
        int nFlows = edgeSet().size();
        if (nFlows < 1) {
            return 0;
        }
        Iterator<Flow> iter = flowIterator();
        double min = iter.next().getValue();
        while (iter.hasNext()) {
            double v = iter.next().getValue();
            if (v < min) {
                min = v;
            }
        }
        return min;
    }
    
    public double getMaxNodeValue() {
        int nNodes = vertexSet().size();
        if (nNodes < 1) {
            return 0;
        }
        Iterator<Point> iter = nodeIterator();
        double max = iter.next().getValue();
        while (iter.hasNext()) {
            double v = iter.next().getValue();
            if (v > max) {
                max = v;
            }
        }
        return max;
    }
    
     /**
     * Gets the average value of all nodes on the map.
     *
     * @return mean node value
     */
    public double getMeanNodeValue() {
        double sum = 0;
        int counter = 0;
        Iterator<Point> iterator = nodeIterator();
        while (iterator.hasNext()) {
            sum += iterator.next().getValue();
            counter++;
        }
        return sum / counter;
    }

    /**
     * Gets the average value of all flows on the map.
     *
     * @return
     */
    public double getMeanFlowValue() {

        Iterator<Flow> flowIterator = flowIterator();
        double sum = 0;
        int counter = 0;
        while (flowIterator.hasNext()) {
            sum += flowIterator.next().getValue();
            counter++;
        }
        return sum / counter;
    }
    
     /**
     * Get the length of longest flow baseline.
     *
     * @return the length of the longest flow baseline
     */
    public double getLongestFlowLength() {
        double maxLength = 0;
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            double l = iterator.next().getBaselineLength();
            if (l > maxLength) {
                maxLength = l;
            }
        }
        return maxLength;
    }

    /**
     * Get the length of the shortest flow baseline.
     *
     * @return the shortest flow baseline.
     */
    public double getShortestFlowLength() {
        double minLength = Double.MAX_VALUE;
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            double l = iterator.next().getBaselineLength();
            if (l < minLength) {
                minLength = l;
            }
        }
        return minLength;
    }

}
