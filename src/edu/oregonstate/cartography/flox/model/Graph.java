package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import org.jgrapht.graph.DirectedMultigraph;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public final class Graph {

    private double minFlowValue;
    private double maxFlowValue;
    private double meanFlowValue;
    private double minFlowLength;
    private double maxFlowLength;
    private double minNodeValue;
    private double maxNodeValue;
    private double meanNodeValue;

    private final DirectedMultigraph<Point, Flow> graph = new DirectedMultigraph<>(Flow.class);
    

    public Graph() {
    }

    /**
     * Copy constructor.
     *
     * @param original
     */
    public Graph(Graph original) {
        Iterator<Flow> flowIterator = original.flowIterator();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            addFlow(flow);
        }
    }

    public void updateCachedValues() {
        int nFlows = graph.edgeSet().size();
        if (nFlows < 1) {
            minFlowValue = 0;
            maxFlowValue = 0;
        }
        Iterator<Flow> flowIterator = flowIterator();
        double flowSum = 0;
        int flowCounter = 0;
        minFlowLength = Double.MAX_VALUE;
        maxFlowLength = 0;
        minFlowValue = maxFlowValue = flowIterator.next().getValue();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next(); 
            double v = flow.getValue();
            if (v < minFlowValue) {
                minFlowValue = v;
            }
            if (v > maxFlowValue) {
                maxFlowValue = v;
            }
            flowSum += v;
            flowCounter++;
            double l = flow.getBaselineLength();
            if (l > maxFlowLength) {
                maxFlowLength = l;
            }
            if (l < minFlowLength) {
                minFlowLength = l;
            }
        }
        meanFlowValue = flowSum / flowCounter;

        int nNodes = graph.vertexSet().size();
        if (nNodes < 1) {
            minNodeValue = 0;
            maxNodeValue = 0;
        }
        Iterator<Point> nodeIterator = nodeIterator();
        double nodeSum = 0;
        int nodeCounter = 0;
        minNodeValue = maxNodeValue = nodeIterator.next().getValue();
        while (nodeIterator.hasNext()) {
            double v = nodeIterator.next().getValue();
            if (v < minNodeValue) {
                minNodeValue = v;
            }
            if (v > maxNodeValue) {
                maxNodeValue = v;
            }
            nodeSum += v;
            nodeCounter++;
        }
        meanNodeValue = nodeSum / nodeCounter;
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
        updateCachedValues();
    }

    /**
     * Add a node
     *
     * @param node The node to add
     */
    public void addNode(Point node) {
        Point newNode = findNodeInGraph(node);
        graph.addVertex(newNode);
        updateCachedValues();
    }
    
    void removeEdge(Flow flow) {
        graph.removeEdge(flow);
        updateCachedValues();
    }

    void removeVertex(Point node) {
        graph.removeVertex(node);
        updateCachedValues();
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

    public int getNbrFlows() {
        return graph.edgeSet().size();
    }

    public int getNbrNodes() {
        return graph.vertexSet().size();
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
     * Get an ordered list of all flows.
     *
     * @param increasing Increasing or decreasing list.
     * @return A list with all ordered flows.
     */
    public ArrayList<Flow> getOrderedFlows(boolean increasing) {
        ArrayList<Flow> flows = new ArrayList<>(graph.edgeSet());
        java.util.Collections.sort(flows, (Flow flow1, Flow flow2) -> {
            if (increasing) {
                return Double.compare(flow1.getValue(), flow2.getValue());
            } else {
                return Double.compare(flow2.getValue(), flow1.getValue());
            }
        });
        return flows;
    }

    /**
     * Returns a list of flows incident at a node. The flows are ordered
     * anti-clockwise. The origin of the polar coordinate system is the
     * horizontal x axis.
     *
     * @param node the point to search incoming flows for.
     * @return A list with the ordered flows.
     */
    public ArrayList<Flow> getAnticlockwiseOrderedIncomingFlows(Point node) {
        Collection<Flow> unsorted = graph.incomingEdgesOf(node);
        ArrayList<Flow> list = new ArrayList<>(unsorted);
        java.util.Collections.sort(list, (Flow f1, Flow f2) -> {
            double a1 = f1.getBaselineAzimuth();
            double a2 = f2.getBaselineAzimuth();
            if (a1 < a2) {
                return -1;
            }
            if (a1 > a2) {
                return 1;
            }
            return 0;
        });
        return list;
    }

    public ArrayList<Point> getOrderedNodes(boolean increasing) {
        ArrayList<Point> nodes = new ArrayList<>(graph.vertexSet());
        java.util.Collections.sort(nodes, (Point node1, Point node2) -> {
            if (increasing) {
                return Double.compare(node1.getValue(), node2.getValue());
            } else {
                return Double.compare(node2.getValue(), node1.getValue());
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
        return maxFlowValue;
    }

    /**
     * Returns the smallest flow value.
     *
     * @return The minimum flow value.
     */
    public double getMinFlowValue() {
        return minFlowValue;
    }

    /**
     * Returns the largest flow value.
     *
     * @return The maximum flow value.
     */
    public double getMaxNodeValue() {
        return maxNodeValue;
    }

    /**
     * Gets the average value of all nodes on the map.
     *
     * @return mean node value
     */
    public double getMeanNodeValue() {
       return meanNodeValue;
    }

    /**
     * Gets the average value of all flows on the map.
     *
     * @return
     */
    public double getMeanFlowValue() {
        return meanFlowValue;
    }

    /**
     * Get the length of longest flow baseline.
     *
     * @return the length of the longest flow baseline
     */
    public double getLongestFlowLength() {
        return maxFlowLength;
    }

    /**
     * Get the length of the shortest flow baseline.
     *
     * @return the shortest flow baseline.
     */
    public double getShortestFlowLength() {
        return minFlowLength;
    }

}
