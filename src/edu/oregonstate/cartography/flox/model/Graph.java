package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import org.jgrapht.graph.DirectedMultigraph;

/**
 * Graph of Flows (edges) and Points (nodes).
 *
 * The JAXB GraphSerializer converts a Graph to and from a SerializedGraph (and
 * ultimately to XML).
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public final class Graph {

    /**
     * A set of cached values to accelerate computations. The cached values need
     * to be updated whenever a node or flow value changes. Use
     * updateCachedValues() to update.
     */
    private double minFlowValue;
    private double maxFlowValue;
    private double meanFlowValue;
    private double minFlowLength;
    private double maxFlowLength;
    private double minNodeValue;
    private double maxNodeValue;
    private double meanNodeValue;

    /**
     * JGraphT graph
     */
    private final DirectedMultigraph<Point, Flow> graph = new DirectedMultigraph<>(Flow.class);

    public Graph() {
    }

    /**
     * Update cached values. Call this whenever a node or flow value changes.
     */
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
        minFlowValue = Double.MAX_VALUE;
        maxFlowValue = -Double.MAX_VALUE;
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
        minNodeValue = Double.MAX_VALUE;
        maxNodeValue = -Double.MAX_VALUE;
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
     * Add a flow. If vertices with identical x and y coordinate already exist
     * in the graph, the existing nodes are used and the passed flow is linked
     * to these nodes.
     *
     * @param flow The flow to add. Start and end nodes may be changed.
     */
    public void addFlow(Flow flow) {
        addFlowNoCacheUpdate(flow);
        updateCachedValues();
    }

    /**
     * Add a list of flows. This is more efficient than calling addFlow multiple
     * times.
     *
     * @param flows flows to add. Start and end nodes may be changed.
     */
    public void addFlows(Collection<Flow> flows) {
        for (Flow flow : flows) {
            addFlowNoCacheUpdate(flow);
        }
        updateCachedValues();
    }

    /**
     * Add a flow without updating the cached values.
     *
     * @param flow flow to add. Start and end nodes may be changed.
     */
    private void addFlowNoCacheUpdate(Flow flow) {
        assert (hasFlowWithID(flow.id) == false);

        Point startPoint = findNodeInGraph(flow.getStartPt());
        Point endPoint = findNodeInGraph(flow.getEndPt());
        flow.setStartPt(startPoint);
        flow.setEndPt(endPoint);
        graph.addVertex(startPoint);
        graph.addVertex(endPoint);
        graph.addEdge(startPoint, endPoint, flow);
    }

    /**
     * Add a node if no node with identical x and y coordinates already exists
     * in the graph. Hence, there is no guarantee that the passed node will be
     * added.
     *
     * @param node The node to add
     * @return The added node.
     */
    public Point addNode(Point node) {
        node = findNodeInGraph(node);
        graph.addVertex(node);
        updateCachedValues();
        return node;
    }

    /**
     * Remove one flow.
     *
     * @param flow flow to remove
     */
    void removeFlow(Flow flow) {
        graph.removeEdge(flow);
        updateCachedValues();
    }

    /**
     * Remove all flows
     */
    void removeAllFlows() {
        graph.removeAllEdges(graph.edgeSet());
        updateCachedValues();
    }

    /**
     * Remove on node
     *
     * @param node node to remove
     */
    void removeNode(Point node) {
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

    private boolean hasFlowWithID(long id) {
        Iterator<Flow> iter = graph.edgeSet().iterator();
        while (iter.hasNext()) {
            if (iter.next().id == id) {
                return true;
            }
        }
        return false;
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
     * @return null if no flows exist.
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
     * Returns a list with all flows. It is recommended to use flowIterator()
     * instead.
     *
     * @return list with flows
     */
    public ArrayList<Flow> getFlows() {
        return new ArrayList<>(graph.edgeSet());
    }

    /**
     * Returns the flow from the end to the start point of the passed flow, if
     * it exists.
     *
     * @param flow search for flow in opposite direction to this flow
     * @return flow with opposite direction or null
     */
    public Flow getOpposingFlow(Flow flow) {
        return graph.getEdge(flow.getEndPt(), flow.getStartPt());
    }

    /**
     * Returns a list of flows connected to a node. The flows are sorted
     * anti-clockwise by the orientation of the line connecting start and end
     * points. The origin of the polar coordinate system is the horizontal x
     * axis.
     *
     * @param node the node to search connected flows for.
     * @return a list with the ordered flows.
     */
    public ArrayList<Flow> getAnticlockwiseOrderedFlowsAtNode(Point node) {
        Collection<Flow> unsorted = graph.edgesOf(node);
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

    /**
     * Returns a set of flows connected to a node.
     *
     * @param node search for flows connected to this node
     * @return a collection of Flows connected to the passed node
     */
    public Collection<Flow> getFlowsForNode(Point node) {
        return graph.edgesOf(node);
    }

    /**
     * Returns the first flow that can be found connecting two nodes.
     *
     * Throws an exception if either of the two nodes is not in this graph.
     *
     * @param node1 search for flows connected to this node
     * @param node2 search for flows connected to this node
     * @return a flow connected to the passed two nodes or null
     */
    public Flow getFirstFlowBetweenNodes(Point node1, Point node2) {
        Collection<Flow> flows = graph.edgesOf(node1);
        for (Flow flow : flows) {
            if (flow.endPt == node2 || flow.startPt == node2) {
                return flow;
            }
        }
        return null;
    }

    /**
     * Returns all flows starting at node 1 and ending at node 2 and vice versa.
     *
     * Throws an exception if either of the two nodes is not in this graph.
     *
     * @param node1 search for flows connected to this node
     * @param node2 search for flows connected to this node
     * @return all flows between the two nodes. This set can be empty.
     */
    public Set<Flow> getAllFlowsBetweenNodes(Point node1, Point node2) {
        Set<Flow> set1 = graph.getAllEdges(node1, node2);
        Set<Flow> set2 = graph.getAllEdges(node2, node1);
        set1.addAll(set2);
        return set1;
    }

    public ArrayList<Point> getSortedNodes(boolean increasing) {
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

    public ArrayList<Flow> getOrderedFlows(boolean increasing) {
        ArrayList<Flow> flows = new ArrayList<>(graph.edgeSet());
        java.util.Collections.sort(flows, (Flow f1, Flow f2) -> {
            if (increasing) {
                return Double.compare(f1.getValue(), f2.getValue());
            } else {
                return Double.compare(f2.getValue(), f1.getValue());
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
     * @return the minNodeValue
     */
    public double getMinNodeValue() {
        return minNodeValue;
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
