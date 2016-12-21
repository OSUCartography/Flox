package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import java.util.TreeSet;
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

    /**
     * show opposing flows between the same start node and end node as parallel
     * lines.
     */
    private boolean bidirectionalFlowsParallel = true;

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
            double v;
            if (flow instanceof FlowPair) {
                v = ((FlowPair) flow).getValue1();
                minFlowValue = Math.min(minFlowValue, v);
                maxFlowValue = Math.max(maxFlowValue, v);
                flowSum += v;
                ++flowCounter;
                v = ((FlowPair) flow).getValue2();
            } else {
                v = flow.getValue();
            }
            minFlowValue = Math.min(minFlowValue, v);
            maxFlowValue = Math.max(maxFlowValue, v);
            flowSum += v;
            ++flowCounter;

            // FIXME should compare with squared length
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
    public void addFlows(Collection<? extends Flow> flows) {
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

        // replace start and end node of flow if they are already in the graph
        Point startPoint = findNodeInGraph(flow.getStartPt());
        Point endPoint = findNodeInGraph(flow.getEndPt());
        flow.setStartPt(startPoint);
        flow.setEndPt(endPoint);

        // add the nodes and the flow
        graph.addVertex(startPoint);
        graph.addVertex(endPoint);

        // update bidirectional flows
        if (bidirectionalFlowsParallel) {
            if (flow instanceof FlowPair == false && flow.isLocked() == false) {
                Flow oppositeFlow = getOpposingFlow(flow);
                if (oppositeFlow != null
                        && oppositeFlow instanceof FlowPair == false
                        && !oppositeFlow.isLocked()) {
                    removeFlow(oppositeFlow);
                    flow = new FlowPair(flow, oppositeFlow);
                }
            }
        }

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
     * Remove all flows contained in a collection. This is more efficient than
     * calling addFlow multiple times.
     *
     * @param flows flows to add. Start and end nodes may be changed.
     */
    public void removeFlows(Collection<? extends Flow> flows) {
        for (Flow flow : flows) {
            graph.removeEdge(flow);
        }
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

    // FIXME not used. This dynamic search should replace Flow.idCounter
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
     * Returns an iterator for the flows sorted by a provided Comparator
     *
     * @param comparator the comparator
     * @return iterator for sorted flows
     */
    public Iterator<Flow> flowIterator(Comparator<Flow> comparator) {
        TreeSet<Flow> treeSet = new TreeSet<>(comparator);
        treeSet.addAll(graph.edgeSet());
        return treeSet.iterator();
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
            double a1 = f1.getBaselineOrientation();
            double a2 = f2.getBaselineOrientation();
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
            if (flow.getEndPt() == node2 || flow.getStartPt() == node2) {
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
        Set<Flow> set12 = graph.getAllEdges(node1, node2);
        Set<Flow> set21 = graph.getAllEdges(node2, node1);
        set12.addAll(set21);
        return set12;
    }

    /**
     * Returns the net sum of flow values for all flows between node 1 and node
     * 2. The net sum is the sum of all flow values moving from node 1 to node 2
     * minus the sum of all flow values moving from node 2 to node 1. The
     * returned value can be smaller than 0.
     *
     * @param node1 start node
     * @param node2 end node
     * @return The sum of all flow values moving from node 1 to node 2 minus the
     * sum of all flow values moving from node 2 to node 1. The returned value
     * can be smaller than 0.
     */
    public double netSum(Point node1, Point node2) {
        double sum12 = 0;
        double sum21 = 0;
        Set<Flow> flows12 = graph.getAllEdges(node1, node2);
        for (Flow flow : flows12) {
            if (flow instanceof FlowPair) {
                FlowPair flowPair = (FlowPair) flow;
                sum12 += flowPair.getValue1();
                sum21 += flowPair.getValue2();
            } else {
                sum12 += flow.getValue();
            }
        }

        Set<Flow> flows21 = graph.getAllEdges(node2, node1);
        for (Flow flow : flows21) {
            if (flow instanceof FlowPair) {
                FlowPair flowPair = (FlowPair) flow;
                sum21 += flowPair.getValue1();
                sum12 += flowPair.getValue2();
            } else {
                sum21 += flow.getValue();
            }
        }

        return sum12 - sum21;
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

    /**
     * Returns a new array with all flows sorted from largest to smallest (i.e.
     * the reverse of the natural order). Each FlowPair is converted to two
     * regular Flows.
     *
     * @param model data model
     * @param increasing increasing or decreasing sort order
     * @return a sorted list
     */
    public ArrayList<Flow> getSortedFlowsForDrawing(Model model, boolean increasing) {
        Set<Flow> flows = graph.edgeSet();
        ArrayList<Flow> sortedFlows = new ArrayList<>(flows.size());
        for (Flow flow : flows) {
            if (flow instanceof FlowPair) {
                FlowPair flowPair = (FlowPair) flow;
                sortedFlows.add(flowPair.createOffsetFlow1(model, Flow.FlowOffsettingQuality.HIGH));
                sortedFlows.add(flowPair.createOffsetFlow2(model, Flow.FlowOffsettingQuality.HIGH));
            } else {
                sortedFlows.add(flow);
            }
        }
        sortedFlows.sort(increasing ? null : Collections.reverseOrder());
        return sortedFlows;
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

    /**
     * Returns whether opposing flows between the same start point and end point
     * are to be shown as two parallel flows.
     *
     * @return if true, opposing flows are shown as two parallel flows.
     */
    public boolean isBidirectionalFlowsParallel() {
        return bidirectionalFlowsParallel;
    }

    /**
     * Set whether opposing flows between the same start point and end point are
     * to be shown as two parallel flows.
     *
     * @param bidirectionalFlowsParallel if true, opposing flows are shown as
     * two parallel flows.
     * @param model data model
     */
    public void setBidirectionalFlowsParallel(boolean bidirectionalFlowsParallel,
            Model model) {
        if (this.bidirectionalFlowsParallel == bidirectionalFlowsParallel) {
            return;
        }
        this.bidirectionalFlowsParallel = bidirectionalFlowsParallel;

        if (bidirectionalFlowsParallel) {
            toBidirectionalFlows();
        } else {
            toUnidirectionalFlows(model);
        }
    }

    /**
     * Replaces pairs of opposing flows between the same two nodes with a single
     * FlowPair.
     */
    public void toBidirectionalFlows() {
        ArrayList<FlowPair> flowsToAdd = new ArrayList<>();
        HashSet<Flow> flowsToRemove = new HashSet<>();
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow1 = iterator.next();
            if (flow1 instanceof FlowPair) {
                continue;
            }

            Flow flow2 = getOpposingFlow(flow1);
            if (flow2 != null) {
                if (flow2 instanceof FlowPair) {
                    continue;
                }
                if (flow1.isLocked() || flow2.isLocked()) {
                    continue;
                }

                if (flowsToRemove.contains(flow1) == false) {
                    flowsToAdd.add(new FlowPair(flow1, flow2));
                    flowsToRemove.add(flow1);
                    flowsToRemove.add(flow2);
                }
            }
        }

        removeFlows(flowsToRemove);
        addFlows(flowsToAdd);
    }

    /**
     * Replaces instances of FlowPair with two Flow instances.
     *
     * @param model data model
     */
    public void toUnidirectionalFlows(Model model) {
        ArrayList<Flow> flowsToAdd = new ArrayList<>();
        ArrayList<FlowPair> flowsToRemove = new ArrayList<>();
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow instanceof FlowPair) {
                FlowPair biFlow = (FlowPair) flow;
                flowsToRemove.add(biFlow);
                flowsToAdd.add(biFlow.createFlow1());
                flowsToAdd.add(biFlow.createFlow2(model));
            }
        }

        removeFlows(flowsToRemove);
        addFlows(flowsToAdd);
    }
}
