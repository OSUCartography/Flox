package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryCollectionIterator;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import edu.oregonstate.cartography.utils.ColorUtils;
import edu.oregonstate.cartography.utils.GeometryUtils;
import edu.oregonstate.cartography.utils.JTSUtils;
import java.awt.Color;
import java.awt.geom.Rectangle2D;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.StringWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.adapters.XmlJavaTypeAdapter;

/**
 * Model for Flox.
 *
 * @author Bernhard Jenny
 */
//Defines root element of JAXB XML file
@XmlRootElement

//Every non static, non transient field in a JAXB-bound class will be 
//automatically bound to XML, unless annotated by @XmlTransient
@XmlAccessorType(XmlAccessType.FIELD)

public class Model {

    /**
     * The default value for a new node.
     */
    public static final double DEFAULT_NODE_VALUE = 1;

    /**
     * The default value for a flow.
     */
    public static final double DEFAULT_FLOW_VALUE = 1;

    /**
     * default value when all nodes have same value on import.
     */
    public static final int DEFAULT_NODE_RADIUS_PX = 10;

    /**
     * Density of points along flows.
     */
    public enum FlowNodeDensity {

        // segment length in pixels relative to current map reference scale.
        LOW(50d),
        MEDIUM(20d),
        HIGH(10d);

        private final double segmentLength;

        FlowNodeDensity(double segmentLength) {
            this.segmentLength = segmentLength;
        }
    }

    /**
     * A Comparator defines conditions to compare two double values. Used to
     * select flows and nodes by their values.
     */
    public enum FlowSelector {

        /**
         * first value is greater than second value
         */
        GREATER_THAN("greater than (>)"),
        /**
         * first value is greater than or equal to second value
         */
        GREATER_THAN_OR_EQUAL("greater than or equal to (>=)"),
        /**
         * the two values are equal
         */
        EQUAL("equal to (=)"),
        /**
         * the first value is smaller than the second value
         */
        SMALLER_THAN("smaller than (<)"),
        /**
         * the first value is smaller than or equal to the second value
         */
        SMALLER_THAN_OR_EQUAL("smaller than or equal to (<=)");

        private final String description;

        private FlowSelector(String s) {
            description = s;
        }

        /**
         * Returned string is meant for use in GUI.
         *
         * @return
         */
        @Override
        public String toString() {
            return description;
        }

        /**
         * Returns whether two values satisfy the conditions of this comparator.
         *
         * @param v1 first value
         * @param v2 second value
         * @return true if values satisfy requirements, false otherwise.
         */
        public boolean match(double v1, double v2) {
            int i = Double.compare(v1, v2);
            switch (this) {
                case GREATER_THAN:
                    return i == 1;
                case GREATER_THAN_OR_EQUAL:
                    return i == 1 || i == 0;
                case EQUAL:
                    return i == 0;
                case SMALLER_THAN:
                    return i == -1;
                case SMALLER_THAN_OR_EQUAL:
                    return i == -1 || i == 0;
                default:
                    return false;
            }
        }
    }

    /**
     * Determines the maximum number of intermediate nodes per flow.
     */
    private FlowNodeDensity flowNodeDensity = FlowNodeDensity.MEDIUM;

    /**
     * Helper class for pairs of intersecting flows.
     */
    public static class IntersectingFlowPair implements Comparable<Model.IntersectingFlowPair> {

        public Flow flow1;
        public Flow flow2;
        public Point sharedNode;

        public IntersectingFlowPair(Flow flow1, Flow flow2, Point sharedNode) {
            this.flow1 = flow1;
            this.flow2 = flow2;
            this.sharedNode = sharedNode;
        }

        /**
         * Reposition control points such that the two flows no longer
         * intersect. Move control point along line connecting the opposite node
         * and the current position of the control point. The new position is
         * the intersection of that line with the line connecting the shared
         * node and the control point of the other flow. This is done for both
         * control points.
         */
        public void resolveIntersection() {
            // node shared by both flows
            double x = sharedNode.x;
            double y = sharedNode.y;
            // opposite nodes
            Point node1 = flow1.getOppositePoint(sharedNode);
            Point node2 = flow2.getOppositePoint(sharedNode);

            Point cPt1 = flow1.getCtrlPt();
            Point cPt2 = flow2.getCtrlPt();
            Point cPt1New = GeometryUtils.getLineLineIntersection(x, y, cPt2.x, cPt2.y, cPt1.x, cPt1.y, node1.x, node1.y);
            Point cPt2New = GeometryUtils.getLineLineIntersection(x, y, cPt1.x, cPt1.y, cPt2.x, cPt2.y, node2.x, node2.y);
            if (cPt1New != null && cPt2New != null) {
                if (flow1.isLocked() == false) {
                    flow1.setControlPoint(cPt1New);
                }
                if (flow2.isLocked() == false) {
                    flow2.setControlPoint(cPt2New);
                }
            }
        }

        /**
         * Sort intersecting flow pairs by summed flow values and length.
         * Returns a negative integer, zero, or a positive integer as this pair
         * is less than, equal to, or greater than the specified pair.
         *
         * @param pair pair to compare to
         * @return a negative integer, zero, or a positive integer as this
         * object is less than, equal to, or greater than the specified pair.
         */
        @Override
        public int compareTo(Model.IntersectingFlowPair pair) {
            double v1 = flow1.getValue() + flow2.getValue();
            double v2 = pair.flow1.getValue() + pair.flow2.getValue();
            int i = Double.compare(v1, v2);
            if (i == 0) {
                // if both pairs have same total value, compare lengths
                double l1 = flow1.getBaselineLength() + flow2.getBaselineLength();
                double l2 = pair.flow1.getBaselineLength() + pair.flow2.getBaselineLength();
                i = Double.compare(l1, l2);
            }
            return i;
        }
    }

    /**
     * Name of this model
     */
    private String name;

    /**
     * Number of iterations for layout computation
     */
    private int nbrIterations = 50;

    /**
     * Graph of edges (Flow class) and nodes (Point class).
     */
    @XmlJavaTypeAdapter(GraphSerializer.class)
    private Graph graph = new Graph();

    /**
     * Start and end node exert a larger force than points along flow lines.
     */
    private double nodesWeight = 0.5;

    /**
     * Weight for the anti-torsion force.
     */
    private double antiTorsionWeight = 0.8;

    /**
     * Weight of angular distribution force.
     */
    private double angularDistributionWeight = 3.75;

    /**
     * Stiffness factor for peripheral flows.
     */
    private double peripheralStiffnessFactor = 2.5;

    /**
     * spring stiffness of longest flow.
     */
    private double maxFlowLengthSpringConstant = 0.05;

    /**
     * spring stiffness of zero-length flow.
     */
    private double minFlowLengthSpringConstant = 0.5;

    /**
     * This determines the amount of force that objects far away from the target
     * can apply to the target. The lower the distanceWeightExponent, the more
     * force distant objects are permitted to apply.
     */
    private int distanceWeightExponent = 4;

    /**
     * If this is true, control points of flows are prevented from moving
     * outside of the range box of a flow.
     */
    private boolean enforceRangebox = true;

    /**
     * This value determines the width of a flows bounding box. The width of the
     * bounding box = this value * the length of the flow's baseline. Currently
     * modified by a GUI slider.
     */
    private double flowRangeboxHeight = 0.5;

    /**
     * If this is true, control points are prevented from moving outside of the
     * canvas.
     */
    private boolean enforceCanvasRange = true;

    /**
     * Determines the size of the canvas. The minimum canvas size is the
     * bounding box of all flows. This value is used to increases the length and
     * width of the minimum canvas by (length * this value) and (width * this
     * value).
     */
    private double canvasPadding = 0.2;

    /**
     * Flag to indicate whether the control points of intersecting flows
     * connected to the same node are moved.
     */
    private boolean resolveIntersectionsForSiblings = true;

    /**
     * Color for drawing the thinnest flow
     */
    @XmlJavaTypeAdapter(ColorJaxbAdaptor.class)
    private Color minFlowColor = new Color(80, 80, 80);

    /**
     * Color for drawing the thickest flow
     */
    @XmlJavaTypeAdapter(ColorJaxbAdaptor.class)
    private Color maxFlowColor = Color.BLACK;

    /**
     * If true, arrows are drawn onto the end of flows.
     */
    private boolean drawArrows = false;

    /**
     * If true direction indications are drawn on flow lines.
     */
    private boolean drawInlineArrows = false;

    /**
     * Used by the Arrow class to determine the length of arrowheads.
     */
    private double arrowLengthScaleFactor = 2.0;

    /**
     * Used by the Arrow class to determine the width of arrowheads.
     */
    private double arrowWidthScaleFactor = 0.8;

    /**
     * Used by the Arrow class to determine the location of the arrow edge
     * control points. The control point is moved from the arrowhead's base
     * towards the tip by the length of the arrowhead times this number.
     */
    private double arrowEdgeCtrlLength = 0.5;

    /**
     * Used by the Arrow class to determine the location of the arrow edge
     * control points. The control point is moved away from the centerline of
     * the arrow by the width of the arrowhead times this number.
     */
    private double arrowEdgeCtrlWidth = 0.5;

    /**
     * Used by the Arrow class to determine the horizontal position of the
     * arrow's corners relative to the base. The corners are moved from the base
     * of the arrowhead towards the tip by the length of the arrowhead times
     * this number.
     */
    private double arrowCornerPosition = 0.0;

    /**
     * Used by the Arrow class to determine the size of the smallest arrowhead.
     * Larger values result in a larger minimum arrowhead size.
     */
    private double arrowSizeRatio = 0.1;

    /**
     * Used by the Arrow class to determine the length of the longest arrowhead.
     * Smaller values result in longer arrowheads; the absolute difference
     * between the the shortest arrow and the arrow being drawn is multiplied by
     * this and subtracted from the arrow length.
     */
    private double arrowLengthRatio = 0.0;

    /**
     * Determines the gap (in pixels) between a flow's end node and the end of
     * the flow line.
     */
    private double flowDistanceFromEndPointPx = 0.0d;

    /**
     * Determines the gap (in pixels) between a flow's start node and the start
     * of the flow line.
     */
    private double flowDistanceFromStartPointPx = 0.0d;

    /**
     * Maximum allowed flow width in pixels. The flow with the highest value
     * will have this width. All other flows are scaled down relative to this
     * value.
     */
    private double maxFlowStrokeWidthPx = 20;

    /**
     * Distance between parallel flows in pixels.
     */
    private double parallelFlowsGapPx = 4;

    /**
     * Radius of largest node in pixels.
     */
    private double maxNodeSizePx = 30;

    /**
     * Stroke width for drawing node circles. In pixels.
     */
    private float nodeStrokeWidthPx = 2;

    /**
     * Color for drawing node outlines
     */
    @XmlJavaTypeAdapter(ColorJaxbAdaptor.class)
    private Color nodeStrokeColor = Color.BLACK;

    /**
     * Color for filling nodes.
     */
    @XmlJavaTypeAdapter(ColorJaxbAdaptor.class)
    private Color nodeFillColor = Color.WHITE;

    /**
     * scale factor for converting between ground coordinates and the map
     * coordinates in pixels. Note: this scale factor is independent of the
     * scale factor of the map component that displays the map. Conversion:
     * mapPixelX = groundX * referenceMapScale;
     */
    private double referenceMapScale = 1;

    /**
     * Clip the ends of flows. This flags is only used to update the GUI.
     */
    private boolean clipFlowEnds = true;

    /**
     * Clip the beginnings of flows. This flags is only used to update the GUI.
     */
    private boolean clipFlowStarts = true;

    /**
     * A geometry (collection) used for clipping start or end of flows.
     */
    @XmlJavaTypeAdapter(GeometrySerializer.class)
    private Geometry clipAreas;

    /**
     * Buffer width for start clip areas in pixels.
     */
    private double startClipAreaBufferDistancePx = 30;

    /**
     * Buffer width for end clip areas in pixels.
     */
    private double endClipAreaBufferDistancePx = 30;

    /**
     * whether to move flows that overlap obstacles (i.e. nodes and arrowheads)
     */
    private boolean moveFlowsOverlappingObstacles = true;

    /**
     * Minimum distance of flows from obstacles in pixels.
     */
    private int minObstacleDistPx = 5;

    /**
     * A map with a set of symbolized layers.
     */
    private final Map map = new Map();

    /**
     * Background color of map.
     */
    @XmlJavaTypeAdapter(ColorJaxbAdaptor.class)
    private Color backgroundColor = Color.WHITE;

    /**
     * Constructor of the model.
     */
    public Model() {
    }

    /**
     * Copy model.
     *
     * @return an exact copy of this model.
     */
    public Model copy() {
        try {
            return Model.unmarshal(marshal());
        } catch (JAXBException exc) {
            throw new IllegalStateException(exc);
        }
    }

    private static JAXBContext getJAXBContext() throws JAXBException {
        String packageName = Model.class.getPackage().getName();
        return JAXBContext.newInstance(packageName, Model.class.getClassLoader());
    }

    public static Model unmarshal(InputStream is) throws JAXBException {
        Unmarshaller unmarshaller = getJAXBContext().createUnmarshaller();
        return (Model) unmarshaller.unmarshal(is);
    }

    public static Model unmarshal(byte[] buf) throws JAXBException {
        return unmarshal(new ByteArrayInputStream(buf));
    }

    public static Model unmarshal(String fileName) throws JAXBException, FileNotFoundException {
        FileInputStream is = null;
        try {
            is = new FileInputStream(fileName);
            return unmarshal(is);
        } finally {
            try {
                if (is != null) {
                    is.close();
                }
            } catch (IOException ex) {
                Logger.getLogger(Model.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    public void marshal(OutputStream os) throws JAXBException {
        Marshaller m = getJAXBContext().createMarshaller();
        m.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
        m.marshal(this, os);
    }

    public void marshal(Writer w) throws JAXBException {
        Marshaller m = getJAXBContext().createMarshaller();
        m.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
        m.setProperty(Marshaller.JAXB_ENCODING, "UTF-8");
        m.marshal(this, w);
    }

    public byte[] marshal() throws JAXBException {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        marshal(baos);
        return baos.toByteArray();
    }

    public void marshal(String fileName) throws JAXBException, FileNotFoundException {
        FileOutputStream os = null;
        try {
            os = new FileOutputStream(fileName);
            marshal(os);
        } finally {
            try {
                if (os != null) {
                    os.close();
                }
            } catch (IOException ex) {
                Logger.getLogger(Model.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    @Override
    public String toString() {
        StringWriter sw = new StringWriter();
        try {
            marshal(sw);
            return sw.toString();
        } catch (JAXBException ex) {
            Logger.getLogger(Model.class.getName()).log(Level.SEVERE, null, ex);
            return "could not serliaze model";
        }
    }

    /**
     * Change control point location of a flow.
     *
     * @param id identifier of the flow in the graph
     * @param x horizontal coordinate of control point
     * @param y vertical coordinate of control point
     */
    public void replaceControlPoint(long id, double x, double y) {
        // FIXME inefficient iteration, should use hash map
        Iterator<Flow> iter = graph.flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            if (flow.id == id) {
                flow.getCtrlPt().x = x;
                flow.getCtrlPt().y = y;
                break;
            }
        }
    }

    /**
     * Calculates the ratio between the maximum allowed node size and the
     * highest node value. Needed for drawing nodes to the correct radius
     * relative to the maximum radius permitted by the GUI settings. If there
     * are no nodes on the map yet, it assumes a max node value of 1, which is
     * the default value of new Points.
     *
     * @return node size scale factor
     */
    public double getNodeSizeScaleFactor() {

        // Find the maximum node area as permitted by settings
        double area = Math.PI * (maxNodeSizePx * maxNodeSizePx);

        // Get the maximum current node value
        double maxVal = getMaxNodeValue();

        // return the ratio of the maximum allowed node area and the current
        // max node value.
        if (maxVal == 0) { // There are no nodes yet
            return area / 1; // The value of the new node will be 1 by default
        } else {
            return area / maxVal;
        }
    }

    /**
     * Gets the maximum value of all nodes on the map.
     *
     * @return maximum node value
     */
    public double getMaxNodeValue() {
        return graph.getMaxNodeValue();
    }

    /**
     * Gets the minimum value of all nodes on the map.
     *
     * @return maximum node value
     */
    public double getMinNodeValue() {
        return graph.getMinNodeValue();
    }

    /**
     * Gets the average value of all nodes on the map.
     *
     * @return mean node value
     */
    public double getMeanNodeValue() {
        return graph.getMeanNodeValue();
    }

    /**
     * Gets the average value of all flows on the map.
     *
     * @return
     */
    public double getMeanFlowValue() {
        return graph.getMeanFlowValue();
    }

    /**
     * Returns the number of flows.
     *
     * @return The number of flows.
     */
    public int getNbrFlows() {
        return graph.getNbrFlows();
    }

    /**
     * Returns the number of nodes in the graph.
     *
     * @return The number of nodes.
     */
    public int getNbrNodes() {
        return graph.getNbrNodes();
    }

    /**
     * Add a flow.
     *
     * @param flow The flow to add.
     */
    public void addFlow(Flow flow) {
        graph.addFlow(flow);
    }

    /**
     * Selects all nodes that are not connected to any other node. Does not
     * change the selection state of other nodes or flows.
     *
     * @return number of unconnected nodes
     */
    public int selectUnconnectedNodes() {
        int n = 0;
        Iterator<Point> iter = nodeIterator();
        while (iter.hasNext()) {
            Point node = iter.next();
            if (graph.getFlowsForNode(node).isEmpty()) {
                node.setSelected(true);
                ++n;
            }
        }
        return n;
    }

    /**
     * Select all nodes that satisfy the conditions of a Comparator.
     *
     * @param comparator determines what conditions need to be met to select a
     * node
     * @param threshold threshold value
     * @return number of nodes that meet the conditions of the Comparator
     */
    public int selectNodesByValue(FlowSelector comparator, double threshold) {
        int n = 0;
        Iterator<Point> iter = nodeIterator();
        while (iter.hasNext()) {
            Point node = iter.next();
            if (comparator.match(node.getValue(), threshold)) {
                node.setSelected(true);
                ++n;
            }
        }
        return n;
    }

    /**
     * Select all flows that satisfy the conditions of a Comparator.
     *
     * @param comparator determines what conditions need to be met to select a
     * flow
     * @param threshold threshold value
     * @return number of flows that meet the conditions of the Comparator
     */
    public int selectFlowsByValue(FlowSelector comparator, double threshold) {
        int n = 0;
        Iterator<Flow> iter = flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            if (comparator.match(flow.getValue(), threshold)) {
                flow.setSelected(true);
                ++n;
            }
        }
        return n;
    }

    /**
     * Returns the number of nodes that are not connected to any other node.
     *
     * @return number of unconnected nodes
     */
    public int countUnconnectedNodes() {
        int nbr = 0;
        Iterator<Point> iter = nodeIterator();
        while (iter.hasNext()) {
            Point node = iter.next();
            if (graph.getFlowsForNode(node).isEmpty()) {
                ++nbr;
            }
        }
        return nbr;
    }

    public void mergeSelectedNodes() {
        ArrayList<Point> nodes = getSelectedNodes();
        if (nodes.size() < 2) {
            return;
        }

        // location and value of the new node
        double x = nodes.get(0).x;
        double y = nodes.get(0).y;
        double v = nodes.get(0).getValue();
        for (int i = 1; i < nodes.size(); i++) {
            x += nodes.get(i).x;
            y += nodes.get(i).y;
            v += nodes.get(i).getValue();
        }
        x /= nodes.size();
        y /= nodes.size();
        Point mergedNode = new Point(x, y, v);

        // find all flows connected to the nodes that will be merged
        ArrayList<Flow> flowsToMerg = new ArrayList<>();
        for (Point node : nodes) {
            Collection<Flow> flows = graph.getFlowsForNode(node);
            flowsToMerg.addAll(flows);
        }

        // remove flows that will be merged from the graph
        for (Flow flow : flowsToMerg) {
            graph.removeFlow(flow);
        }

        // add merged flows
        for (Flow flow : flowsToMerg) {
            // ignore collapsing flows
            if (flow.getStartPt().isSelected() && flow.getEndPt().isSelected()) {
                continue;
            }
            // test whether a flow has been added between the merged node and the opposite node of this flow 
            Flow existingFlow = graph.getFirstFlowBetweenNodes(flow.getStartPt(), mergedNode);
            if (existingFlow == null) {
                existingFlow = graph.getFirstFlowBetweenNodes(flow.getEndPt(), mergedNode);
            }

            if (existingFlow == null) {
                // add a new flow to the graph
                // replace start or end node of existing flows
                if (flow.getStartPt().isSelected()) {
                    flow.setStartPt(mergedNode);
                    //graph.removeVertex(flow.getStartPt());
                } else {
                    flow.setEndPt(mergedNode);
                    //graph.removeVertex(flow.getEndPt());
                }
                graph.addFlow(flow);
            } else {
                // increase the flow value of the existing flow
                existingFlow.setValue(existingFlow.getValue() + flow.getValue());
                graph.updateCachedValues();
            }
        }

    }

    /**
     * Delete a flow.
     *
     * @param flow The flow to delete
     */
    public void deleteFlow(Flow flow) {
        graph.removeFlow(flow);
    }

    /**
     * Returns the flow from the end to the start point of the passed flow, if
     * it exists.
     *
     * @param flow search for flow in opposite direction to this flow
     * @return flow with opposite direction or null
     */
    public Flow getOpposingFlow(Flow flow) {
        return graph.getOpposingFlow(flow);
    }

    /**
     * Inverse the direction of all selected flows.
     */
    public void reverseSelectedFlows() {
        ArrayList<Flow> flows = getSelectedFlows();
        for (Flow flow : flows) {
            graph.removeFlow(flow);
            flow.reverseFlow(this);
            graph.addFlow(flow);
        }
    }

    /**
     * Delete a node.
     *
     * @param node The node to delete.
     */
    public void deleteNode(Point node) {
        graph.removeNode(node);
    }

    /**
     * Removes all selected nodes and flows from the graph. Selects nodes that
     * were connected to a selected flow.
     *
     * @return The number of flows and nodes that were deleted.
     */
    public int deleteSelectedFlowsAndNodes() {
        ArrayList<Flow> flowsToRemove = new ArrayList<>();
        ArrayList<Point> nodesToRemove = new ArrayList<>();

        Iterator<Flow> iter = flowIterator();
        while (iter.hasNext()) {
            Flow flow = iter.next();
            if (flow.isSelected()) {
                flowsToRemove.add(flow);
            }
        }

        Iterator nodes = nodeIterator();
        while (nodes.hasNext()) {
            Point node = (Point) nodes.next();
            if (node.isSelected()) {
                nodesToRemove.add(node);
            }
        }

        flowsToRemove.stream().forEach((flow) -> {
            flow.getStartPt().setSelected(true);
            flow.getEndPt().setSelected(true);
            deleteFlow(flow);
        });

        nodesToRemove.stream().forEach((node) -> {
            deleteNode(node);
        });

        return flowsToRemove.size() + nodesToRemove.size();
    }

    /**
     * Replace the current flows with new flows.
     *
     * @param flows The new flows.
     */
    public void setFlows(Collection<Flow> flows) {
        // reset the graph
        graph = new Graph();

        // add new flows
        graph.addFlows(flows);
    }

    /**
     * Finds the first OGC simple feature that contains a point.
     *
     * @param geometry A geometry or geometry collection to search through.
     * @param point The point for which a containing geometry is to be searched.
     * @param f A JTS GeometryFactory
     * @return The found containing geometry or null if none is found.
     */
    private Geometry findContainingGeometry(Geometry geometry,
            Point point, GeometryFactory f) {
        if (geometry == null) {
            return null;
        }

        Coordinate coordinate = new Coordinate(point.x, point.y);
        com.vividsolutions.jts.geom.Point p = f.createPoint(coordinate);

        Iterator geomi = new GeometryCollectionIterator(geometry);
        while (geomi.hasNext()) {
            Geometry g = (Geometry) geomi.next();
            if (g instanceof GeometryCollection == false && p.within(g)) {
                return g;
            }
        }

        return null;
    }

    /**
     * Finds the clip areas for the start of one flow. First buffers the clip
     * area geometry, then finds containing geometry for the start point of the
     * flow, then assigns the start clip area to the flow.
     *
     * @param flow flow to find start clip area for
     */
    public void updateStartClipArea(Flow flow) {
        if (clipAreas == null) {
            return;
        }

        GeometryFactory f = new GeometryFactory();
        Geometry startClipArea = findContainingGeometry(clipAreas, flow.getStartPt(), f);
        if (startClipArea != null) {
            double d = startClipAreaBufferDistancePx / getReferenceMapScale();
            startClipArea = startClipArea.buffer(-d);
        }
        flow.setStartClipArea(startClipArea);
    }

    /**
     * Finds the clip areas for the start of flows.
     */
    public void updateStartClipAreas() {
        if (clipAreas == null) {
            return;
        }
        Iterator<Flow> flowIterator = flowIterator();
        while (flowIterator.hasNext()) {
            updateStartClipArea(flowIterator.next());
        }
    }

    /**
     * Finds the clip areas for the the end of one flow. First buffers the clip
     * area geometry, then finds the containing geometry for the end point of
     * the flow, then assigns the end clip area to the flow.
     *
     * @param flow flow to find end clip area for
     */
    public void updateEndClipArea(Flow flow) {
        if (clipAreas == null) {
            return;
        }

        GeometryFactory f = new GeometryFactory();
        Geometry endClipArea = findContainingGeometry(clipAreas, flow.getEndPt(), f);
        if (endClipArea != null) {
            double d = endClipAreaBufferDistancePx / getReferenceMapScale();
            endClipArea = endClipArea.buffer(-d);
        }
        flow.setEndClipArea(endClipArea);
    }

    /**
     * Finds the clip areas for the the end of flows.
     */
    public void updateEndClipAreas() {
        if (clipAreas == null) {
            return;
        }
        Iterator<Flow> flowIterator = flowIterator();
        GeometryFactory f = new GeometryFactory();
        while (flowIterator.hasNext()) {
            updateEndClipArea(flowIterator.next());
        }
    }

    /**
     * Set the start and end clip areas for flows.
     *
     * @param clipAreas
     */
    public void setClipAreas(Geometry clipAreas) {
        this.clipAreas = clipAreas;

        if (isClipFlowEnds()) {
            updateEndClipAreas();
        }
        if (isClipFlowStarts()) {
            updateStartClipAreas();
        }
    }

    /**
     * Returns true if clip areas exist.
     *
     * @return
     */
    public boolean hasClipAreas() {
        return clipAreas != null;
    }

    /**
     * Removes the end clip areas from all flows.
     */
    public void removeEndClipAreasFromFlows() {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            iterator.next().setEndClipArea(null);
        }
    }

    /**
     * Removes the start clip areas from all flows.
     */
    public void removeStartClipAreasFromFlows() {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            iterator.next().setStartClipArea(null);
        }
    }

    /**
     * @return the startClipAreaBufferDistance
     */
    public double getStartClipAreaBufferDistancePx() {
        return startClipAreaBufferDistancePx;
    }

    /**
     * @param startClipAreaBufferDistance the startClipAreaBufferDistance to set
     */
    public void setStartClipAreaBufferDistancePx(double startClipAreaBufferDistance) {
        this.startClipAreaBufferDistancePx = startClipAreaBufferDistance;
        updateStartClipAreas();
    }

    /**
     * @return the endClipAreaBufferDistance
     */
    public double getEndClipAreaBufferDistancePx() {
        return endClipAreaBufferDistancePx;
    }

    /**
     * @param endClipAreaBufferDistance the endClipAreaBufferDistance to set
     */
    public void setEndClipAreaBufferDistancePx(double endClipAreaBufferDistance) {
        this.endClipAreaBufferDistancePx = endClipAreaBufferDistance;
        updateEndClipAreas();
    }

    /**
     * Returns a list of obstacles, i.e., arrowheads and nodes.
     *
     * @return list of obstacles
     */
    public List<Obstacle> getObstacles() {
        List<Obstacle> obstacles = new ArrayList<>();

        // nodes are obstacles
        Iterator<Point> nodeIterator = nodeIterator();
        while (nodeIterator.hasNext()) {
            Point node = nodeIterator.next();

            // nodes are obstacles
            double nodeRadiusPx = getNodeRadiusPx(node);
            double strokeWidthPx = getNodeStrokeWidthPx();
            final double rPx;
            if (strokeWidthPx > nodeRadiusPx * 2) {
                // stroke is wider than the diameter of the circle
                rPx = strokeWidthPx;
            } else {
                rPx = nodeRadiusPx + 0.5 * strokeWidthPx;
            }
            double rWorld = rPx / getReferenceMapScale();
            obstacles.add(new Obstacle(node, node.x, node.y, rWorld));
        }

        // arrowheads are obstacles
        if (isDrawArrowheads()) {
            Iterator<Flow> flowIterator = flowIterator();
            while (flowIterator.hasNext()) {
                Flow flow = flowIterator.next();
                Arrow arrow;
                if (flow instanceof FlowPair) {
                    arrow = ((FlowPair) flow).createParallelFlow1(this).getArrow(this);
                    if (arrow.getLength() > Circle.TOL && arrow.getWidth() > Circle.TOL) {
                        Obstacle obstacle = new Obstacle(arrow.getTipPt(),
                                arrow.getCorner1Pt(), arrow.getCorner2Pt(), flow);
                        obstacles.add(obstacle);
                    }
                    arrow = ((FlowPair) flow).createParallelFlow2(this).getArrow(this);
                } else {
                    arrow = flow.getArrow(this);
                }

                if (arrow.getLength() > Circle.TOL && arrow.getWidth() > Circle.TOL) {
                    Obstacle obstacle = new Obstacle(arrow.getTipPt(),
                            arrow.getCorner1Pt(), arrow.getCorner2Pt(), flow);
                    obstacles.add(obstacle);
                }
            }
        }

        return obstacles;
    }

    /**
     * Returns the geometry of all layers.
     *
     * @return the geometry
     */
    public GeometryCollection getLayerGeometry() {
        return map.getGeometryCollection();
    }

    /**
     * Returns the bounding box of all flows, excluding the other geometry.
     *
     * @return null if no flows exist.
     */
    public Rectangle2D getFlowsBoundingBox() {
        return graph.getFlowsBoundingBox();
    }

    /**
     * Compute the bounding box for all map geometry, including the flows.
     *
     * @return The bounding box.
     */
    public Rectangle2D getBoundingBox() {
        Rectangle2D mapBB = map.getBoundingBox();
        Rectangle2D flowsBB = getFlowsBoundingBox();
        if (mapBB != null) {
            return flowsBB == null ? mapBB : mapBB.createUnion(flowsBB);
        }
        return flowsBB;
    }

    /**
     * Returns an iterator for the flows.
     *
     * @return The iterator.
     */
    public Iterator<Flow> flowIterator() {
        return graph.flowIterator();
    }

    /**
     * Returns an iterator for all sorted flows. Each FlowPair is converted to
     * two regular Flows.
     *
     * @param increasing increasing or decreasing sort order
     * @return iterator for sorted flows
     */
    public Iterator<Flow> sortedFlowIteratorForDrawing(boolean increasing) {
        return graph.getSortedFlowsForDrawing(this, increasing).iterator();
    }

    // FIXME remove?
    public ArrayList<Flow> getFlows() {
        return graph.getFlows();
    }

    /**
     * Returns a new ArrayList with references to the nodes in increasing or
     * decreasing order.
     *
     * @param increasing If true, the nodes are arranged in increasing order.
     * @return A new ArrayList with references to the nodes in the graph.
     */
    public ArrayList<Point> getSortedNodes(boolean increasing) {
        return graph.getSortedNodes(increasing);
    }

    /**
     * Returns an iterator for the nodes.
     *
     * @return The iterator.
     */
    public Iterator<Point> nodeIterator() {
        return graph.nodeIterator();
    }

    /**
     * Returns all nodes in the graph. This method should be avoided because it
     * creates an extra ArrayList . An iterator should be used instead. However,
     * some applications (e.g. selection tool) require a reverse iteration.
     *
     * @return All nodes.
     */
    public ArrayList<Point> getNodes() {
        return graph.getNodes();
    }

    /**
     * Returns a list of flows connected to a node. The flows are ordered
     * anti-clockwise by the orientation of the line connecting start and end
     * points. The origin of the polar coordinate system is the horizontal x
     * axis.
     *
     * @param node the node to search connected flows for.
     * @return a list with the ordered flows.
     */
    public List<Flow> getAnticlockwiseOrderedFlowsAtNode(Point node) {
        return graph.getAnticlockwiseOrderedFlowsAtNode(node);
    }

    /**
     * Returns the maximum flow value.
     *
     * @return The maximum flow value.
     */
    public double getMaxFlowValue() {
        return graph.getMaxFlowValue();
    }

    public double getMinFlowValue() {
        return graph.getMinFlowValue();
    }

    /**
     * Get the length of longest flow baseline.
     *
     * @return the length of the longest flow baseline
     */
    public double getLongestFlowLength() {
        return graph.getLongestFlowLength();
    }

    /**
     * Get the length of the shortest flow baseline.
     *
     * @return the shortest flow baseline.
     */
    public double getShortestFlowLength() {
        return graph.getShortestFlowLength();
    }

    /**
     * Converts all flows that are not locked to straight lines.
     *
     * @param onlySelected If true, only flows that are selected are converted
     * to straight lines.
     */
    public void straightenFlows(boolean onlySelected) {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (onlySelected && flow.isSelected() == false) {
                continue;
            }
            if (!flow.isLocked()) {
                flow.straighten();
            }
        }
    }

    /**
     * Returns an ideal length for segmenting flows.
     *
     * @return the segment length
     */
    public double segmentLength() {
        return flowNodeDensity.segmentLength / getReferenceMapScale();
    }

    /**
     * Returns true if any flows are selected
     *
     * @return True if any flows are selected
     */
    public boolean isFlowSelected() {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isSelected()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns true if a locked flow is selected
     *
     * @return
     */
    public boolean isLockedFlowSelected() {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isSelected() && flow.isLocked()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns true if an unlocked flow is selected.
     *
     * @return true if an unlocked flow is selected
     */
    public boolean isUnlockedFlowSelected() {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isSelected() && !flow.isLocked()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Gets an ArrayList containing all selected flows.
     *
     * @return an ArrayList of all selected flows.
     */
    public ArrayList<Flow> getSelectedFlows() {
        ArrayList<Flow> selectedFlows = new ArrayList();
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isSelected()) {
                selectedFlows.add(flow);

            }
        }
        return selectedFlows;
    }

    /**
     * Change the flow value of all selected flows.
     *
     * @param value new value for selected flows.
     */
    public void setValueOfSelectedFlows(double value) {
        ArrayList<Flow> selectedFlows = getSelectedFlows();
        for (Flow selectedFlow : selectedFlows) {
            selectedFlow.setValue(value);
        }
        graph.updateCachedValues();
    }

    /**
     * Change the node value of all selected flows.
     *
     * @param value new value for selected nodes.
     */
    public void setValueOfSelectedNodes(double value) {
        ArrayList<Point> selectedPoints = getSelectedNodes();
        for (Point selectedPoint : selectedPoints) {
            selectedPoint.setValue(value);
        }
        graph.updateCachedValues();
    }

    /**
     * Returns true if a node is selected.
     *
     * @return True if a node is selected.
     */
    public boolean isNodeSelected() {
        Iterator nodes = nodeIterator();
        while (nodes.hasNext()) {
            Point node = (Point) nodes.next();
            if (node.isSelected()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Gets an ArrayList of all selected nodes.
     *
     * @return ArrayList of all selected nodes.
     */
    public ArrayList<Point> getSelectedNodes() {

        ArrayList<Point> selectedNodes = new ArrayList();
        Iterator nodes = nodeIterator();
        while (nodes.hasNext()) {
            Point node = (Point) nodes.next();
            if (node.isSelected()) {
                selectedNodes.add(node);
            }
        }
        return selectedNodes;
    }

    /**
     * Selects or deselects all flows and nodes.
     *
     * @param select If true, all flows and nodes are selected. All are
     * deselected otherwise.
     */
    public void setSelectionOfAllFlowsAndNodes(boolean select) {
        setSelectionOfAllFlows(select);
        setSelectionOfAllNodes(select);
    }

    /**
     * Set selection state of all nodes
     *
     * @param selected new selection state
     */
    public void setSelectionOfAllNodes(boolean selected) {
        Iterator<Point> iter = nodeIterator();
        while (iter.hasNext()) {
            iter.next().setSelected(selected);
        }
    }

    /**
     * Set selection state of all flows.
     *
     * @param selected new selection state
     */
    public void setSelectionOfAllFlows(boolean selected) {
        Iterator<Flow> iter = flowIterator();
        while (iter.hasNext()) {
            iter.next().setSelected(selected);
        }
    }

    /**
     * Set the lock for all selected flows.
     *
     * @param lock The new lock state.
     */
    public void setLockOfSelectedFlows(boolean lock) {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.isSelected()) {
                flow.setLocked(lock);
            }
        }
    }

    /**
     * Returns the lock flags of all flows.
     *
     * @return An array with all lock flags in the order of the iterator
     * returned by flowIterator().
     */
    public boolean[] getLocks() {
        boolean[] locks = new boolean[getNbrFlows()];
        Iterator<Flow> flowIterator = flowIterator();
        int i = 0;
        while (flowIterator.hasNext()) {
            locks[i++] = flowIterator.next().isLocked();
        }
        return locks;
    }

    /**
     * Apply lock flags to all flows. Flags must be in the order of an iterator
     * returned by flowIterator().
     *
     * @param locks An array with lock flags.
     */
    public void applyLocks(boolean[] locks) {
        Iterator<Flow> flowIterator = flowIterator();
        int i = 0;
        while (flowIterator.hasNext()) {
            flowIterator.next().setLocked(locks[i++]);
        }
    }

    /**
     * Returns all map layers.
     *
     * @return
     */
    public Collection<Layer> getLayers() {
        return map.getLayers();
    }

    /**
     * Returns a layer by its name.
     *
     * @param name Name must contain at least one non-empty character.
     * @return The found layer or null if non is found.
     */
    public Layer getLayer(String name) {
        return map.getLayer(name);
    }

    /**
     * Returns a layer specified by an index.
     *
     * @param id The index of the layer to return.
     * @return
     */
    public Layer getLayer(int id) {
        return map.getLayer(id);
    }

    /**
     * Add a layer to the map.
     *
     * @param collection Geometry for the layer.
     * @return The new layer.
     */
    public Layer addLayer(GeometryCollection collection) {
        return map.addLayer(collection);
    }

    /**
     * Add a layer to the map.
     *
     * @param layer The layer to add.
     */
    public void addLayer(Layer layer) {
        map.addLayer(layer);
    }

    /**
     * Remove all layers from the map. This does not remove flows.
     */
    public void removeAllLayers() {
        map.removeAllLayers();
    }

    /**
     * Remove a layer.
     *
     * @param id Index of the layer to remove.
     */
    public void removeLayer(int id) {
        map.removeLayer(id);
    }

    /**
     * Returns the number of map layers.
     *
     * @return
     */
    public int getNbrLayers() {
        return map.getNbrLayers();
    }

    /**
     * Returns the background color of the map
     *
     * @return the backgroundColor
     */
    public Color getBackgroundColor() {
        return backgroundColor;
    }

    /**
     * Set the background color of the map.
     *
     * @param backgroundColor the backgroundColor to set
     */
    public void setBackgroundColor(Color backgroundColor) {
        this.backgroundColor = backgroundColor;
    }

    /**
     * @return the nodesWeight
     */
    public double getNodesWeight() {
        return nodesWeight;
    }

    /**
     * @param nodesWeight the nodesWeight to set
     */
    public void setNodesWeight(double nodesWeight) {
        this.nodesWeight = nodesWeight;
    }

    /**
     * @return the antiTorsionWeight
     */
    public double getAntiTorsionWeight() {
        return antiTorsionWeight;
    }

    /**
     * @param antiTorsionWeight the antiTorsionWeight to set
     */
    public void setAntiTorsionWeight(double antiTorsionWeight) {
        //assert (antiTorsionWeight >= 0 && antiTorsionWeight <= 1);
        this.antiTorsionWeight = antiTorsionWeight;
    }

    /**
     * @return the peripheralStiffnessFactor
     */
    public double getPeripheralStiffnessFactor() {
        return peripheralStiffnessFactor;
    }

    /**
     * @param peripheralStiffnessFactor the peripheralStiffnessFactor to set
     */
    public void setPeripheralStiffnessFactor(double peripheralStiffnessFactor) {
        this.peripheralStiffnessFactor = peripheralStiffnessFactor;
    }

    /**
     * Sets the spring constants. This is ultimately called by slider bars in
     * the GUI.
     *
     * @param maxFlowLengthSpringConstant The stiffness of the spring of the
     * longest flow
     * @param minFlowLengthSpringConstant The minimum spring stiffness of all
     * springs on the map.
     */
    public void setSpringConstants(double maxFlowLengthSpringConstant, double minFlowLengthSpringConstant) {
        this.setMaxFlowLengthSpringConstant(maxFlowLengthSpringConstant);
        this.setMinFlowLengthSpringConstant(minFlowLengthSpringConstant);
    }

    /**
     * @return the distanceWeightExponent
     */
    public int getDistanceWeightExponent() {
        return distanceWeightExponent;
    }

    /**
     * Sets the distanceWeightExponent. This is currently set by the slider bar
     * in the GUI.
     *
     * @param idwExponent The distanceWeightExponent to set
     */
    public void setDistanceWeightExponent(int idwExponent) {
        this.distanceWeightExponent = idwExponent;
    }

    /**
     * @return the enforceRangebox
     */
    public boolean isEnforceRangebox() {
        return enforceRangebox;
    }

    public void setEnforceRangebox(boolean enforceRangebox) {
        this.enforceRangebox = enforceRangebox;
    }

    /**
     * @return the enforceCanvasRange
     */
    public boolean isEnforceCanvasRange() {
        return enforceCanvasRange;
    }

    public void setEnforceCanvasRange(boolean enforceCanvasRange) {
        this.enforceCanvasRange = enforceCanvasRange;
    }

    public void setCanvasPadding(double canvasPadding) {
        this.canvasPadding = canvasPadding;
    }

    public double getCanvasPadding() {
        return canvasPadding;
    }

    /**
     * Returns the size of the map canvas. The canvas is defined by the bounding
     * box around all start and end nodes and the percentage defined by
     * canvasPadding.
     *
     * @return canvas rectangle in world coordinates. Null if no nodes exist.
     */
    public Rectangle2D getCanvas() {
        Rectangle2D nodesBoundingBox = getNodesBoundingBox();
        if (nodesBoundingBox != null) {
            double w = nodesBoundingBox.getWidth();
            double h = nodesBoundingBox.getHeight();
            double x = nodesBoundingBox.getX();
            double y = nodesBoundingBox.getY();
            // the additional padding around the nodes is a percentage of the bounding box
            double xPad = w * getCanvasPadding();
            double yPad = h * getCanvasPadding();
            nodesBoundingBox.setRect(x - xPad, y - yPad, w + 2 * xPad, h + 2 * yPad);
        }
        return nodesBoundingBox;
    }

    /**
     * Returns the bounding box containing all start and end points.
     *
     * @return the bounding box in world coordinates. Null if no nodes exist.
     */
    public Rectangle2D getNodesBoundingBox() {
        Iterator<Point> nodeIterator = nodeIterator();
        if (nodeIterator.hasNext() == false) {
            return null;
        }
        Point pt = nodeIterator.next();
        Rectangle2D.Double bb = new Rectangle2D.Double(pt.x, pt.y, 0, 0);
        while (nodeIterator.hasNext()) {
            pt = nodeIterator.next();
            bb.add(pt.x, pt.y);
        }
        return bb;
    }

    /**
     * @return the maxFlowLengthSpringConstant
     */
    public double getMaxFlowLengthSpringConstant() {
        return maxFlowLengthSpringConstant;
    }

    /**
     * @return the minFlowLengthSpringConstant
     */
    public double getMinFlowLengthSpringConstant() {
        return minFlowLengthSpringConstant;
    }

    /**
     * @return the flowRangeboxHeight
     */
    public double getFlowRangeboxHeight() {
        return flowRangeboxHeight;
    }

    /**
     * @param flowRangeboxHeight the flowRangeboxHeight to set
     */
    public void setFlowRangeboxHeight(double flowRangeboxHeight) {
        this.flowRangeboxHeight = flowRangeboxHeight;
    }

    /**
     * @return the flowArrowEndPointRadius
     */
    public double getFlowDistanceFromEndPointPixel() {
        return flowDistanceFromEndPointPx;
    }

    /**
     * @return the flowArrowEndPointRadius
     */
    public double getFlowDistanceFromStartPointPixel() {
        return flowDistanceFromStartPointPx;
    }

    /**
     * @param flowArrowEndPointRadius the flowArrowEndPointRadius to set
     */
    public void setFlowDistanceFromEndPointPixel(double flowArrowEndPointRadius) {
        this.flowDistanceFromEndPointPx = flowArrowEndPointRadius;
    }

    public void setFlowDistanceFromStartPointPixel(double flowArrowStartPointRadius) {
        this.flowDistanceFromStartPointPx = flowArrowStartPointRadius;
    }

    /**
     * @return true if arrowheads are added to flows.
     */
    public boolean isDrawArrowheads() {
        return drawArrows;
    }

    /**
     * @param addArrows set whether arrowheads are to be added.
     */
    public void setDrawArrowheads(boolean addArrows) {
        this.drawArrows = addArrows;
    }

    /**
     * @return the arrowSideCurveFactor
     */
    public double getArrowEdgeCtrlLength() {
        return arrowEdgeCtrlLength;
    }

    /**
     * @param arrowSideCurveFactor the arrowSideCurveFactor to set
     */
    public void setArrowEdgeCtrlLength(double arrowSideCurveFactor) {
        this.arrowEdgeCtrlLength = arrowSideCurveFactor;
    }

    /**
     * @return the arrowEdgeCtrlWidth
     */
    public double getArrowEdgeCtrlWidth() {
        return arrowEdgeCtrlWidth;
    }

    /**
     * @param arrowEdgeCtrlWidth the arrowEdgeCtrlWidth to set
     */
    public void setArrowEdgeCtrlWidth(double arrowEdgeCtrlWidth) {
        this.arrowEdgeCtrlWidth = arrowEdgeCtrlWidth;
    }

    /**
     * @return the arrowCornerPosition
     */
    public double getArrowCornerPosition() {
        return arrowCornerPosition;
    }

    /**
     * @param arrowCornerPosition the arrowCornerPosition to set
     */
    public void setArrowCornerPosition(double arrowCornerPosition) {
        this.arrowCornerPosition = arrowCornerPosition;
    }

    /**
     * @return the arrowSizeRatio
     */
    public double getArrowSizeRatio() {
        return arrowSizeRatio;
    }

    public double getArrowLengthRatio() {
        return arrowLengthRatio;
    }

    /**
     * @param arrowSizeRatio the arrowSizeRatio to set
     */
    public void setArrowSizeRatio(double arrowSizeRatio) {
        this.arrowSizeRatio = arrowSizeRatio;
    }

    public void setArrowLengthRatio(double arrowLengthRatio) {
        this.arrowLengthRatio = arrowLengthRatio;
    }

    /**
     * @param maxFlowLengthSpringConstant the maxFlowLengthSpringConstant to set
     */
    public void setMaxFlowLengthSpringConstant(double maxFlowLengthSpringConstant) {
        this.maxFlowLengthSpringConstant = maxFlowLengthSpringConstant;
    }

    /**
     * @param minFlowLengthSpringConstant the minFlowLengthSpringConstant to set
     */
    public void setMinFlowLengthSpringConstant(double minFlowLengthSpringConstant) {
        this.minFlowLengthSpringConstant = minFlowLengthSpringConstant;
    }

    /**
     * @return the arrowLengthScaleFactor
     */
    public double getArrowLengthScaleFactor() {
        return arrowLengthScaleFactor;
    }

    /**
     * @param arrowLengthScaleFactor the arrowLengthScaleFactor to set
     */
    public void setArrowLengthScaleFactor(double arrowLengthScaleFactor) {
        this.arrowLengthScaleFactor = arrowLengthScaleFactor;
    }

    /**
     * @return the arrowWidthScaleFactor
     */
    public double getArrowWidthScaleFactor() {
        return arrowWidthScaleFactor;
    }

    /**
     * @param arrowWidthScaleFactor the arrowWidthScaleFactor to set
     */
    public void setArrowWidthScaleFactor(double arrowWidthScaleFactor) {
        this.arrowWidthScaleFactor = arrowWidthScaleFactor;
    }

    /**
     * Checks to see if any control points are selected.
     *
     * @return the controlPtIsSelected
     */
    public boolean isControlPtSelected() {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow.getCtrlPt().isSelected()) {
                return true;
            }
        }
        return false;
    }

    /**
     * @return the flowNodeDensity
     */
    public FlowNodeDensity getFlowNodeDensity() {
        return flowNodeDensity;
    }

    /**
     * @param flowNodeDensity the flowNodeDensity to set
     */
    public void setFlowNodeDensity(FlowNodeDensity flowNodeDensity) {
        this.flowNodeDensity = flowNodeDensity;
    }

    /**
     * @return the maxFlowStrokeWidthPx
     */
    public double getMaxFlowStrokeWidthPixel() {
        return maxFlowStrokeWidthPx;
    }

    /**
     * @param maxFlowStrokeWidthPixel the maxFlowStrokeWidthPx to set
     */
    public void setMaxFlowStrokeWidthPixel(double maxFlowStrokeWidthPixel) {
        this.maxFlowStrokeWidthPx = maxFlowStrokeWidthPixel;
    }

    /**
     * Maximizes maxFlowStrokeWidthPixel such that no flow is wider than the
     * diameter of its connected nodes.
     *
     * @param maxFlowWidthPx the largest flow width cannot be wider than this
     * width in pixels.
     * @return true if maxFlowStrokeWidthPx changed
     */
    public boolean adjustMaxFlowStrokeWidthToNodeSize(double maxFlowWidthPx) {
        double maxScaleFactor = Double.MAX_VALUE;
        Iterator<Flow> flowIterator = flowIterator();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            double value = Math.abs(flow.getValue());

            // disregard flows with zero value
            if (value == 0) {
                continue;
            }

            // compute line widths fitting the size of the two connected nodes
            double w1 = 2d * getNodeRadiusPx(flow.getStartPt());
            double w2 = 2d * getNodeRadiusPx(flow.getEndPt());

            // disregard if both nodes have 0 values
            if (w1 == 0 && w2 == 0) {
                continue;
            }
            double flowWidth;
            if (w1 == 0) {
                flowWidth = w2;
            } else if (w2 == 0) {
                flowWidth = w1;
            } else {
                flowWidth = Math.min(w1, w2);
            }
            flowWidth += nodeStrokeWidthPx;
            double scaleFactor = flowWidth / value;
            if (scaleFactor < maxScaleFactor) {
                maxScaleFactor = scaleFactor;
            }
        }
        if (maxScaleFactor < Double.MAX_VALUE) {
            double newMaxFlowWidth = Math.min(maxFlowWidthPx, maxScaleFactor * getMaxFlowValue());
            boolean changed = newMaxFlowWidth != maxFlowStrokeWidthPx;
            maxFlowStrokeWidthPx = newMaxFlowWidth;
            return changed;
        } else {
            return false;
        }
    }

    /**
     * @return the maxNodeSizePx
     */
    public double getMaxNodeSizePx() {
        return maxNodeSizePx;
    }

    /**
     * @param maxNodeSizePx the maxNodeSizePx to set
     */
    public void setMaxNodeSizePx(double maxNodeSizePx) {
        this.maxNodeSizePx = maxNodeSizePx;
    }

    /**
     * @return the nodeStrokeWidthPx
     */
    public float getNodeStrokeWidthPx() {
        return nodeStrokeWidthPx;
    }

    /**
     * @param nodeStrokeWidthPx the nodeStrokeWidthPx to set
     */
    public void setNodeStrokeWidthPx(float nodeStrokeWidthPx) {
        this.nodeStrokeWidthPx = nodeStrokeWidthPx;
    }

    /**
     * Returns the stroke width in pixels for a flow value.
     *
     * @param value the value
     * @return width in pixels
     */
    protected double getFlowWidthPx(double value) {
        double flowWidthScaleFactor = getMaxFlowStrokeWidthPixel() / getMaxFlowValue();
        return Math.abs(value) * flowWidthScaleFactor;
    }

    /**
     * Returns the stroke width in pixels for a flow. Includes
     * parallelFlowsGapPx if the flow is a FlowPair.
     *
     * @param flow the flow
     * @return width in pixels
     */
    public double getFlowWidthPx(Flow flow) {
        double value = flow.getValue();
        double flowWidthPx = getFlowWidthPx(value);
        if (flow instanceof FlowPair) {
            flowWidthPx += parallelFlowsGapPx;
        }
        return flowWidthPx;
    }

    /**
     * Returns a the flow value between 0 and 1 relative to the minimum and
     * maximum values of all flows.
     *
     * @param flow the flow for which a relative value is needed.
     * @return relative flow value between 0 and 1. Returns 1 if all flows have
     * the same value.
     */
    public double getRelativeFlowValue(Flow flow) {
        double min = getMinFlowValue();
        double max = getMaxFlowValue();
        double dif = max - min;
        return dif < 1e-12 ? 1 : (flow.getValue() - min) / dif;
    }

    /**
     * Returns the color for drawing flows.
     *
     * @param flow the flow for which a color is needed
     * @return the color of flows
     */
    public Color getFlowColor(Flow flow) {
        double w = getRelativeFlowValue(flow);
        return ColorUtils.blend(minFlowColor, maxFlowColor, w);
    }

    /**
     * @return the angularDistributionWeight
     */
    public double getAngularDistributionWeight() {
        return angularDistributionWeight;
    }

    /**
     * @param angularDistributionWeight the angularDistributionWeight to set
     */
    public void setAngularDistributionWeight(double angularDistributionWeight) {
        this.angularDistributionWeight = angularDistributionWeight;
    }

    /**
     * @return the clipFlowEnds
     */
    public boolean isClipFlowEnds() {
        return clipFlowEnds;
    }

    /**
     * @param clipFlowEnds the clipFlowEnds to set
     */
    public void setClipFlowEnds(boolean clipFlowEnds) {
        this.clipFlowEnds = clipFlowEnds;
    }

    /**
     * @return the clipFlowStarts
     */
    public boolean isClipFlowStarts() {
        return clipFlowStarts;
    }

    /**
     * @param clipFlowStarts the clipFlowStarts to set
     */
    public void setClipFlowStarts(boolean clipFlowStarts) {
        this.clipFlowStarts = clipFlowStarts;
    }

    /**
     * @return the drawInlineArrows
     */
    public boolean isDrawInlineArrows() {
        return drawInlineArrows;
    }

    /**
     * @param drawInlineArrows the drawInlineArrows to set
     */
    public void setDrawInlineArrows(boolean drawInlineArrows) {
        this.drawInlineArrows = drawInlineArrows;
    }

    /**
     * Returns a flow with clipped start and end segments. Clipping takes node
     * dimensions, required distances to start and end points, and masking areas
     * into account. If nothing is clipped, the passed flow is returned
     * unaltered.
     *
     * @param flow flow to clip
     * @param clipArrowhead if true, the flow line is clipped to make space for
     * an arrowhead
     * @param forceClipNodes if true, the start and end of the flow are clipped
     * by at least the radius of the respective node. This is useful for overlap
     * test, for example. If false, the start and end of the flow are clipped
     * only by the radius of the respective node if there is a gap required
     * between the node and the end of the flow. This is useful for drawing a
     * flow, where the start and end of the flow are hidden by an overlaying
     * node symbol.
     * @return a flow with clipped start and end segments. If nothing is
     * clipped, the passed flow is returned unaltered.
     */
    public Flow clipFlow(Flow flow, boolean clipArrowhead, boolean forceClipNodes) {

        LineString lineString = null;
        if (flow.getEndClipArea() != null || flow.getStartClipArea() != null) {
            // FIXME better use irregular intervals
            lineString = JTSUtils.pointsToLineString(flow.regularIntervals(segmentLength()));
        }

        // clipping radius for start node
        double startNodeClipR = 0;

        // clip the start if there must be a gap between the start of 
        // the flow line and the start node symbol.
        // distance between start of flow and start node
        if (forceClipNodes || flowDistanceFromStartPointPx > 0) {
            // Compute the radius of the start node (add half stroke width)
            double startNodeRadiusPx = getNodeStrokeWidthPx() / 2 + getNodeRadiusPx(flow.getStartPt());
            startNodeClipR = (flowDistanceFromStartPointPx + startNodeRadiusPx) / getReferenceMapScale();
        }

        // clipping radius for start mask area
        double startMaskClipR = 0;
        if (flow.getStartClipArea() != null) {
            startMaskClipR = flow.maskClippingRadius(lineString, true);
        }

        // start and end clipping radius
        double startR = Math.max(startNodeClipR, startMaskClipR);
        double endR = endClipRadius(flow, clipArrowhead, lineString, forceClipNodes);

        // cut off the end piece
        double endT = flow.getIntersectionTWithCircleAroundEndPoint(endR);
        if (endT < 1) {
            flow = flow.split(endT)[0];
        }

        // cut off the start piece
        double startT = flow.getIntersectionTWithCircleAroundStartPoint(startR);
        if (startT > 0) {
            flow = flow.split(startT)[1];
        }

        return flow;
    }

    /**
     * Computes clipping radius around end node for a flow. The clipping radius
     * can include the arrowhead, the end node radius, a gap between the end of
     * the line (or the tip of the arrowhead) and the end node, and the end mask
     * clip area.
     *
     * @param flow flow compute radius for
     * @param clipArrowhead if true, the circle radius includes the arrowhead
     * (if arrowheads are drawn).
     * @param lineString the geometry of this flow converted to straight line
     * segments.
     * @param clipEndNode if true, the returned circle radius includes the end
     * node. The end node and a gap to the end node are also included when there
     * is a gap greater than 0.
     * @return radius of circle around end node
     */
    public double endClipRadius(Flow flow, boolean clipArrowhead,
            LineString lineString, boolean clipEndNode) {
        // clipping radius for end node
        double endNodeClipRadius = 0;
        if (clipArrowhead && isDrawArrowheads()) {
            // Compute radius of clipping circle around end point without taking
            // the arrow into account. This is a recursive call with 
            // clipArrowhead flag set to false.
            double arrowTipClipRadius = endClipRadius(flow, false, lineString, clipEndNode);

            // Create an arrowhead
            Arrow arrow = flow.getArrow(this, arrowTipClipRadius);

            // get clip radius including arrowhead
            endNodeClipRadius = arrow.getClipRadius();
        } else if (clipEndNode || getFlowDistanceFromEndPointPixel() > 0 || isDrawArrowheads()) {
            // clip the end if there must be a gap between the end of the 
            // flow line and the end node symbol.
            double gapDistanceToEndNodesPx = getFlowDistanceFromEndPointPixel();
            // Compute the radius of the end node (add stroke width / 2 to radius)
            double endNodeRadiusPx = getNodeStrokeWidthPx() / 2 + getNodeRadiusPx(flow.getEndPt());
            endNodeClipRadius = (gapDistanceToEndNodesPx + endNodeRadiusPx) / getReferenceMapScale();
        }

        // clipping radius for end mask area
        double endMaskClipRadius = 0;
        if (flow.getEndClipArea() != null) {
            if (lineString == null && (flow.getEndClipArea() != null || flow.getStartClipArea() != null)) {
                // FIXME better use irregular intervals
                lineString = JTSUtils.pointsToLineString(flow.regularIntervals(segmentLength()));
            }
            endMaskClipRadius = flow.maskClippingRadius(lineString, false);
        }

        // end clipping radius
        return Math.max(endNodeClipRadius, endMaskClipRadius);
    }

    /**
     * Get a node's radius in pixels at the reference scale.
     *
     * @param node
     * @return radius in pixels
     */
    public double getNodeRadiusPx(Point node) {
        double area = Math.abs(node.getValue() * getNodeSizeScaleFactor());
        return Math.sqrt(area / Math.PI);
    }

    /**
     * @return the nodeTolerancePx
     */
    public int getMinObstacleDistPx() {
        return minObstacleDistPx;
    }

    /**
     * @param minObstacleDistPx the nodeTolerancePx to set
     */
    public void setMinObstacleDistPx(int minObstacleDistPx) {
        this.minObstacleDistPx = minObstacleDistPx;
    }

    /**
     * Conversion factor between pixels and world coordinates. world = px /
     * getReferenceMapScale()
     *
     * @return the referenceMapScale
     */
    public final double getReferenceMapScale() {
        return referenceMapScale;
    }

    /**
     * @param referenceMapScale the referenceMapScale to set
     */
    public void setReferenceMapScale(double referenceMapScale) {
        this.referenceMapScale = referenceMapScale;
    }

    /**
     * @return the minFlowColor
     */
    public Color getMinFlowColor() {
        return minFlowColor;
    }

    /**
     * @param minFlowColor the minFlowColor to set
     */
    public void setMinFlowColor(Color minFlowColor) {
        this.minFlowColor = minFlowColor;
    }

    /**
     * @return the maxFlowColor
     */
    public Color getMaxFlowColor() {
        return maxFlowColor;
    }

    /**
     * @param maxFlowColor the maxFlowColor to set
     */
    public void setMaxFlowColor(Color maxFlowColor) {
        this.maxFlowColor = maxFlowColor;
    }

    /**
     * @return the resolveIntersectionsForSiblings
     */
    public boolean isResolveIntersectionsForSiblings() {
        return resolveIntersectionsForSiblings;
    }

    /**
     * @param resolveIntersectionsForSiblings the
     * resolveIntersectionsForSiblings to set
     */
    public void setResolveIntersectionsForSiblings(boolean resolveIntersectionsForSiblings) {
        this.resolveIntersectionsForSiblings = resolveIntersectionsForSiblings;
    }

    /**
     * @return the moveFlowsOverlappingObstacles
     */
    public boolean isMoveFlowsOverlappingObstacles() {
        return moveFlowsOverlappingObstacles;
    }

    /**
     * @param moveFlowsOverlappingObstacles the moveFlowsOverlappingObstacles to
     * set
     */
    public void setMoveFlowsOverlappingObstacles(boolean moveFlowsOverlappingObstacles) {
        this.moveFlowsOverlappingObstacles = moveFlowsOverlappingObstacles;
    }

    /**
     * @return the nbrIterations
     */
    public int getNbrIterations() {
        return nbrIterations;
    }

    /**
     * @param nbrIterations the nbrIterations to set
     */
    public void setNbrIterations(int nbrIterations) {
        this.nbrIterations = nbrIterations;
    }

    /**
     * @return the nodeStrokeColor
     */
    public Color getNodeStrokeColor() {
        return nodeStrokeColor;
    }

    /**
     * @param nodeStrokeColor the nodeStrokeColor to set
     */
    public void setNodeStrokeColor(Color nodeStrokeColor) {
        this.nodeStrokeColor = nodeStrokeColor;
    }

    /**
     * @return the nodeFillColor
     */
    public Color getNodeFillColor() {
        return nodeFillColor;
    }

    /**
     * Set fill color.
     *
     * @param nodeFillColor The new fill color.
     */
    public void setNodeFillColor(Color nodeFillColor) {
        this.nodeFillColor = nodeFillColor;
    }

    private double sumFlowValues(Collection<Flow> flows) {
        double totalValue = 0;
        for (Flow flow : flows) {
            totalValue += flow.getValue();
        }
        return totalValue;
    }
    
    /**
     * Sum values of flows between same start and end nodes.
     */
    public void convertToTotalFlows() {
        ArrayList<Flow> totalFlows = new ArrayList<>();
        List<Point> nodes = getNodes();
        for (int i = 0; i < nodes.size(); i++) {
            Point node1 = nodes.get(i);
            for (int j = i + 1; j < nodes.size(); j++) {
                Point node2 = nodes.get(j);
                Set<Flow> flows = graph.getAllFlowsBetweenNodes(node1, node2);
                if (flows.isEmpty() == false) {
                    totalFlows.add(new Flow(node1, node2, sumFlowValues(flows)));
                }
            }
        }
        setFlows(totalFlows);
    }

    /**
     * Computes net flows by computing the difference between opposing flows
     * between the same nodes. Net values are always positive. If the two
     * opposing flows have the same value, no net flow is created.
     *
     * If there are multiple flows with the same direction between two nodes,
     * the total value of all flows pointing in the same direction is first
     * computed.
     */
    public void convertToNetFlows() {
        ArrayList<Flow> netFlows = new ArrayList<>();
        List<Point> nodes = getNodes();
        for (int i = 0; i < nodes.size(); i++) {
            Point node1 = nodes.get(i);
            for (int j = i + 1; j < nodes.size(); j++) {
                Point node2 = nodes.get(j);
                double netSum = graph.netSum(node1, node2);
                if (netSum > 0) {
                    netFlows.add(new Flow(node1, node2, netSum));
                } else if (netSum < 0) {
                    netFlows.add(new Flow(node2, node1, Math.abs(netSum)));
                }
                // if netSum equals 0, no flow is created
            }
        }
        setFlows(netFlows);
    }

    /**
     * @return the name
     */
    public String getName() {
        return name;
    }

    /**
     * @param name the name to set
     */
    public void setName(String name) {
        this.name = name;
    }

    /**
     * @return the parallelFlowsGapPx
     */
    public double getParallelFlowsGapPx() {
        return parallelFlowsGapPx;
    }

    /**
     * @param parallelFlowsGapPx the parallelFlowsGapPx to set
     */
    public void setParallelFlowsGapPx(double parallelFlowsGapPx) {
        this.parallelFlowsGapPx = parallelFlowsGapPx;
    }

    /**
     * Returns whether opposing flows between the same start point and end point
     * are to be shown as two parallel flows.
     *
     * @return the bidirectionalFlowsParallel if true, opposing flows are shown
     * as two parallel flows.
     */
    public boolean isBidirectionalFlowsParallel() {
        return graph.isBidirectionalFlowsParallel();
    }

    /**
     * Set whether opposing flows between the same start point and end point are
     * to be shown as two parallel flows.
     *
     * @param bidirectionalFlowsParallel if true, opposing flows are shown as
     * two parallel flows.
     */
    public void setBidirectionalFlowsParallel(boolean bidirectionalFlowsParallel) {
        graph.setBidirectionalFlowsParallel(bidirectionalFlowsParallel, this);
    }

}
