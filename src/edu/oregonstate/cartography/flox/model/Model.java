package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryCollectionIterator;
import com.vividsolutions.jts.geom.GeometryFactory;
import edu.oregonstate.cartography.utils.ColorUtils;
import edu.oregonstate.cartography.utils.GeometryUtils;
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
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlTransient;
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
    
    // FIXME
    public boolean limitNodesRepulsionToBandHack = false;

    // FIXME
    public boolean liveDrawing = false;

    /**
     * Density of points along flows.
     */
    public enum FlowNodeDensity {

        // de Casteljau tolerance in pixels relative to current map reference scale.
        LOW(2.5d),
        MEDIUM(1d),
        HIGH(0.5);

        private final double deCasteljauTolerance;

        /**
         * Multiply deCasteljauTolerance by DE_CASTELJAU_TO_LINE_SEGMENT_LENGTH
         * to compute target line segment length when converting from Bezier
         * curves to straight line segments.
         */
        public static final double DE_CASTELJAU_TO_LINE_SEGMENT_LENGTH = 20;

        FlowNodeDensity(double deCasteljauTolerance) {
            this.deCasteljauTolerance = deCasteljauTolerance;
        }
    }

    /**
     * Determines the maximum number of intermediate nodes per flow.
     */
    private FlowNodeDensity flowNodeDensity = FlowNodeDensity.MEDIUM;

    /**
     * Helper class for pairs of intersecting flows.
     */
    public static class IntersectingFlowPair {

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
    }

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
     * Weight of angular distribution force. Currently modified by a GUI slider.
     */
    private double angularDistributionWeight = 0.5;

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
     * If this is true, control points of flows are prevented from movingm
     * outside of its flow's rangebox.
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
    private double canvasPadding = 0.5;

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
    private double arrowLengthScaleFactor = 1.6;

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
     * Maximum allowed node radius in pixels.
     */
    private double maxNodeSizePx = 10;

    /**
     * Stroke width for drawing node circles. In pixels.
     */
    private float nodeStrokeWidthPx = 2;

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
    private boolean clipFlowEnds = false;

    /**
     * Clip the beginnings of flows. This flags is only used to update the GUI.
     */
    private boolean clipFlowStarts = false;

    /**
     * A geometry (collection) used for clipping start or end of flows.
     */
    // FIXME should not be transient?
    @XmlTransient
    private Geometry clipAreas;

    /**
     * Buffer width for start clip areas.
     */
    private double startClipAreaBufferDistance = 0;

    /**
     * Buffer width for end clip areas.
     */
    private double endClipAreaBufferDistance = 0;

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
    @XmlTransient
    private Map map = new Map();

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

    public void copyTransientFields(Model destination) {
        // map layers are not currently serialized
        destination.map = map;
        // FIXME should not be transient?
        destination.clipAreas = clipAreas;
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
     * Replace the graph of this model with a reference to the graph of the
     * passed model. Only a shallow copy is made.
     *
     * @param model copy graph reference from this model
     */
    public void assignGraph(Model model) {
        this.graph = model.graph;
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
        ArrayList<Point> nodes = getNodes();
        for (Point node : nodes) {
            if (graph.getFlowsForNode(node).isEmpty()) {
                node.setSelected(true);
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
        ArrayList<Point> nodes = getNodes();
        for (Point node : nodes) {
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
            graph.removeEdge(flow);
        }

        // add merged flows
        for (Flow flow : flowsToMerg) {
            // ignore collapsing flows
            if (flow.getStartPt().isSelected() && flow.getEndPt().isSelected()) {
                continue;
            }
            // test whether a flow has been added between the merged node and the opposite node of this flow 
            Flow existingFlow = graph.getFlowBetweenNodes(flow.getStartPt(), mergedNode);
            if (existingFlow == null) {
                existingFlow = graph.getFlowBetweenNodes(flow.getEndPt(), mergedNode);
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
        graph.removeEdge(flow);
    }

    public void changeToBidirectionalFlows() {
        ArrayList<BidirectionalFlow> flowsToAdd = new ArrayList<>();
        HashSet<Flow> flowsToRemove = new HashSet<>();
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            Flow flow2 = graph.getOpposingFlow(flow);
            if (flow2 != null) {
                if (flowsToRemove.contains(flow) == false) {
                    flowsToAdd.add(new BidirectionalFlow(flow, flow2));
                    flowsToRemove.add(flow);
                    flowsToRemove.add(flow2);
                }
            }
        }
        System.out.println();
        System.out.println("changeToBidirectionalFlows");
        System.out.println("# initial flows: " + graph.getNbrFlows());
        System.out.println("# opposing flows to remove: " + flowsToRemove.size());
        System.out.println("# bidirectional flows to add: " + flowsToAdd.size());

        for (Flow flow : flowsToRemove) {
            graph.removeEdge(flow);
        }
        for (BidirectionalFlow bidirectionalFlow : flowsToAdd) {
            graph.addFlow(bidirectionalFlow);
        }
        System.out.println("# final flows: " + graph.getNbrFlows());
    }

    public void changeToUnidirectionalFlows() {
        ArrayList<Flow> flowsToAdd = new ArrayList<>();
        ArrayList<BidirectionalFlow> flowsToRemove = new ArrayList<>();
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            if (flow instanceof BidirectionalFlow) {
                BidirectionalFlow biFlow = (BidirectionalFlow) flow;
                flowsToRemove.add(biFlow);
                flowsToAdd.add(biFlow.createFlow1());
                flowsToAdd.add(biFlow.createFlow2());
            }
        }
        System.out.println();
        System.out.println("changeToUnidirectionalFlows");
        System.out.println("# initial flows: " + graph.getNbrFlows());
        System.out.println("# bidirectional flows to remove: " + flowsToRemove.size());
        System.out.println("# opposing flows to add: " + flowsToAdd.size());

        for (BidirectionalFlow flow : flowsToRemove) {
            graph.removeEdge(flow);
        }
        for (Flow bidirectionalFlow : flowsToAdd) {
            graph.addFlow(bidirectionalFlow);
        }
        System.out.println("# final flows: " + graph.getNbrFlows());
    }

    public void reverseSelectedFlows() {
        ArrayList<Flow> flows = getSelectedFlows();
        for (Flow flow : flows) {
            graph.removeEdge(flow);
            flow.reverseFlow();
            graph.addFlow(flow);
        }
    }

    /**
     * Delete a node.
     *
     * @param node The node to delete.
     */
    public void deleteNode(Point node) {
        graph.removeVertex(node);
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
        flows.stream().forEach((flow) -> {
            addFlow(flow);
        });
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
     * Finds the clip areas for the start of flows. First buffers the clip area
     * geometry, then finds containing geometry for the start point of each
     * flow, then assigns a start clip area to the flows.
     */
    public void updateStartClipAreas() {
        if (clipAreas == null) {
            return;
        }

        Iterator<Flow> flowIterator = flowIterator();
        GeometryFactory f = new GeometryFactory();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            Geometry startClipArea = findContainingGeometry(clipAreas, flow.getStartPt(), f);
            if (startClipArea != null) {
                startClipArea = startClipArea.buffer(-startClipAreaBufferDistance);
            }
            flow.setStartClipArea(startClipArea);
        }
    }

    /**
     * Finds the clip areas for the the end of flows. First buffers the clip
     * area geometry, then finds containing geometry for the end point of each
     * flow, then assigns an end clip area to each flow.
     */
    public void updateEndClipAreas() {
        if (clipAreas == null) {
            return;
        }

        Iterator<Flow> flowIterator = flowIterator();
        GeometryFactory f = new GeometryFactory();
        while (flowIterator.hasNext()) {
            Flow flow = flowIterator.next();
            Geometry endClipArea = findContainingGeometry(clipAreas, flow.getEndPt(), f);
            if (endClipArea != null) {
                endClipArea = endClipArea.buffer(-endClipAreaBufferDistance);
            }
            flow.setEndClipArea(endClipArea);
        }
    }

    /**
     * Set the start and end clip areas for flows.
     *
     * @param clipAreas
     */
    public void setClipAreas(Geometry clipAreas) {
        this.clipAreas = clipAreas;
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
    public double getStartClipAreaBufferDistance() {
        return startClipAreaBufferDistance;
    }

    /**
     * @param startClipAreaBufferDistance the startClipAreaBufferDistance to set
     */
    public void setStartClipAreaBufferDistance(double startClipAreaBufferDistance) {
        this.startClipAreaBufferDistance = startClipAreaBufferDistance;
        updateStartClipAreas();
    }

    /**
     * @return the endClipAreaBufferDistance
     */
    public double getEndClipAreaBufferDistance() {
        return endClipAreaBufferDistance;
    }

    /**
     * @param endClipAreaBufferDistance the endClipAreaBufferDistance to set
     */
    public void setEndClipAreaBufferDistance(double endClipAreaBufferDistance) {
        this.endClipAreaBufferDistance = endClipAreaBufferDistance;
        updateEndClipAreas();
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
     * @return
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
     * Returns an iterator for all flows sorted by their value.
     *
     * @param increasing sort by increasing (true) or decreasing (false) values
     * @return iterator for sorted flows
     */
    public Iterator<Flow> sortedFlowIterator(boolean increasing) {
        return graph.getOrderedFlows(increasing).iterator();
    }

    public ArrayList<Flow> getFlows() {
        return graph.getFlows();
    }

    /**
     * Sort a list of flows by flow values.
     *
     * @param flows flows to order
     * @param increasing increasing or decreasing sort
     */
    public static void sortFlows(List<Flow> flows, boolean increasing) {
        java.util.Collections.sort(flows, (Flow flow1, Flow flow2) -> {
            if (increasing) {
                return Double.compare(flow1.getValue(), flow2.getValue());
            } else {
                return Double.compare(flow2.getValue(), flow1.getValue());
            }
        });
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
     * Returns all nodes in the graph. FIXME getNodes should not be needed, as
     * an extra ArrayList is created with this call. An iterator should be used
     * instead. However, some applications (e.g. selection tool) require a
     * reverse iteration.
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
     * Returns an empirical tolerance value for the De Casteljau algorithm,
     * which converts a Bezier curve to straight line segments. The returned
     * value is relative to the current reference map scale.
     *
     * @return de Casteljau tolerance in world units.
     */
    public double getDeCasteljauTolerance() {
        return flowNodeDensity.deCasteljauTolerance / getReferenceMapScale();
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
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            iterator.next().setSelected(select);
        }

        Iterator nodes = nodeIterator();
        while (nodes.hasNext()) {
            Point node = (Point) nodes.next();
            node.setSelected(select);
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
        assert (antiTorsionWeight >= 0 && antiTorsionWeight <= 1);
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
     * Returns the bounding box containing all start and end points.
     *
     * @return the bounding box
     */
    public Rectangle2D getNodesBoundingBox() {
        Iterator<Point> iterator = nodeIterator();
        if (iterator.hasNext() == false) {
            return null;
        }
        Point pt = iterator.next();
        Rectangle2D.Double bb = new Rectangle2D.Double(pt.x, pt.y, 0, 0);
        while (iterator.hasNext()) {
            pt = iterator.next();
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
     * Returns the stroke width in pixels of a flow based on its value.
     *
     * @param flow the flow
     * @return width in pixels
     */
    public double getFlowWidthPx(Flow flow) {
        double flowWidthScaleFactor = getMaxFlowStrokeWidthPixel() / getMaxFlowValue();
        return Math.abs(flow.getValue()) * flowWidthScaleFactor;
    }

    /**
     * Returns a the flow value between 0 and 1 relative to the minimum and
     * maximum values of all flows.
     *
     * @param flow the flow for which a relative value is needed.
     * @return
     */
    public double getRelativeFlowValue(Flow flow) {
        return (flow.getValue() - getMinFlowValue())
                / (getMaxFlowValue() - getMinFlowValue());
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
     * @return a flow with clipped start and end segments. If nothing is
     * clipped, the passed flow is returned unaltered.
     */
    public Flow clipFlow(Flow flow, boolean clipArrowhead) {
        double[] clipRadii = flow.clipRadii(this, clipArrowhead);

        // cut off the end piece
        double endT = flow.getIntersectionTWithCircleAroundEndPoint(clipRadii[1]);
        if (endT < 1) {
            flow = flow.split(endT)[0];
        }

        // cut off the start piece
        double startT = flow.getIntersectionTWithCircleAroundStartPoint(clipRadii[0]);
        if (startT > 0) {
            flow = flow.split(startT)[1];
        }

        return flow;
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

    public void computeArrowheads() {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();

            // Compute radius of clipping circle around end point.
            // Clip the flow with the clipping area and/or a circle around the end node
            double arrowTipClipRadius = flow.endClipRadius(this, false, null);

            // stroke width in world coordinates of the flow based on its value.
            double flowStrokeWidth = getFlowWidthPx(flow) / getReferenceMapScale();

            // Create an arrowhead
            flow.computeArrowhead(this, flowStrokeWidth, arrowTipClipRadius);
        }

//        // TODO adjust the width of arrowheads
//        ArrayList<Point> points = model.getNodes();
//        for (Point point : points) {
//            ArrayList<Flow> incomingFlows = model.getAnticlockwiseOrderedIncomingFlows(point);
//            System.out.println("Number of incoming flows at node " + point + ": " + incomingFlows.size());
//        }
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
}
