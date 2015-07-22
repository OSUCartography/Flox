package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryCollectionIterator;
import com.vividsolutions.jts.geom.GeometryFactory;
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
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
//Defines root element of JAXB XML file
@XmlRootElement

//Every non static, non transient field in a JAXB-bound class will be 
//automatically bound to XML, unless annotated by @XmlTransient
@XmlAccessorType(XmlAccessType.FIELD)

public class Model {

    public enum CurveType {

        CUBIC,
        QUADRATIC
    }

    /**
     * Order of drawing of flows
     */
    public enum FlowOrder {

        INCREASING,
        DECREASING,
        UNORDERED
    }

    public enum FlowNodeDensity {

        LOW,
        MEDIUM,
        HIGH
    }

    /**
     * Graph of edges (BŽzier flows) and nodes (Point)
     */
    @XmlJavaTypeAdapter(GraphSerializer.class)
    private final Graph graph = new Graph();

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
     * control points.
     */
    private double arrowEdgeCtrlLength = 0.5;

    /**
     * Used by the Arrow class to determine the location of the arrow edge
     * control points.
     */
    private double arrowEdgeCtrlWidth = 0.5;

    /**
     * Used by the Arrow class to determine the horizontal postion of the
     * arrow's corners relative to the base.
     */
    private double arrowCornerPosition = 0.0;

    /**
     * Used by the Arrow class to determine the size of the smallest arrowhead.
     */
    private double arrowSizeRatio = 0.1;

    /**
     * Start and end node exert a larger force than points along flow lines.
     */
    private double nodeWeightFactor = 0.0;

    /**
     * Weight for the anti-torsion force.
     */
    private double antiTorsionWeight = .8;

    /**
     * Stiffness factor for peripheral flows.
     */
    private double peripheralStiffnessFactor = 0.5;

    /**
     * spring stiffness of longest flow.
     */
    private double maxFlowLengthSpringConstant = .05;

    /**
     * spring stiffness of zero-length flow.
     */
    private double minFlowLengthSpringConstant = 0.5;

    /**
     * This determines the amount of force that objects far away from the target
     * can apply to the target. The lower the distanceWeightExponent, the more
     * force distant objects are permitted to apply.
     */
    private double distanceWeightExponent = 30.0;

    /**
     * If this is true, control points of flows are prevented from moving 
     * outside of its flow's rangebox. Modified by a GUI checkbox.
     */
    private boolean enforceRangebox = true;

    /**
     * If this is true, control points are prevented from moving outside of the
     * canvas.
     */
    private boolean enforceCanvasRange = true;

    /**
     * If this is true, arrows are drawn onto the end of flows. Modified by a
     * GUI checkbox.
     */
    private boolean drawArrows = false;

    /**
     * Determines the size of the canvas. The minimum canvas size is the bounding
     * box of all flows. This value is used to increases the length and width of 
     * the minimum canvas by (length * this value) and (width * this value).
     * Modified currently by a GUI slider.
     */
    private double canvasPadding = 0.5;

    /**
     * This value determines the width of a flows bounding box. The width of 
     * the bounding box = this value * the length of the flow's baseline.
     * Currently modified by a GUI slider.
     */
    private double flowRangeboxHeight = .5;

    /**
     * Determines the distance (in pixels) a flow line stops before reaching
     * its endpoint. If arrows are being drawn, this is the distance from
     * the tip of the arrow to the outside of the node. If arrows are NOT being
     * drawn, this is the distance from the end of the flow line to the center
     * of the node. Currently modified by a GUI modifiable text field.
     */
    private double flowDistanceFromEndPoint = 0.0d;

    // FIXME
    private boolean controlPtIsSelected = false;

    /**
     * Maximum allowed flow width in pixels. The flow with the highest value
     * will have this width. All other flows are scaled down 
     * relative to this value.
     */
    private double maxFlowStrokeWidth = 20;

    /**
     * Gets the ratio between the maximum permitted flow stroke width and the
     * maximum current flow value.
     * @return maxFlowStrokeWidth/maxFlowValue
     */
    public double getFlowWidthScaleFactor() {
        return getMaxFlowStrokeWidth() / getMaxFlowValue();
    }

    /**
     * Maximum allowed node radius in pixels.
     */
    private double maxNodeSize = 10;

    /**
     * 
     * @return 
     */
    public double getNodeSizeScaleFactor() {

        // Get the area needed to satisfy the radius
        double area = Math.PI * (maxNodeSize * maxNodeSize);

        return area / getMaxNodeValue();
    }

    public double getMaxNodeValue() {
        return graph.getMaxNodeValue();
    }

    /**
     * Flag to indicate when the flow width is locked to the current map scale.
     */
    private boolean flowWidthLocked = false;

    /**
     * The map scale at the time it was locked.
     */
    private double lockedMapScale = 1;

    /**
     * Determines the maximum number of intermediate nodes per flow. This is
     * modified by a comboBox in the GUI (low, medium, high).
     */
    private FlowNodeDensity flowNodeDensity = FlowNodeDensity.MEDIUM;

    // FIXME should not be transient
    @XmlJavaTypeAdapter(RectangleSerializer.class)
    private Rectangle2D canvas;

    /**
     * A reference to the map with layers and geometry.
     */
    @XmlTransient
    private Map map = new Map();

    /**
     * Either work with cubic or quadratic curves
     */
    private CurveType curveType = CurveType.QUADRATIC;

    /**
     * Order of drawing of flows
     */
    private FlowOrder flowOrder = FlowOrder.DECREASING;

    /**
     * A geometry (collection) used for clipping start or end of flows
     */
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

    private double longestFlowLength;

    /**
     * Constructor of the model.
     */
    public Model() {
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
        destination.map = map;
        destination.clipAreas = clipAreas;

        // FIXME should not be transient
        destination.canvas = canvas;
    }

    @Override
    public String toString() {
        StringWriter sw = new StringWriter();
        try {
            marshal(sw);
            return sw.toString();
        } catch (JAXBException ex) {
            Logger.getLogger(Model.class.getName()).log(Level.SEVERE, null, ex);
            return "could not marshal model";
        }
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
        graph.addFlow(flow);
    }

    public void deleteFlow(Flow flow) {
        graph.removeEdge(flow);
    }

    public void deleteNode(Point node) {
        graph.removeVertex(node);
    }

    /**
     * Removes all selected nodes and flows from the graph.
     *
     * @return The number of flows and nodes that were deleted.
     */
    public int deleteSelectedFlowsAndNodes() {
        ArrayList<Flow> flowsToRemove = new ArrayList<>();
        ArrayList<Point> nodesToRemove = new ArrayList<>();

        Iterator flows = flowIterator();
        while (flows.hasNext()) {
            Flow flow = (Flow) flows.next();
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
        clearFlows();
        flows.stream().forEach((flow) -> {
            addFlow(flow);
        });
        setLongestFlowLength();
    }

    /**
     * Remove all flows.
     */
    public void clearFlows() {
        graph.clearFlows();
    }

    /**
     * Finds an OGC simple feature that contains a point.
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
     * flow.
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
                startClipArea = startClipArea.buffer(startClipAreaBufferDistance);
            }
            flow.setStartClipArea(startClipArea);
        }
    }

    /**
     * Finds the clip areas for the the end of flows. First buffers the clip
     * area geometry, then finds containing geometry for the end point of each
     * flow.
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
                endClipArea = endClipArea.buffer(endClipAreaBufferDistance);
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
     * Removes the end clip areas from all flows
     */
    public void removeEndClipAreasFromFlows() {
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            iterator.next().setEndClipArea(null);
        }
    }

    /**
     * Removes the start clip areas from all flows
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
     * Returns true if the model contains at least one flow.
     *
     * @return True if the model has at least one flow.
     */
    public boolean hasFlows() {
        return !graph.edgeSet().isEmpty();
    }

    /**
     * Returns all flows in the graph. The flows are not ordered.
     *
     * @return All flows.
     */
    public ArrayList<Flow> getFlows() {
        return graph.getFlows();
    }

    /**
     * Get an ordered list of all flows.
     *
     * @param increasing Increasing or decreasing list.
     * @return A list with all ordered flows.
     */
    public ArrayList<Flow> getOrderedFlows(boolean increasing) {
        return graph.getOrderedFlows(increasing);
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
     * Returns true if the model contains at least one node.
     *
     * @return True if the model has at least one node.
     */
    public boolean hasNodes() {
        return !graph.vertexSet().isEmpty();
    }

    /**
     * Returns all nodes in the graph.
     *
     * @return All nodes.
     */
    public ArrayList<Point> getNodes() {
        return graph.getNodes();
    }

    /**
     * Returns the maximum flow value.
     *
     * @return The maximum flow value.
     */
    public double getMaxFlowValue() {
        return graph.getMaxFlowValue();
    }

    /**
     * Returns the length of the longest flow base line.
     *
     * @return The length of the longest base line.
     */
    public double getLongestFlowLength() {
        return longestFlowLength;
    }

    public void setLongestFlowLength() {
        double maxLength = 0;
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            double l = flow.getBaselineLength();
            if (l > maxLength) {
                maxLength = l;
            }
        }
        longestFlowLength = maxLength;
    }

    public double getShortestFlowLength() {
        double minLength = Double.POSITIVE_INFINITY;
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            double l = flow.getBaselineLength();
            if (l < minLength) {
                minLength = l;
            }
        }

        return minLength == Double.POSITIVE_INFINITY ? null : minLength;
    }

    public double getDeCasteljauTolerance() {

        double maxFlowNodes;
        if (flowNodeDensity == FlowNodeDensity.LOW) {
            maxFlowNodes = 10;
        } else if (flowNodeDensity == FlowNodeDensity.MEDIUM) {
            maxFlowNodes = 25;
        } else { // flowNodeDensity == FlowNodeDensity.HIGH
            maxFlowNodes = 40;
        }

        double tol = getShortestFlowLength() / maxFlowNodes;
        if (longestFlowLength / tol <= maxFlowNodes) {
            return tol;
        } else {
            tol = longestFlowLength / maxFlowNodes;
            return tol;
        }
        
    }

    public double setLargestFlowValue() {
        double maxValue = 0;
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            double val = flow.getValue();
            if (val < maxValue) {
                maxValue = val;
            }
        }
        return maxValue;
    }

    public boolean isFlowSelected() {
        Iterator flows = flowIterator();
        while (flows.hasNext()) {
            Flow flow = (Flow) flows.next();
            if (flow.isSelected()) {
                return true;
            }
        }
        return false;
    }

    public boolean isLockedFlowSelected() {
        Iterator flows = flowIterator();
        while (flows.hasNext()) {
            Flow flow = (Flow) flows.next();
            if (flow.isSelected() && flow.isLocked()) {
                return true;
            }
        }
        return false;
    }

    public boolean isUnlockedFlowSelected() {
        Iterator flows = flowIterator();
        while (flows.hasNext()) {
            Flow flow = (Flow) flows.next();
            if (flow.isSelected() && !flow.isLocked()) {
                return true;
            }
        }
        return false;
    }

    public ArrayList<Flow> getSelectedFlows() {
        ArrayList<Flow> selectedFlows = new ArrayList();
        Iterator flows = flowIterator();
        while (flows.hasNext()) {
            Flow flow = (Flow) flows.next();
            if (flow.isSelected()) {
                selectedFlows.add(flow);

            }
        }
        return selectedFlows;
    }

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
        Iterator flows = flowIterator();
        while (flows.hasNext()) {
            Flow flow = (Flow) flows.next();
            flow.setSelected(select);
        }

        Iterator nodes = nodeIterator();
        while (nodes.hasNext()) {
            Point node = (Point) nodes.next();
            node.setSelected(select);
        }
    }

    /**
     * Set the lock for all selected flows.
     * @param lock The new lock state.
     */
    public void setLockOfSelectedFlows(boolean lock) {
        Iterator flows = flowIterator();
        while (flows.hasNext()) {
            Flow flow = (Flow) flows.next();
            if (flow.isSelected()) {
                flow.setLocked(lock);
            }
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
     * @return the curveType
     */
    public CurveType getCurveType() {
        return curveType;
    }

    /**
     * @param curveType the curveType to set
     */
    public void setCurveType(CurveType curveType) {
        this.curveType = curveType;
    }

    /**
     * @return the nodeWeightFactor
     */
    public double getNodeWeightFactor() {
        return nodeWeightFactor;
    }

    /**
     * @param nodeWeightFactor the nodeWeightFactor to set
     */
    public void setNodeWeightFactor(double nodeWeightFactor) {
        this.nodeWeightFactor = nodeWeightFactor;
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
    public double getDistanceWeightExponent() {
        return distanceWeightExponent;
    }

    /**
     * Sets the distanceWeightExponent. This is currently set by the slider bar
     * in the GUI.
     *
     * @param idwExponent The distanceWeightExponent to set
     */
    public void setDistanceWeightExponent(double idwExponent) {
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
     * @return the canvas
     */
    public Rectangle2D getCanvas() {
        return canvas;
    }

    public void setCanvas(Rectangle2D canvas) {
        this.canvas = canvas;
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
     * @return the flowOrder
     */
    public FlowOrder getFlowOrder() {
        return flowOrder;
    }

    /**
     * @param flowOrder the flowOrder to set
     */
    public void setFlowOrder(FlowOrder flowOrder) {
        this.flowOrder = flowOrder;
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
    public double getFlowDistanceFromEndPoint() {
        return flowDistanceFromEndPoint;
    }

    /**
     * @param flowArrowEndPointRadius the flowArrowEndPointRadius to set
     */
    public void setFlowDistanceFromEndPoint(double flowArrowEndPointRadius) {
        this.flowDistanceFromEndPoint = flowArrowEndPointRadius;
    }

    /**
     * @return the addArrows
     */
    public boolean isDrawArrows() {
        return drawArrows;
    }

    /**
     * @param addArrows the addArrows to set
     */
    public void setAddArrows(boolean addArrows) {
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

    /**
     * @param arrowSizeRatio the arrowSizeRatio to set
     */
    public void setArrowSizeRatio(double arrowSizeRatio) {
        this.arrowSizeRatio = arrowSizeRatio;
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
     * @return the controlPtIsSelected
     */
    public boolean isControlPtIsSelected() {
        return controlPtIsSelected;
    }

    /**
     * @param controlPtIsSelected the controlPtIsSelected to set
     */
    public void setControlPtIsSelected(boolean controlPtIsSelected) {
        this.controlPtIsSelected = controlPtIsSelected;
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
     * @return the flowWidthLocked
     */
    public boolean isFlowWidthLocked() {
        return flowWidthLocked;
    }

    /**
     * @param flowWidthLocked the flowWidthLocked to set
     */
    public void setFlowWidthLocked(boolean flowWidthLocked) {
        this.flowWidthLocked = flowWidthLocked;
    }

    /**
     * @return the lockedMapScale
     */
    public double getLockedMapScale() {
        return lockedMapScale;
    }

    /**
     * @param lockedMapScale the lockedMapScale to set
     */
    public void setLockedMapScale(double lockedMapScale) {
        this.lockedMapScale = lockedMapScale;
    }

    /**
     * @return the maxFlowStrokeWidth
     */
    public double getMaxFlowStrokeWidth() {
        return maxFlowStrokeWidth;
    }

    /**
     * @param maxFlowStrokeWidth the maxFlowStrokeWidth to set
     */
    public void setMaxFlowStrokeWidth(double maxFlowStrokeWidth) {
        this.maxFlowStrokeWidth = maxFlowStrokeWidth;
    }

    /**
     * @return the maxNodeSize
     */
    public double getMaxNodeSize() {
        return maxNodeSize;
    }

    /**
     * @param maxNodeSize the maxNodeSize to set
     */
    public void setMaxNodeSize(double maxNodeSize) {
        this.maxNodeSize = maxNodeSize;
    }

}
