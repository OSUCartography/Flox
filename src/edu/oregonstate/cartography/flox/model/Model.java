package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryCollectionIterator;
import com.vividsolutions.jts.geom.GeometryFactory;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.Iterator;
import org.jgrapht.UndirectedGraph;
import org.jgrapht.graph.SimpleGraph;

/**
 * Model for Flox.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Model {

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
     * @return the minFlowNodes
     */
    public double getMinFlowNodes() {
        return minFlowNodes;
    }

    /**
     * @param minFlowNodes the minFlowNodes to set
     */
    public void setMinFlowNodes(double minFlowNodes) {
        this.minFlowNodes = minFlowNodes;
    }

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

    /**
     * Graph of edges (CubicBezierFlow) and nodes (Point)
     */
    private UndirectedGraph<Point, Flow> graph = new SimpleGraph<>(Flow.class);

    // Length of the arrow. This number is modified by a GUI slider bar.
    // It is multiplied by the value of each flow and the flowWidthScale.
    private double arrowLength = 0.01;

    // Width of the arrow. This number is modified This number is modified by a GUI slider bar.
    // It is multiplied by the value of each flow and the flowWidthScale.
    private double arrowWidth = 0.01;

    private double arrowEdgeCtrlLength = 0.5;

    private double arrowEdgeCtrlWidth = 0.5;

    private double arrowCornerPosition = 0.0;

    private double arrowSizeRatio = 0.0;

    /**
     * Scale factor to transform flow values to flow stroke widths
     */
    private double flowWidthScale = 1;

    /**
     * if true, a flow exerts forces on itself
     */
    private boolean flowExertingForcesOnItself = false;

    /**
     * Start and end node exert a larger force than points along flow lines
     */
    private double nodeWeightFactor = 0.0;

    /**
     * Weight for the anti-torsion force
     */
    private double antiTorsionWeight = .8;

    /**
     * Stiffness factor for peripheral flows
     */
    private double peripheralStiffnessFactor = 0.0;

    /**
     * spring stiffness of longest flow
     */
    private double maxFlowLengthSpringConstant = .2;

    /**
     * spring stiffness of zero-length flow
     */
    private double minFlowLengthSpringConstant = 1.0;

    // This determines the amount of force that objects far away from the target
    // can apply to the target.  The lower the distanceWeightExponent, the more force distant
    // objects are permitted to apply.
    private double distanceWeightExponent = 10.0;

    private boolean enforceRangebox = true;

    private boolean enforceCanvasRange = true;

    private boolean drawArrows = true;

    private double canvasPadding = 0.5;

    private double flowRangeboxHeight = .5;

    private double flowDistanceFromEndPoint = 0.5d;

    private double minFlowNodes = 2.5;
    
    private Rectangle2D canvas;

    /**
     * A reference to the map with layers and geometry.
     */
    private final Map map = new Map();

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
     * Constructor of the model.
     */
    public Model() {
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
        Point sourceVertex = findNodeInGraph(flow.getStartPt());
        Point targetVertex = findNodeInGraph(flow.getEndPt());
        graph.addVertex(sourceVertex);
        graph.addVertex(targetVertex);
        graph.addEdge(sourceVertex, targetVertex, flow);
    }

    /**
     * Searches for a point in the graph with the specified coordinates
     *
     * @param target A point with the coordinates to search.
     * @return The point with coordinates x and y in the graph or the passed
     * point if no point with the same coordinates exist in the graph.
     */
    private Point findNodeInGraph(Point target) {
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
     * Replace the current flows with new flows.
     *
     * @param flows The new flows.
     */
    public void setFlows(Collection<Flow> flows) {
        clearFlows();
        flows.stream().forEach((flow) -> {
            addFlow(flow);
        });
    }

    /**
     * Remove all flows.
     */
    public void clearFlows() {
        graph = new SimpleGraph<>(CubicBezierFlow.class);
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
        Iterator<Flow> iter = graph.edgeSet().iterator();
        double max = iter.next().value;
        while (iter.hasNext()) {
            double v = iter.next().getValue();
            if (v > max) {
                max = v;
            }
        }
        return max;
    }

    /**
     * Returns the length of the longest flow base line.
     *
     * @return The length of the longest base line.
     */
    public double getLongestFlowLength() {
        double maxLength = 0;
        Iterator<Flow> iterator = flowIterator();
        while (iterator.hasNext()) {
            Flow flow = iterator.next();
            double l = flow.getBaselineLength();
            if (l > maxLength) {
                maxLength = l;
            }
        }
        return maxLength;
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

    public double getShortestFlowLengthDividedByMinFlowNodes() {
        return getShortestFlowLength() / getMinFlowNodes();
    }
    
    public double getLargestFlowValue() {
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
     * Returns the scale factor applied to flow values when drawing the flows.
     *
     * @return the flowWidthScale
     */
    public double getFlowWidthScale() {
        return flowWidthScale;
    }

    /**
     * Sets the scale factor applied to flow values when drawing the flows.
     *
     * @param flowWidthScale the flowWidthScale to set
     */
    public void setFlowWidthScale(double flowWidthScale) {
        this.flowWidthScale = flowWidthScale;
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
     * @return the flowExertingForcesOnItself
     */
    public boolean isFlowExertingForcesOnItself() {
        return flowExertingForcesOnItself;
    }

    /**
     * @param flowExertingForcesOnItself the flowExertingForcesOnItself to set
     */
    public void setFlowExertingForcesOnItself(boolean flowExertingForcesOnItself) {
        this.flowExertingForcesOnItself = flowExertingForcesOnItself;
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

    public double getArrowLength() {
        return arrowLength;
    }

    public double getArrowWidth() {
        return arrowWidth;
    }

    public void setArrowLength(double arrowLength) {
        this.arrowLength = arrowLength;
    }

    public void setArrowWidth(double arrowWidth) {
        this.arrowWidth = arrowWidth;
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
}
