package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.GeometryCollection;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
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

    public enum CurveType {
        CUBIC,
        QUADRATIC
    }
    
    /**
     * Graph of edges (CubicBezierFlow) and nodes (Point)
     */
    private UndirectedGraph<Point, Flow> graph = new SimpleGraph<>(Flow.class);

    /**
     * Scale factor to transform flow values to flow stroke widths
     */
    private double flowWidthScale = 1;

    /**
     * flag for drawing control points
     */
    private boolean drawControlPoints;
    
    /**
     * A reference to the map with layers and geometry.
     */
    private final Map map = new Map();

    /**
     * Either work with cubic or quadratic curves
     */
    private CurveType curveType = CurveType.CUBIC;
    
    /**
     * Constructor of the model.
     */
    public Model() {
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
     * Returns the geometry if it is a GeometryCollection. Otherwise creates and
     * returns a new GeometryCollection containing the geometry.
     *
     * @return the geometry
     */
    public GeometryCollection getGeometryCollection() {
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
     * Returns an iterator for the nodes.
     *
     * @return The iterator.
     */
    public Iterator<Point> nodeIterator() {
        return graph.vertexSet().iterator();
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
     * Returns all map layers.
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
     * @param id Index of the layer to remove.
     */
    public void removeLayer(int id) {
        map.removeLayer(id);
    }

    /**
     * Returns the number of map layers.
     * @return 
     */
    public int getNbrLayers() {
        return map.getNbrLayers();
    }

    /**
     * Returns the scale factor applied to flow values when drawing the flows.
     * @return the flowWidthScale
     */
    public double getFlowWidthScale() {
        return flowWidthScale;
    }

    /**
     * Sets the scale factor applied to flow values when drawing the flows.
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
     * @return the drawControlPoints
     */
    public boolean isDrawControlPoints() {
        return drawControlPoints;
    }

    /**
     * 
     * @param drawControlPoint the drawControlPoints to set
     */
    public void setDrawControlPoints(boolean drawControlPoints) {
        this.drawControlPoints = drawControlPoints;
    }
    
}
