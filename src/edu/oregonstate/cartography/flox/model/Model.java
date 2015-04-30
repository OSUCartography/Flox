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

    /**
     * Graph of edges (BezierFlow) and nodes (Point)
     */
    private UndirectedGraph<Point, BezierFlow> graph = new SimpleGraph<>(BezierFlow.class);

    /**
     * Scale factor to transform flow values to flow stroke widths
     */
    private double flowWidthScale = 1;

    /**
     * A reference to the map with layers and geometry.
     */
    private final Map map = new Map();

    public Model() {
    }

    /**
     * Add a flow.
     *
     * @param flow The flow to add.
     */
    public void addFlow(BezierFlow flow) {
        Point sourceVertex = flow.getStartPt();
        Point targetVertex = flow.getEndPt();
        graph.addVertex(sourceVertex);
        graph.addVertex(targetVertex);
        graph.addEdge(sourceVertex, targetVertex, flow);
    }

    public void setFlows(Collection<BezierFlow> flows) {
        clearFlows();
        for (BezierFlow flow : flows) {
            addFlow(flow);
        }
    }

    /**
     * Remove all flows.
     */
    public void clearFlows() {
        graph = new SimpleGraph<>(BezierFlow.class);
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
        Iterator<BezierFlow> iter = graph.edgeSet().iterator();
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
    public Iterator<BezierFlow> flowIterator() {
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
        Iterator<BezierFlow> iter = graph.edgeSet().iterator();
        double max = iter.next().value;
        while (iter.hasNext()) {
            double v = iter.next().getValue();
            if (v > max) {
                max = v;
            }
        }
        return max;
    }

    public Iterator<Layer> layerIterator() {
        return map.layerIterator();
    }

    public Collection<Layer> getLayers() {
        return map.getLayers();
    }

    /**
     * Returns a layer specified by an id.
     *
     * @param id
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

    public void removeAllLayers() {
        map.removeAllLayers();
    }

    public void removeLayer(int id) {
        map.removeLayer(id);
    }

    public int getNbrLayers() {
        return map.getNbrLayers();
    }

    /**
     * @return the flowWidthScale
     */
    public double getFlowWidthScale() {
        return flowWidthScale;
    }

    /**
     * @param flowWidthScale the flowWidthScale to set
     */
    public void setFlowWidthScale(double flowWidthScale) {
        this.flowWidthScale = flowWidthScale;
    }
}
