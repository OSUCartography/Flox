package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.GeometryCollection;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

/**
 * Model for Flox.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Model {

    /**
     * Flows
     */
    private final ArrayList<BezierFlow> flows = new ArrayList<>();

    private final Map map = new Map();
    
    
    public Model() {
    }

    /**
     * Add a flow.
     *
     * @param flow The flow to add.
     */
    public void addFlow(BezierFlow flow) {
        flows.add(flow);
    }

    /**
     * Remove all flows.
     */
    public void clearFlows() {
        flows.clear();
    }

    /**
     * Returns the geometry if it is a GeometryCollection. Otherwise creates
     * and returns a new GeometryCollection containing the geometry.
     * @return the geometry
     */
    public GeometryCollection getGeometryCollection() {
        return map.getGeometryCollection();
    }
    
    /**
     * Returns the bounding box of all flows, excluding the other geometry.
     * @return 
     */
    public Rectangle2D getFlowsBoundingBox() {
        int nFlows = flows.size();
        if (nFlows == 0) {
            return null;
        }
        Rectangle2D bb = flows.get(0).getBoundingBox();
        for (int i = 0; i< nFlows; i++) {
            bb = bb.createUnion(flows.get(i).getBoundingBox());
        }
        return bb;
    }
    
    /**
     * Compute the bounding box for all map geometry, including the flows.
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
     * @return The iterator.
     */
    public Iterator<BezierFlow> flowIterator() {
        return flows.iterator();
    }

    public Iterator<Layer> layerIterator() {
        return map.layerIterator();
    }

    public Collection<Layer> getLayers() {
        return map.getLayers();
    }
    
    /**
     * Returns a layer specified by an id.
     * @param id
     * @return 
     */
    public Layer getLayer(int id) {
        return map.getLayer(id);
    }
    
    /**
     * Add a layer to the map.
     * @param collection Geometry for the layer.
     * @return The new layer.
     */
    public Layer addLayer(GeometryCollection collection) {
        return map.addLayer(collection);
    }

    /**
     * Add a layer to the map.
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
}
