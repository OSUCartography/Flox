package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * A map contains a set of layers with symbolization.
 * 
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Map {

    /**
     * The layers of this map.
     */
    private final ArrayList<Layer> layers = new ArrayList<>();

    public Map() {
    }

    /**
     * @param geometry the geometry to set
     */
    public void addLayer(Geometry geometry) {
        layers.add(new Layer(geometry));
    }

    /**
     * Deletes all current layers.
     */
    public void removeAllLayers() {
        layers.clear();
    }

    /**
     * Returns an iterator for all layers.
     * @return 
     */
    public Iterator<Layer> layerIterator() {
        return layers.iterator();
    }

    /**
     * Returns a OGC Simple Feature geometry collection with all geometry 
     * features of the map.
     * @return 
     */
    public GeometryCollection getGeometryCollection() {
        ArrayList<Geometry> layerGeometries = new ArrayList<>();
        for (Layer layer : layers) {
            layerGeometries.add(layer.getGeometry());
        }
        Geometry[] geomArray = layerGeometries.toArray(new Geometry[0]);
        return new GeometryFactory().createGeometryCollection(geomArray);
    }

    /**
     * Returns the bounding box including all layers.
     *
     * @return
     */
    Rectangle2D getBoundingBox() {
        Rectangle2D bb = null;
        for (Layer layer : layers) {
            Rectangle2D layerBB = layer.getBoundingBox();
            if (layerBB != null) {
                if (bb == null) {
                    bb = layerBB;
                } else {
                    bb.createUnion(layerBB);
                }
            }
        }
        return bb;
    }

}
