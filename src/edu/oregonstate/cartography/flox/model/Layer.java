package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;
import java.awt.geom.Rectangle2D;

/**
 * A layer for a map. Contains OGC Simple Feature geometry and a symbol definition
 * to draw the geometry.
 * 
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Layer {

    /**
     * layer geometry
     */
    private Geometry geometry;

    /**
     * Symbol for drawing the layer geometry
     */
    private final VectorSymbol vectorSymbol = new VectorSymbol();

    /**
     * Constructor
     */
    public Layer() {
    }

    /**
     * Constructor
     * @param geometry The geometry for the layer.
     */
    Layer(Geometry geometry) {
        this.geometry = geometry;
    }

    /**
     * @return the geometry
     */
    public Geometry getGeometry() {
        return geometry;
    }

    /**
     * @param geometry the geometry to set
     */
    public void setGeometry(Geometry geometry) {
        this.geometry = geometry;
    }
    
    /**
     * Returns the geometry if it is a GeometryCollection. Otherwise creates
     * and returns a new GeometryCollection containing the geometry.
     * @return the geometry
     */
    public GeometryCollection getGeometryCollection() {
        if (geometry instanceof GeometryCollection) {
            return (GeometryCollection) geometry;
        } else {
            Geometry[] geometries = new Geometry[]{geometry};
            return new GeometryFactory().createGeometryCollection(geometries);
        }
    }

    /**
     * Returns the bounding box of all geometries in this layer.
     *
     * @return
     */
    Rectangle2D getBoundingBox() {
        Envelope env = geometry.getEnvelopeInternal();
        if (env == null) {
            return null;
        }
        return new Rectangle2D.Double(env.getMinX(), env.getMinY(),
                env.getWidth(), env.getHeight());
    }

    /**
     * @return the vectorSymbol
     */
    public VectorSymbol getVectorSymbol() {
        return vectorSymbol;
    }
    
}
