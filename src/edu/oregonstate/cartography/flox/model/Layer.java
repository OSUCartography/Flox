package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;
import java.awt.geom.Rectangle2D;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.adapters.XmlJavaTypeAdapter;

/**
 * A layer for a map. Contains OGC Simple Feature geometry and a symbol definition
 * to draw the geometry.
 * 
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */

//Every non static, non transient field in a JAXB-bound class will be 
//automatically bound to XML, unless annotated by @XmlTransient
@XmlAccessorType(XmlAccessType.FIELD)

public class Layer {

    /**
     * counts created layers
     */
    private static int layerCounter = 0;
    
    /**
     * layer geometry
     */
    @XmlJavaTypeAdapter(GeometrySerializer.class)
    private Geometry geometry;

    /**
     * symbol for drawing the layer geometry
     */
    private final VectorSymbol vectorSymbol = new VectorSymbol();

    /**
     * layer name
     */
    private String name = "Layer " + ++layerCounter;
    
    /**
     * Constructor
     */
    public Layer() {
    }

    /**
     * Constructor
     * @param geometry The geometry for the layer.
     */
    public Layer(Geometry geometry) {
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
            Geometry[] geometries = geometry != null ? new Geometry[]{geometry} : null;
            return new GeometryFactory().createGeometryCollection(geometries);
        }
    }

    /**
     * Returns the bounding box of all geometries in this layer.
     *
     * @return
     */
    Rectangle2D getBoundingBox() {
        if (geometry == null) {
            return null;
        }
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
    
    @Override
    public String toString() {
        return name;
    }
}
