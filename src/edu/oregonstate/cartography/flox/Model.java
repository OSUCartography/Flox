package edu.oregonstate.cartography.flox;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;
import java.util.ArrayList;

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

    /**
     * map geometry
     */
    private Geometry geometry;

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

}
