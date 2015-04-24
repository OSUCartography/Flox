package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
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
        if (geometry == null) {
            return getFlowsBoundingBox();
        }
        Envelope env = geometry.getEnvelopeInternal();
        Rectangle2D bb = new Rectangle2D.Double(env.getMinX(), env.getMinY(),
                env.getWidth(), env.getHeight());
        Rectangle2D flowsBB = getFlowsBoundingBox();
        return flowsBB == null ? bb : bb.createUnion(flowsBB);
    }

    /**
     * Returns an iterator for the flows.
     * @return The iterator.
     */
    public Iterator<BezierFlow> flowIterator() {
        return flows.iterator();
    }
}
