package edu.oregonstate.cartography.flox;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Flox {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        Coordinate[] coordinates = new Coordinate[]{
            new Coordinate(0, 0), new Coordinate(10, 10),
            new Coordinate(20, 20)};
        Geometry g1 = new GeometryFactory().createLineString(coordinates);

    }

}
