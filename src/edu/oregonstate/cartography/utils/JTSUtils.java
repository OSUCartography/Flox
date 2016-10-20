package edu.oregonstate.cartography.utils;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import edu.oregonstate.cartography.flox.model.Point;
import java.util.ArrayList;

/**
 * JTS Java Topology Suite utilities.
 *
 * @author Bernhard Jenny, School of Science - Geospatial Science, RMIT
 * University, Melbourne
 */
public class JTSUtils {

    private JTSUtils() {
    }

    /**
     * Converts a list of points to a JTS LineString.
     *
     * @param points point to convert
     * @return a new line string
     */
    public static LineString pointsToLineString(ArrayList<Point> points) {
        GeometryFactory geometryFactory = new GeometryFactory();
        int numPoints = points.size();
        Coordinate[] xy = new Coordinate[numPoints];
        for (int i = 0; i < numPoints; i++) {
            Point point = points.get(i);
            xy[i] = new Coordinate(point.x, point.y);
        }
        return geometryFactory.createLineString(xy);
    }
}
