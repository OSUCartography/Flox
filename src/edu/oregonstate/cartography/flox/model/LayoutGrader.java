/*
 * 
 */
package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.linearref.LinearGeometryBuilder;
import java.util.ArrayList;

/**
 *
 * @author Dan Stephen
 */
public class LayoutGrader {

    /**
     * Counts the total number of intersections between flows in an ArrayList.
     * Converts the flows to JTS geometry objects, and uses the intersect()
     * method on every possible combination of 2 flows from the ArrayList.
     *
     * @param flows An ArrayList of Flow objects
     * @return
     */
    public static int countFlowIntersections(ArrayList<Flow> flows) {

        ArrayList<Geometry> flowPolylines = new ArrayList<>();

        for (Flow flow : flows) {

            ArrayList<Point> flowPoints = new ArrayList<>(flow.toStraightLineSegments(0.01));

            GeometryFactory geometryFactory = new GeometryFactory();
            LinearGeometryBuilder lineBuilder = new LinearGeometryBuilder(geometryFactory);

            for (Point point : flowPoints) {
                lineBuilder.add(new Coordinate(point.x, point.y));
            }

            flowPolylines.add(lineBuilder.getGeometry());
        }

        int intersections = 0;
        for (int i = 0; i < flowPolylines.size(); i++) {

            for (int j = i + 1; j < flowPolylines.size(); j++) {

                Geometry flow1 = flowPolylines.get(i);
                Geometry flow2 = flowPolylines.get(j);

                Geometry intersects = flow1.intersection(flow2);
                int theLength = intersects.getNumGeometries();

                if (flow1.crosses(flow2)) {
                    intersections++;
                }

                if (theLength > 1) {
                    intersections = intersections + (theLength - 1);
                }

            }

        }

        return intersections;
    }

}
