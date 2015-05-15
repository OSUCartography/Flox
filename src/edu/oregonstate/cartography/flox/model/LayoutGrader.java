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
                int numberOfIntersections = intersects.getNumGeometries();

                if (flow1.crosses(flow2) && numberOfIntersections > 1) {

                    Coordinate start1 = flow1.getCoordinates()[0];
                    Coordinate start2 = flow2.getCoordinates()[0];
                    Coordinate end1 = flow1.getCoordinates()[flow1.getNumPoints() - 1];
                    Coordinate end2 = flow2.getCoordinates()[flow2.getNumPoints() - 1];

                    if (start1.equals(start2) || start1.equals(end2) || end1.equals(start2) || end1.equals(end2)) {
                        intersections = intersections + (numberOfIntersections - 1);
                    } else {
                        intersections = intersections + numberOfIntersections;
                    }

                } else if (flow1.crosses(flow2)) {
                    intersections++;
                }

            }

        }

        return intersections;
    }

    public static boolean linesIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        // Return false if either of the lines have zero length
        if (x1 == x2 && y1 == y2
                || x3 == x4 && y3 == y4) {
            return false;
        }
        // Fastest method, based on Franklin Antonio's "Faster Line Segment Intersection" topic "in Graphics Gems III" book (http://www.graphicsgems.org/)
        double ax = x2 - x1;
        double ay = y2 - y1;
        double bx = x3 - x4;
        double by = y3 - y4;
        double cx = x1 - x3;
        double cy = y1 - y3;

        double alphaNumerator = by * cx - bx * cy;
        double commonDenominator = ay * bx - ax * by;
        if (commonDenominator > 0) {
            if (alphaNumerator < 0 || alphaNumerator > commonDenominator) {
                return false;
            }
        } else if (commonDenominator < 0) {
            if (alphaNumerator > 0 || alphaNumerator < commonDenominator) {
                return false;
            }
        }
        double betaNumerator = ax * cy - ay * cx;
        if (commonDenominator > 0) {
            if (betaNumerator < 0 || betaNumerator > commonDenominator) {
                return false;
            }
        } else if (commonDenominator < 0) {
            if (betaNumerator > 0 || betaNumerator < commonDenominator) {
                return false;
            }
        }
        if (commonDenominator == 0) {
         // This code wasn't in Franklin Antonio's method. It was added by Keith Woodward.
            // The lines are parallel.
            // Check if they're collinear.
            double y3LessY1 = y3 - y1;
            double collinearityTestForP3 = x1 * (y2 - y3) + x2 * (y3LessY1) + x3 * (y1 - y2);   // see http://mathworld.wolfram.com/Collinear.html
            // If p3 is collinear with p1 and p2 then p4 will also be collinear, since p1-p2 is parallel with p3-p4
            if (collinearityTestForP3 == 0) {
                // The lines are collinear. Now check if they overlap.
                if (x1 >= x3 && x1 <= x4 || x1 <= x3 && x1 >= x4
                        || x2 >= x3 && x2 <= x4 || x2 <= x3 && x2 >= x4
                        || x3 >= x1 && x3 <= x2 || x3 <= x1 && x3 >= x2) {
                    if (y1 >= y3 && y1 <= y4 || y1 <= y3 && y1 >= y4
                            || y2 >= y3 && y2 <= y4 || y2 <= y3 && y2 >= y4
                            || y3 >= y1 && y3 <= y2 || y3 <= y1 && y3 >= y2) {
                        return true;
                    }
                }
            }
            return false;
        }
        return true;
    }
    
    public static Point getLineLineIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
      double det1And2 = det(x1, y1, x2, y2);
      double det3And4 = det(x3, y3, x4, y4);
      double x1LessX2 = x1 - x2;
      double y1LessY2 = y1 - y2;
      double x3LessX4 = x3 - x4;
      double y3LessY4 = y3 - y4;
      double det1Less2And3Less4 = det(x1LessX2, y1LessY2, x3LessX4, y3LessY4);
      if (det1Less2And3Less4 == 0){
         // the denominator is zero so the lines are parallel and there's either no solution (or multiple solutions if the lines overlap) so return null.
         return null;
      }
      double x = (det(det1And2, x1LessX2,
            det3And4, x3LessX4) /
            det1Less2And3Less4);
      double y = (det(det1And2, y1LessY2,
            det3And4, y3LessY4) /
            det1Less2And3Less4);
      return new Point(x, y);
   }
   protected static double det(double a, double b, double c, double d) {
      return a * d - b * c;
   }

}
