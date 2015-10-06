package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollectionIterator;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.io.WKTWriter;
import edu.oregonstate.cartography.utils.GeometryUtils;
import java.awt.geom.GeneralPath;
import java.awt.geom.PathIterator;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * A flow based on a quadratic BŽzier curve.
 *
 * @author Bernhard Jenny
 * @author Daniel Stephen
 */
public final class Flow {

    /**
     * start point of flow.
     */
    protected Point startPt;

    /**
     * end point of flow.
     */
    protected Point endPt;

    /**
     * control point.
     */
    private Point cPt;

    /**
     * mapped value.
     */
    private double value;

    /**
     * clip area for the start of the flow.
     */
    private Geometry startClipArea;

    /**
     * startClipArea serialized to WKT format
     */
    private String startClipAreaWKT;
    /**
     * clip area for the end of the flow.
     */
    private Geometry endClipArea;

    /**
     * endClipArea serialized to WKT format
     */
    private String endClipAreaWKT;

    /**
     * selection flag
     */
    private boolean selected = false;

    /**
     * Locked flag. Locked flows are not affected by forces, but still emit
     * forces onto other flows.
     */
    private boolean locked = false;

    /**
     * Construct a Flow from 3 points.
     *
     * @param startPt Start point
     * @param ctrlPt Control point
     * @param endPt End point
     */
    public Flow(Point startPt, Point ctrlPt, Point endPt) {
        this.startPt = startPt;
        this.cPt = ctrlPt;
        this.endPt = endPt;
    }

    /**
     * Construct a Flow from a start point and an end point.
     *
     * @param startPt Start point
     * @param endPt End point
     */
    public Flow(Point startPt, Point endPt) {

        this.startPt = startPt;
        this.endPt = endPt;

        // Angle between the straight line connecting start and end point and 
        // the line connecting the start/end point with the corresponding Bezier 
        // control point.
        double alpha = 0.5;

        // Distance between startPt and endPt
        double dist = getBaselineLength();
        double tangentLength = dist * 0.5;
        computeCtrlPt(alpha, tangentLength);
    }

    /**
     * Returns the start point of the flow.
     *
     * @return the startPt
     */
    public Point getStartPt() {
        return startPt;
    }

    /**
     * Set the start point of the flow.
     *
     * @param startPt the startPt to set
     */
    public void setStartPt(Point startPt) {
        this.startPt = startPt;
    }

    /**
     * Returns the end point of the flow.
     *
     * @return the endPt
     */
    public Point getEndPt() {
        return endPt;
    }

    /**
     * Set the end point of the flow.
     *
     * @param endPt the endPt to set
     */
    public void setEndPt(Point endPt) {
        this.endPt = endPt;
    }

    /**
     * Returns the distance between start and end point
     *
     * @return
     */
    public double getBaselineLength() {
        double dx = startPt.x - endPt.x;
        double dy = startPt.y - endPt.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Returns the azimuthal angle for a line between a start and end point
     *
     * @return Angle in radians, counter-clockwise, 0 is pointing eastwards
     */
    public double getBaselineAzimuth() {
        final double dx = endPt.x - startPt.x;
        final double dy = endPt.y - startPt.y;
        return Math.atan2(dy, dx);
    }

    public Point getBaseLineMidPoint() {
        return new Point((endPt.x + startPt.x) / 2, (endPt.y + startPt.y) / 2);
    }

    public void reverseFlow() {
        Point temp = startPt;
        startPt = endPt;
        endPt = temp;
    }

    /**
     * @return the flow value
     */
    public double getValue() {
        return value;
    }

    /**
     * @param value the value to set
     */
    public void setValue(double value) {
        this.value = value;
    }

    /**
     * @return the startClipArea
     */
    public Geometry getStartClipArea() {
        return startClipArea;
    }

    /**
     * @param startClipArea the startClipArea to set
     */
    public void setStartClipArea(Geometry startClipArea) {
        this.startClipArea = startClipArea;
        if (startClipArea != null) {
            startClipAreaWKT = new WKTWriter().write(startClipArea);
        } else {
            startClipAreaWKT = null;
        }
    }

    /**
     * @return the endClipArea
     */
    public Geometry getEndClipArea() {
        return endClipArea;
    }

    /**
     * @param endClipArea the endClipArea to set
     */
    public void setEndClipArea(Geometry endClipArea) {
        this.endClipArea = endClipArea;
        if (endClipArea != null) {
            this.endClipAreaWKT = new WKTWriter().write(endClipArea);
        } else {
            endClipAreaWKT = null;
        }
    }

    /**
     * @return the selected
     */
    public boolean isSelected() {
        return selected;
    }

    /**
     * @param selected the selected to set
     */
    public void setSelected(boolean selected) {
        this.selected = selected;
    }

    /**
     * @return the locked
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * @param locked the locked to set
     */
    public void setLocked(boolean locked) {
        this.locked = locked;
    }

    /**
     * Creates a straight flow line by placing the control between the start
     * point and the end point.
     */
    public void straighten() {
        cPt.x = (startPt.x + endPt.x) / 2;
        cPt.y = (startPt.y + endPt.y) / 2;
    }

    /**
     * Compute first control point from orientation of base line
     *
     * @param alpha angle between the base line and the line connecting the
     * start point with the first control point.
     * @param dist Distance between start point and first control point.
     */
    private void computeCtrlPt(double alpha, double dist) {
        final double lineOrientation = getBaselineAzimuth();
        final double azimuth = lineOrientation + alpha;
        final double dx1 = Math.sin(azimuth) * dist;
        final double dy1 = Math.cos(azimuth) * dist;
        double cPt1X = startPt.x + dx1;
        double cPt1Y = startPt.y + dy1;
        cPt = new Point(cPt1X, cPt1Y);
    }

    /**
     * Returns a bounding box containing the curve. The control point can be
     * outside of the bounding box returned by this method. Does not take any
     * line width into account. Based on
     * http://pomax.github.io/bezierinfo/#boundingbox
     *
     * @return Bounding box.
     */
    public Rectangle2D.Double getBoundingBox() {
        // initialize bounding box with start and end points
        double xmin, xmax, ymin, ymax;
        if (startPt.x > endPt.x) {
            xmin = endPt.x;
            xmax = startPt.x;
        } else {
            xmin = startPt.x;
            xmax = endPt.x;
        }
        if (startPt.y > endPt.y) {
            ymin = endPt.y;
            ymax = startPt.y;
        } else {
            ymin = startPt.y;
            ymax = endPt.y;
        }

        // Compute parameter t for the root of the first derivative of the x 
        // position. This is the t parameter for the extremum in x of the curve, 
        // as the first derivative is 0 at the extremum.
        double tx = (startPt.x - cPt.x) / (startPt.x - 2 * cPt.x + endPt.x);
        // t must be in [0,1]
        if (Double.isFinite(tx) && tx >= 0 && tx <= 1) {
            double one_minus_tx = 1d - tx;
            // compute x position of extrema
            double x = one_minus_tx * one_minus_tx * startPt.x
                    + 2 * one_minus_tx * tx * cPt.x + tx * tx * endPt.x;
            // extend bounding box
            xmin = Math.min(xmin, x);
            xmax = Math.max(xmax, x);
        }

        // repeat for y
        double ty = (startPt.y - cPt.y) / (startPt.y - 2 * cPt.y + endPt.y);
        if (Double.isFinite(ty) && ty >= 0 && ty <= 1) {
            double one_minus_ty = 1d - ty;
            double y = one_minus_ty * one_minus_ty * startPt.y
                    + 2 * one_minus_ty * ty * cPt.y + ty * ty * endPt.y;
            ymin = Math.min(ymin, y);
            ymax = Math.max(ymax, y);
        }

        return new Rectangle2D.Double(xmin, ymin, xmax - xmin, ymax - ymin);
    }

    /**
     * Returns the control point.
     *
     * @return the control point
     */
    public Point getCtrlPt() {
        return cPt;
    }

    /**
     * Sets the control point.
     *
     * @param cPt the control point to set
     */
    public void setControlPoint(Point cPt) {
        this.cPt = cPt;
    }

    /**
     *
     * @param deCasteljauTol
     * @return
     */
    public ArrayList<Point> toStraightLineSegmentsWithIrregularLength(
            double deCasteljauTol) {
        assert (deCasteljauTol > 0);

        ArrayList<Point> irregularPoints = new ArrayList<>();
        GeneralPath path = new GeneralPath();
        path.moveTo(startPt.x, startPt.y);
        path.quadTo(cPt.x, cPt.y, endPt.x, endPt.y);
        // FIXME division by 100?
        PathIterator iter = path.getPathIterator(null, deCasteljauTol / 100);
        double[] coords = new double[6];
        while (!iter.isDone()) {
            iter.currentSegment(coords);
            irregularPoints.add(new Point(coords[0], coords[1]));
            iter.next();
        }

        return irregularPoints;
    }

    private static LineString pointsToLineString(ArrayList<Point> points) {
        // construct LineString from the current Bezier flow geometry
        GeometryFactory geometryFactory = new GeometryFactory();
        int numPoints = points.size();
        Coordinate[] xy = new Coordinate[numPoints];
        for (int i = 0; i < numPoints; i++) {
            Point point = points.get(i);
            xy[i] = new Coordinate(point.x, point.y);
        }

        return geometryFactory.createLineString(xy);
    }

    /**
     * Converts this Bezier curve to straight line segments. Applies clipping
     * with start and end nodes and arrowheads.
     *
     * @param endClipRadius Clip the end of the flow with a circle of this
     * radius.
     * @param deCasteljauTol The maximum distance between the curve and the
     * straight line segments.
     * @return An list of irregularPoints, including copies of the start point
     * and the end point.
     */
    public ArrayList<Point> toClippedStraightLineSegments(double endClipRadius, double deCasteljauTol) {

        // FIXME 0 parameter
        Flow clippedFlow = Flow.clipFlow(this, endClipRadius, deCasteljauTol);
        return clippedFlow.toUnclippedStraightLineSegments(deCasteljauTol);
    }

    /**
     * Returns the length of a simple line string defined by a series of points.
     *
     * @param lineString
     * @return The length.
     */
    private double lineStringLength(ArrayList<Point> lineString) {
        double l = 0;
        int nPoints = lineString.size();
        double x0 = lineString.get(0).x;
        double y0 = lineString.get(0).y;

        for (int i = 1; i < nPoints; i++) {
            double x1 = lineString.get(i).x;
            double y1 = lineString.get(i).y;
            double dx = x0 - x1;
            double dy = y0 - y1;
            l += Math.sqrt(dx * dx + dy * dy);
            x0 = x1;
            y0 = y1;
        }
        return l;
    }

    /**
     * Converts this Bezier curve to straight line segments. Does not apply
     * clipping with start and end nodes. Does not apply clipping for
     * arrowheads.
     *
     * @param deCasteljauTol The maximum distance between the curve and the
     * straight line segments.
     * @return An list of irregularPoints, including copies of the start point
     * and the end point.
     */
    public ArrayList<Point> toUnclippedStraightLineSegments(double deCasteljauTol) {
        assert (deCasteljauTol > 0);

        ArrayList<Point> regularPoints = new ArrayList<>();
        ArrayList<Point> irregularPoints
                = toStraightLineSegmentsWithIrregularLength(deCasteljauTol);

        // compute distance between points in regular line string
        double totalLength = lineStringLength(irregularPoints);
        // FIXME abusing the deCasteljauTol, which is not really the tolerance
        // for de Casteljau's algorithm (it is devided by 100).
        double targetDist = totalLength / Math.round(totalLength / deCasteljauTol);

        // create new point set with regularly distributed irregularPoints
        double startX = irregularPoints.get(0).x;
        double startY = irregularPoints.get(0).y;

        // add start point
        regularPoints.add(new Point(startX, startY));

        double length = 0;
        int nPoints = irregularPoints.size();
        for (int i = 0; i < nPoints; i++) {
            Point inputPt = irregularPoints.get(i);
            double endX = inputPt.x;
            double endY = inputPt.y;

            // normalized direction dx and dy
            double dx = endX - startX;
            double dy = endY - startY;
            final double l = Math.sqrt(dx * dx + dy * dy);
            dx /= l;
            dy /= l;

            double rest = length;
            length += l;
            while (length >= targetDist) {
                // compute new point
                length -= targetDist;
                startX += dx * (targetDist - rest);
                startY += dy * (targetDist - rest);
                rest = 0;
                regularPoints.add(new Point(startX, startY));
            }
            startX = endX;
            startY = endY;
        }

        // add end point
        regularPoints.add(irregularPoints.get(irregularPoints.size() - 1));
        return regularPoints;
    }

    /**
     * Returns the location on the BŽzier curve at parameter value t.
     *
     * @param t Parameter [0..1]
     * @return Location on curve.
     */
    public Point pointOnCurve(double t) {
        assert (t >= 0d && t <= 1d);

        double t2 = t * t;
        double mt = 1 - t;
        double mt2 = mt * mt;
        double x = startPt.x * mt2 + cPt.x * 2 * mt * t + endPt.x * t2;
        double y = startPt.y * mt2 + cPt.y * 2 * mt * t + endPt.y * t2;
        return new Point(x, y);
    }

    /**
     * Split a flow into two new flows. The new flows have the same value,
     * selection and lock state as this flow. The split flows have new start,
     * end, and control points. The first flow has the same start clip area as
     * this flow. The second flow has the same end clip area as this flow.
     *
     * Maths based on http://pomax.github.io/bezierinfo/#matrixsplit
     *
     * @param t Parametric position [0..1]
     * @return Two new flows if tx is > 0 and tx < 1. Otherwise two references
     * to this.
     */
    public Flow[] split(double t) {
        if (t <= 0 || t >= 1) {
            return new Flow[]{this, this};
        }

        double startX1 = startPt.x;
        double startY1 = startPt.y;
        double ctrlX1 = t * cPt.x - (t - 1) * startPt.x;
        double ctrlY1 = t * cPt.y - (t - 1) * startPt.y;
        double endX1 = t * t * endPt.x - 2 * t * (t - 1) * cPt.x + (t - 1) * (t - 1) * startPt.x;
        double endY1 = t * t * endPt.y - 2 * t * (t - 1) * cPt.y + (t - 1) * (t - 1) * startPt.y;

        Point start1 = new Point(startX1, startY1);
        Point ctrl1 = new Point(ctrlX1, ctrlY1);
        Point end1 = new Point(endX1, endY1);

        double startX2 = t * t * endPt.x - 2 * t * (t - 1) * cPt.x + (t - 1) * (t - 1) * startPt.x;
        double startY2 = t * t * endPt.y - 2 * t * (t - 1) * cPt.y + (t - 1) * (t - 1) * startPt.y;
        double ctrlX2 = t * endPt.x - (t - 1) * cPt.x;
        double ctrlY2 = t * endPt.y - (t - 1) * cPt.y;
        double endX2 = endPt.x;
        double endY2 = endPt.y;

        Point start2 = new Point(startX2, startY2);
        Point ctrl2 = new Point(ctrlX2, ctrlY2);
        Point end2 = new Point(endX2, endY2);

        Flow flow1 = new Flow(start1, ctrl1, end1);
        flow1.setValue(getValue());
        flow1.setSelected(isSelected());
        flow1.setLocked(isLocked());
        flow1.setStartClipArea(getStartClipArea());

        Flow flow2 = new Flow(start2, ctrl2, end2);
        flow2.setValue(getValue());
        flow2.setSelected(isSelected());
        flow2.setLocked(isLocked());
        flow2.setEndClipArea(getEndClipArea());

        return new Flow[]{flow1, flow2};
    }

    /**
     * Returns the curve parameter where a circle with radius r around the end
     * point intersects the BŽzier curve.
     *
     * @param r Radius of circle
     * @return Parameter t [0..1] where the circle intersects the flow.
     */
    public double getIntersectionTWithCircleAroundEndPoint(double r) {
        if (r <= 0) {
            return 1;   // tx = 1: end of curve
        }
        double t = 0.5;
        double t_step = 0.25;
        for (int i = 0; i < 20; i++) {
            Point pt = pointOnCurve(t);
            final double dx = endPt.x - pt.x;
            final double dy = endPt.y - pt.y;
            final double d = Math.sqrt(dx * dx + dy * dy);
            if (d < r) {
                t -= t_step;
            } else {
                t += t_step;
            }
            t_step /= 2;
        }

        return t;
    }

    /**
     * Returns the curve parameter where a circle with radius r around the start
     * point intersects the BŽzier curve.
     *
     * @param r Radius of circle
     * @return Parameter t [0..1] where the circle intersects the flow.
     */
    public double getIntersectionTWithCircleAroundStartPoint(double r) {
        if (r <= 0) {
            return 0;   // tx = 0: start of curve
        }
        double t = 0.5;
        double t_step = 0.25;
        for (int i = 0; i < 20; i++) {
            Point pt = pointOnCurve(t);
            final double dx = startPt.x - pt.x;
            final double dy = startPt.y - pt.y;
            final double d = Math.sqrt(dx * dx + dy * dy);
            if (d < r) {
                t += t_step;
            } else {
                t -= t_step;
            }
            t_step /= 2;
        }

        return t;
    }

    /**
     * Returns a flow with the start and/or end masking areas removed. If no
     * masking areas are defined, returns a reference to this flow.
     *
     * @param endClipRadius Clip the end of the flow with a circle of this
     * radius.
     * @param deCasteljauTol Tolerance for conversion to straight line segments.
     * @return A new flow (if something was clipped), or the passed flow.
     */
    public Flow getClippedFlow(double endClipRadius, double deCasteljauTol) {
        return Flow.clipFlow(this, endClipRadius, deCasteljauTol);
    }

    /**
     * Returns a flow with the start and/or end masking areas removed. If no
     * masking areas are defined, returns the passed flow.
     *
     * @param flow The flow to clip
     * @param endClipRadius Clip the end of the flow with a circle of this
     * radius.
     * @param deCasteljauTol Tolerance for conversion to straight line segments.
     * @return A new flow (if something was clipped), or the passed flow.
     */
    private static Flow clipFlow(Flow flow, double endClipRadius, double deCasteljauTol) {

        // Test whether start or end clip areas are defined.
        // If none is defined, the flow is not converted to straight line segments,
        // which is potentially expensive.
        boolean clipWithStartArea = flow.getStartClipArea() != null;
        boolean clipWithEndArea = flow.getEndClipArea() != null;

        // t parameter for location of end clipping
        double endT = 1;

        if (clipWithStartArea || clipWithEndArea) {

            // construct LineString from the current Bezier flow geometry
            ArrayList<Point> points = flow.toUnclippedStraightLineSegments(deCasteljauTol);
            LineString lineString = pointsToLineString(points);

            // clip with start area
            if (clipWithStartArea) {
                double startT = flow.clippingT(lineString, true);
                flow = flow.split(startT)[1];
            }

            // compute t parameter for clipping with the end area
            if (clipWithEndArea) {
                endT = flow.clippingT(lineString, false);
            }
        }

        if (endClipRadius > 0) {
            // compute t parameter for clipping with the circle around the end point
            double endNodeT = flow.getIntersectionTWithCircleAroundEndPoint(endClipRadius);
            // find the smaller of the two t parameters
            endT = Math.min(endT, endNodeT);
        }

        // cut off the end piece
        flow = flow.split(endT)[0];

        return flow;
    }

    /**
     * Computes the parameter t for the location where the flow needs to be
     * split for masking with the start or the end clip area.
     *
     * @param lineString This flow's geometry converted to straight line
     * segments.
     * @param clipWithStartArea If true, clip with start clip area, otherwise
     * clip with the end clip area.
     * @return The parameter t in [0, 1]
     */
    private double clippingT(LineString lineString, boolean clipWithStartArea) {
        Geometry clipArea = clipWithStartArea ? getStartClipArea() : getEndClipArea();
        Geometry clippedFlowLineGeometry = lineString.difference(clipArea);
        double d = 0;
        Iterator geometryIterator = new GeometryCollectionIterator(clippedFlowLineGeometry);
        while (geometryIterator.hasNext()) {
            Geometry geometry = (Geometry) geometryIterator.next();
            if (geometry instanceof LineString) {
                LineString l = (LineString) geometry;
                if (l.getNumPoints() >= 2) {
                    com.vividsolutions.jts.geom.Point linePoint
                            = clipWithStartArea ? l.getStartPoint() : l.getEndPoint();
                    Point flowPoint = clipWithStartArea ? startPt : endPt;
                    double dx = flowPoint.x - linePoint.getX();
                    double dy = flowPoint.y - linePoint.getY();
                    double dist = Math.sqrt(dx * dx + dy * dy);
                    if (dist > d) {
                        d = dist;
                    }
                }
            }
        }
        return clipWithStartArea
                ? getIntersectionTWithCircleAroundStartPoint(d)
                : getIntersectionTWithCircleAroundEndPoint(d);
    }

    /**
     * Computes the shortest distance between a point and any point on this
     * quadratic BŽzier curve. Attention: xy parameter is changed.
     *
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @return The distance.
     */
    public double distance(double[] xy) {
        return GeometryUtils.getDistanceToQuadraticBezierCurve(startPt.x, startPt.y,
                cPt.x, cPt.y, endPt.x, endPt.y, xy);
    }

    /**
     * Computes the square of the shortest distance between a point and any
     * point on this quadratic BŽzier curve. Attention: xy parameter is changed.
     *
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @return The distance.
     */
    public double distanceSq(double[] xy) {
        return GeometryUtils.getDistanceToQuadraticBezierCurveSq(startPt.x, startPt.y,
                cPt.x, cPt.y, endPt.x, endPt.y, xy);
    }

    public double getDistanceBetweenStartPointAndControlPoint() {
        double dx = cPt.x - startPt.x;
        double dy = cPt.y - startPt.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getDistanceBetweenEndPointAndControlPoint() {
        double dx = cPt.x - endPt.x;
        double dy = cPt.y - endPt.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double[] getDirectionVectorFromStartPointToControlPoint() {
        double dx = cPt.x - startPt.x;
        double dy = cPt.y - startPt.y;
        double d = Math.sqrt(dx * dx + dy * dy);
        return new double[]{dx / d, dy / d};
    }

    public double[] getDirectionVectorFromEndPointToControlPoint() {
        double dx = cPt.x - endPt.x;
        double dy = cPt.y - endPt.y;
        double d = Math.sqrt(dx * dx + dy * dy);
        return new double[]{dx / d, dy / d};
    }

    /**
     * Orientation of the line between the start point and the control point.
     *
     * @return Angle in radians relative to horizontal x axis in
     * counter-clockwise direction.
     */
    public double startToCtrlAngle() {
        double dx = cPt.x - startPt.x;
        double dy = cPt.y - startPt.y;
        return Math.atan2(dy, dx);
    }

    /**
     * Orientation of the line between the end point and the control point.
     *
     * @return Angle in radians relative to horizontal x axis in
     * counter-clockwise direction.
     */
    public double endToCtrlAngle() {
        double dx = cPt.x - endPt.x;
        double dy = cPt.y - endPt.y;
        return Math.atan2(dy, dx);
    }

    /**
     * @return the startClipAreaWKT
     */
    public String getStartClipAreaWKT() {
        return startClipAreaWKT;
    }

    /**
     * @return the endClipAreaWKT
     */
    public String getEndClipAreaWKT() {
        return endClipAreaWKT;
    }
}
