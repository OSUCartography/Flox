package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollectionIterator;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.io.WKTWriter;
import static edu.oregonstate.cartography.flox.gui.FloxRenderer.NODE_STROKE_WIDTH;
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
     * The Arrow at the end of the flow, points towards endPt
     */
    private final Arrow endArrow = new Arrow(this);

    /**
     * Construct a Flow from 3 points.
     *
     * @param startPt start point
     * @param ctrlPt control point
     * @param endPt end point
     * @param value flow value
     */
    public Flow(Point startPt, Point ctrlPt, Point endPt, double value) {
        this.startPt = startPt;
        this.cPt = ctrlPt;
        this.endPt = endPt;
        this.value = value;
    }

    /**
     * Construct a Flow from a start point and an end point.
     *
     * @param startPt Start point
     * @param endPt End point
     * @param value value of this flow
     */
    public Flow(Point startPt, Point endPt, double value) {

        this.startPt = startPt;
        this.endPt = endPt;
        this.value = value;

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
     * Change the flow value. <STRONG>Important: Must be followed by a call to
     * Graph.updateCachedValues().</STRONG>
     *
     * @param value the value to set
     */
    protected void setValue(double value) {
        this.value = value;
    }

    /**
     *
     * /**
     * Computes geometry of arrow heads
     *
     * @param model model
     * @param flowStrokeWidth width of flow in world units.
     * @param endClipRadius the tip of the arrow is placed at this distance from
     * the end of the flow
     */
    public void computeArrowhead(Model model, double flowStrokeWidth, double endClipRadius) {
        endArrow.computeArrowhead(model, flowStrokeWidth, endClipRadius);
    }

    public Arrow getEndArrow() {
        return endArrow;
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
     * Returns the slope at t.
     *
     * @param t t parameter, must be within 0 and 1.
     * @return slope in radians
     */
    public double getSlope(double t) {
        double dx = (1 - t) * (cPt.x - startPt.x) + t * (endPt.x - cPt.x);
        double dy = (1 - t) * (cPt.y - startPt.y) + t * (endPt.y - cPt.y);
        return Math.atan2(dy, dx);
    }

    /**
     * Offsets this flow in parallel to the line.
     *
     * http://pomax.github.io/bezierinfo/#offsetting
     *
     * @param d offset distance
     */
    public void offsetFlow(double d) {
        // normal at start
        double dxStart = cPt.x - startPt.x;
        double dyStart = cPt.y - startPt.y;
        double lStart = Math.sqrt(dxStart * dxStart + dyStart * dyStart);
        double nxStart = -dyStart / lStart;
        double nyStart = dxStart / lStart;

        // normal at end
        double dxEnd = endPt.x - cPt.x;
        double dyEnd = endPt.y - cPt.y;
        double lEnd = Math.sqrt(dxEnd * dxEnd + dyEnd * dyEnd);
        double nxEnd = -dyEnd / lEnd;
        double nyEnd = dxEnd / lEnd;

        // offset start point
        startPt.x += nxStart * d;
        startPt.y += nyStart * d;

        // offset end point
        endPt.x += nxEnd * d;
        endPt.y += nyEnd * d;

        // offset control point
        cPt.x += (nxStart + nxEnd) / 2 * d;
        cPt.y += (nyStart + nyEnd) / 2 * d;
    }

    /**
     * Returns an array of flows that are offset.
     *
     * http://pomax.github.io/bezierinfo/#offsetting
     *
     * @param d offset distance
     * @return a new offset flow
     */
    public Flow[] splitAndOffsetFlow(double d) {
        // FIXME adjust segmentation to curvature of flow.
        // http://pomax.github.io/bezierinfo/#offsetting suggests testing whether 
        // the control point is close to center of triangle defined by the 
        // three Bezier points
        Flow[] flows = split(0.5);
        Flow[] flows12 = flows[0].split(0.5);
        Flow[] flows34 = flows[1].split(0.5);
        flows = new Flow[]{flows12[0], flows12[1], flows34[0], flows34[1]};
        for (Flow flow : flows) {
            flow.offsetFlow(d);
        }
        return flows;
    }

    /**
     * Constructs a GeneralPath object for drawing a Flow and optionally offsets
     * the path parallel to its direction.
     *
     * @param scale scale factor for converting from world to pixel coordinates
     * @param west horizontal origin in world coordinates
     * @param north vertical origin in world coordinates
     * @param offset parallel offset in world coordinates
     * @return A GeneralPath for drawing in pixel coordinates.
     */
    public GeneralPath toGeneralPath(double scale, double west, double north, double offset) {
        Flow[] flows = (offset == 0d) ? new Flow[]{this} : splitAndOffsetFlow(offset);
        GeneralPath path = new GeneralPath();
        for (int i = 0; i < flows.length; i++) {
            Flow flow = flows[i];
            if (i == 0) {
                Point pt0 = flow.getStartPt();
                path.moveTo((pt0.x - west) * scale, (north - pt0.y) * scale);
            }
            Point pt1 = flow.getCtrlPt();
            Point pt2 = flow.getEndPt();
            path.quadTo((pt1.x - west) * scale, (north - pt1.y) * scale,
                    (pt2.x - west) * scale, (north - pt2.y) * scale);
        }
        return path;
    }

    /**
     * Constructs a GeneralPath object for drawing a Flow.
     *
     * @param scale scale factor for converting from world to pixel coordinates
     * @param west horizontal origin in world coordinates
     * @param north vertical origin in world coordinates
     * @return A GeneralPath for drawing in pixel coordinates.
     */
    public GeneralPath toGeneralPath(double scale, double west, double north) {
        return toGeneralPath(scale, west, north, 0);
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

    public static LineString pointsToLineString(ArrayList<Point> points) {
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
     * clipping with start and end nodes, mask areas, or arrowheads.
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

        // create new point set with regularly distributed points
        double startX = irregularPoints.get(0).x;
        double startY = irregularPoints.get(0).y;

        // add start point
        regularPoints.add(new Point(startX, startY));

        double length = 0;
        int nPoints = irregularPoints.size();
        for (int i = 1; i < nPoints; i++) {
            Point inputPt = irregularPoints.get(i);
            double endX = inputPt.x;
            double endY = inputPt.y;

            // normalized direction dx and dy
            double dx = endX - startX;
            double dy = endY - startY;
            final double l = Math.sqrt(dx * dx + dy * dy);

            double rest = length;
            length += l;
            while (length >= targetDist) {
                // compute new point
                length -= targetDist;
                startX += dx / l * (targetDist - rest);
                startY += dy / l * (targetDist - rest);
                rest = 0;
                regularPoints.add(new Point(startX, startY));
            }
            startX = endX;
            startY = endY;
        }

        // replace last point with end point
        regularPoints.set(regularPoints.size() - 1,
                irregularPoints.get(irregularPoints.size() - 1));
        
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

        Flow flow1 = new Flow(start1, ctrl1, end1, getValue());
        flow1.setSelected(isSelected());
        flow1.setLocked(isLocked());
        flow1.setStartClipArea(getStartClipArea());

        Flow flow2 = new Flow(start2, ctrl2, end2, getValue());
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
        // FIXME need to handle the case when the entire curve is within the circle with radius r
        // comparing r to the distance between start and end node is not sufficient to handle this case.

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
        // FIXME need to handle the case when the entire curve is within the circle with radius r
        // comparing r to the distance between start and end node is not sufficient to handle this case.
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

    private LineString toLineStringIfClipAreaIsAttached(Model model) {
        // construct LineString from the current Bezier flow geometry if there is
        // a mask area attached to the start node or the end node
        if (getEndClipArea() != null || getStartClipArea() != null) {
            double deCasteljauTol = model.getDeCasteljauTolerance();
            ArrayList<Point> points = toUnclippedStraightLineSegments(deCasteljauTol);
         return Flow.pointsToLineString(points);
        }
        return null;
    }

    /**
     * Returns radii of circles around start and end nodes that can be used to
     * clip the flows. The radii take the node size, gap around nodes, and mask
     * areas and optionally the arrowhead into account. The radii
     * only take the nodes into account, if there is gap between the line and
     * the nodes.
     *
     * @param model model with clipping distances
     * @param clipArrowhead if true, the flow line is clipped to make space for
     * an arrowhead
     * @return clip radii around start and end nodes, first start, then end
     * point
     */
    public double[] clipRadii(Model model, boolean clipArrowhead) {
        
        LineString lineString = toLineStringIfClipAreaIsAttached(model);

        // clipping radius for start node
        double startNodeClipRadius = 0;
        // clip the start if there must be a gap between the start of 
        // the flow line and the start node symbol.
        // distance between start of flow and start node
        double gap = model.getFlowDistanceFromStartPointPixel();
        if (gap > 0) {
            // Compute the radius of the start node (add half stroke width)
            double startNodeRadiusPx = NODE_STROKE_WIDTH / 2 + model.getNodeRadiusPx(getStartPt());
            startNodeClipRadius = (gap + startNodeRadiusPx) / model.getReferenceMapScale();
        }

        // clipping radius for start mask area
        double startMaskClipRadius = 0;
        if (getStartClipArea() != null) {
            startMaskClipRadius = maskClippingRadius(lineString, true);
        }

        // start and end clipping radius
        double startR = Math.max(startNodeClipRadius, startMaskClipRadius);
        double endR = endClipRadius(model, clipArrowhead, lineString);
        return new double[]{startR, endR};
    }
    
    public double endClipRadius(Model model, boolean clipArrowhead, LineString lineString) {
        // clipping radius for end node
        double endNodeClipRadius = 0;
        if (clipArrowhead && model.isDrawArrowheads()) {
            endNodeClipRadius = getEndArrow().getClipRadius();
        } else if (model.getFlowDistanceFromEndPointPixel() > 0 || model.isDrawArrowheads()) {
            // clip the end if there must be a gap between the end of the 
            // flow line and the end node symbol.
            double gapDistanceToEndNodesPx = model.getFlowDistanceFromEndPointPixel();
            // Compute the radius of the end node (add stroke width / 2 to radius)
            double endNodeRadiusPx = NODE_STROKE_WIDTH / 2 + model.getNodeRadiusPx(endPt);
            endNodeClipRadius =  (gapDistanceToEndNodesPx + endNodeRadiusPx) / model.getReferenceMapScale();
        }

        // clipping radius for end mask area
        double endMaskClipRadius = 0;
        if (getEndClipArea() != null) {
            if (lineString == null) {
                lineString = toLineStringIfClipAreaIsAttached(model);
            }
            endMaskClipRadius = maskClippingRadius(lineString, false);
        }

        // end clipping radius
        return Math.max(endNodeClipRadius, endMaskClipRadius);
    }

    /**
     * Computes the clipping radius for the location where the flow needs to be
     * split for masking with the start or the end clip area.
     *
     * @param lineString This flow's geometry converted to straight line
     * segments.
     * @param clipWithStartArea If true, clip with start clip area, otherwise
     * clip with the end clip area.
     * @return The parameter t in [0, 1]
     */
    private double maskClippingRadius(LineString lineString, boolean clipWithStartArea) {
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
        return d;
    }

    /**
     * Computes the shortest distance between a point and any point on this
     * quadratic BŽzier curve. Attention: xy parameter is changed.
     *
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @return The distance.
     */
    public double distance(double[] xy, double tol) {
        return GeometryUtils.getDistanceToQuadraticBezierCurve(startPt.x, startPt.y,
                cPt.x, cPt.y, endPt.x, endPt.y, tol, xy);
    }

    /**
     * Computes the square of the shortest distance between a point and any
     * point on this quadratic BŽzier curve. Attention: xy parameter is changed.
     *
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @return The distance.
     */
    public double distanceSq(double[] xy, double tol) {
        return GeometryUtils.getDistanceToQuadraticBezierCurveSq(startPt.x, startPt.y,
                cPt.x, cPt.y, endPt.x, endPt.y, tol, xy);
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

    /**
     * Returns true if this and the passed flow share a common start or end node.
     * @param flow flow to test
     * @return True if start or end nodes are shared, false otherwise.
     */
    boolean isSharingStartOrEndNode(Flow flow) {
        return startPt == flow.startPt || startPt == flow.endPt
                || endPt == flow.startPt || endPt == flow.endPt;
    }
}
