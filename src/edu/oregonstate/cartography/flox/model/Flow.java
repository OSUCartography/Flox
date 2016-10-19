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
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.adapters.XmlJavaTypeAdapter;

/**
 * A flow based on a quadratic Bézier curve.
 *
 * @author Bernhard Jenny
 * @author Daniel Stephen
 */
//Every non static, non transient field in a JAXB-bound class will be 
//automatically bound to XML, unless annotated by @XmlTransient
@XmlAccessorType(XmlAccessType.FIELD)

public class Flow {

    private static long idCounter = 0;

    protected static synchronized long createID() {
        return idCounter++;
    }

    /**
     * An identifier that is used to find Flows in a Graph. The id is not
     * required to be unique. For example, when a flow is split in two flows
     * both new flows have the same id as the original flow. However, a Graph
     * cannot contain multiple flows with identical IDs.
     */
    public final long id;

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
    @XmlJavaTypeAdapter(GeometrySerializer.class)
    private Geometry startClipArea;

    /**
     * startClipArea serialized to WKT format
     */
    private String startClipAreaWKT;
    /**
     * clip area for the end of the flow.
     */
    @XmlJavaTypeAdapter(GeometrySerializer.class)
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
     * @param startPt start point
     * @param ctrlPt control point
     * @param endPt end point
     * @param value flow value
     * @param id id for this flow
     */
    public Flow(Point startPt, Point ctrlPt, Point endPt, double value, long id) {
        assert (startPt != null);
        assert (endPt != null);
        assert (Double.isFinite(value));

        if (startPt.equals(endPt)) {
            throw new IllegalArgumentException("The start and end node of a flow cannot be identical.");
        }

        this.startPt = startPt;
        this.cPt = ctrlPt;
        this.endPt = endPt;
        this.value = value;
        this.id = id;
    }

    /**
     * Construct a Flow from a start point and an end point.
     *
     * @param startPt Start point
     * @param endPt End point
     * @param value value of this flow
     */
    public Flow(Point startPt, Point endPt, double value) {
        this(startPt, new Point((startPt.x + endPt.x) / 2, (startPt.y + endPt.y) / 2), endPt, value, createID());
    }

    /**
     * Default constructor for JAXB
     */
    public Flow() {
        this(new Point(), new Point(), Model.DEFAULT_FLOW_VALUE);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("Flow ");
        sb.append("id=").append(id);
        sb.append(" start=").append(startPt);
        sb.append(", end=").append(endPt);
        sb.append(", ctrl=").append(cPt);
        sb.append(", value=").append(value);
        sb.append(", selected=").append(selected);
        sb.append(", locked=").append(locked);
        sb.append(", start point hash code=").append(startPt.hashCode());
        sb.append(", end point hash code=").append(endPt.hashCode());
        return sb.toString();
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
     * If the start point is passed, returns the end point, and vice versa.
     *
     * @param point start or end point of this flow
     * @return opposite point or null if neither the start nor the end point was
     * passed.
     */
    public Point getOppositePoint(Point point) {
        if (point == startPt) {
            return endPt;
        } else if (point == endPt) {
            return startPt;
        }
        return null;
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
        assert (Double.isFinite(value));
        this.value = value;
    }

    /**
     * Computes geometry of arrow heads
     *
     * @param model model
     * @param endClipRadius the tip of the arrow is placed at this distance from
     * the end of the flow
     * @return a new Arrow instance
     */
    public Arrow getArrow(Model model, double endClipRadius) {
        return new Arrow(model, this, endClipRadius);
    }

    public Arrow getArrow(Model model) {
        double endClipRadius = endClipRadius(model, false, null);
        return new Arrow(model, this, endClipRadius);
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
    public final void setStartClipArea(Geometry startClipArea) {
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
    public final void setEndClipArea(Geometry endClipArea) {
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
    public final void setLocked(boolean locked) {
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
     * Returns a bounding box containing the curve. The control point can be
     * outside of the bounding box returned by this method. Does not take any
     * line width into account. Based on
     * http://pomax.github.io/bezierinfo/#boundingbox
     *
     * @return a new rectangular bounding box.
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
        assert (t >= 0 && t <= 1);
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
        assert (Double.isFinite(d));

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
     * Converts the Bezier curve to segments of straight line. The segments are
     * of irregular length. Uses standard Java2D functionality.
     *
     * @param deCasteljauTol the de Casteljau tolerance, which is the maximum
     * distance between the Bezier curve and the approximation by straight line
     * segments.
     * @return
     */
    public ArrayList<Point> toStraightLineSegmentsWithIrregularLength(
            double deCasteljauTol) {
        assert (deCasteljauTol > 0);

        ArrayList<Point> irregularPoints = new ArrayList<>();
        GeneralPath path = new GeneralPath();
        path.moveTo(startPt.x, startPt.y);
        path.quadTo(cPt.x, cPt.y, endPt.x, endPt.y);
        PathIterator iter = path.getPathIterator(null, deCasteljauTol);
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
     * Returns the location on the Bézier curve at parameter value t.
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

        Flow flow1 = new Flow(start1, ctrl1, end1, getValue(), id);
        flow1.setSelected(isSelected());
        flow1.setLocked(isLocked());
        flow1.setStartClipArea(getStartClipArea());

        Flow flow2 = new Flow(start2, ctrl2, end2, getValue(), id);
        flow2.setSelected(isSelected());
        flow2.setLocked(isLocked());
        flow2.setEndClipArea(getEndClipArea());

        return new Flow[]{flow1, flow2};
    }

    /**
     * Returns the curve parameter where a circle with radius r around the end
     * point intersects the Bézier curve.
     *
     * @param r Radius of circle
     * @return Parameter t [0..1] where the circle intersects the flow.
     */
    public final double getIntersectionTWithCircleAroundEndPoint(double r) {
        if (r <= 0) {
            return 1;   // t = 1: end of curve
        }

        final double rSqr = r * r;

        // Test whether the entire curve is within the circle with radius r.
        // Compare r to the distance between start and end node.
        double baseLineDx = startPt.x - endPt.x;
        double baseLineDy = startPt.y - endPt.y;
        double baseLineLengthSqr = baseLineDx * baseLineDx + baseLineDy * baseLineDy;
        if (baseLineLengthSqr <= rSqr) {
            return 1; // t = 1: end of curve
        }

        // FIXME should use distance tolerance instead of hard-coded number of iterations
        double t = 0.5;
        double t_step = 0.25;
        for (int i = 0; i < 20; i++) {
            Point pt = pointOnCurve(t);
            final double dx = endPt.x - pt.x;
            final double dy = endPt.y - pt.y;
            final double dSqr = dx * dx + dy * dy;
            if (dSqr < rSqr) {
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
     * point intersects the Bézier curve.
     *
     * @param r Radius of circle
     * @return Parameter t [0..1] where the circle intersects the flow.
     */
    public double getIntersectionTWithCircleAroundStartPoint(double r) {
        if (r <= 0) {
            return 0;   // t = 0: start of curve
        }

        final double rSqr = r * r;

        // Test whether the entire curve is within the circle with radius r.
        // Compare r to the distance between start and end node.
        double baseLineDx = startPt.x - endPt.x;
        double baseLineDy = startPt.y - endPt.y;
        double baseLineLengthSqr = baseLineDx * baseLineDx + baseLineDy * baseLineDy;
        if (baseLineLengthSqr <= rSqr) {
            return 0; // t = 1: start of curve
        }

        // FIXME should use distance tolerance instead of hard-coded number of iterations
        double t = 0.5;
        double t_step = 0.25;
        for (int i = 0; i < 20; i++) {
            Point pt = pointOnCurve(t);
            final double dx = startPt.x - pt.x;
            final double dy = startPt.y - pt.y;
            final double dSqr = dx * dx + dy * dy;
            if (dSqr < rSqr) {
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
            double segmentLength = model.segmentLength();
            ArrayList<Point> points = regularIntervals(segmentLength);
            return Flow.pointsToLineString(points);
        }
        return null;
    }

    /**
     * Returns radii of circles around start and end nodes that can be used to
     * clip the flows. The radii take the node size, gap around nodes, and mask
     * areas and optionally the arrowhead into account. The radii only take the
     * nodes into account, if there is gap between the line and the nodes.
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
            double startNodeRadiusPx = model.getNodeStrokeWidthPx() / 2 + model.getNodeRadiusPx(getStartPt());
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

    /**
     * Computes clipping radius around end node.
     *
     * @param model model
     * @param clipArrowhead if true, the circles with the returned radius
     * includes the arrowhead (if arrowheads are drawn).
     * @param lineString the geometry of this flow converted to straight line
     * segments.
     * @return radius of circle around end node
     */
    public double endClipRadius(Model model, boolean clipArrowhead, LineString lineString) {
        // clipping radius for end node
        double endNodeClipRadius = 0;
        if (clipArrowhead && model.isDrawArrowheads()) {
            // Compute radius of clipping circle around end point without taking
            // the arrow into account. This is a recursive call with 
            // clipArrowhead flag set to false.
            double arrowTipClipRadius = endClipRadius(model, false, lineString);

            // Create an arrowhead
            Arrow arrow = getArrow(model, arrowTipClipRadius);

            // get clip radius including arrowhead
            endNodeClipRadius = arrow.getClipRadius();
        } else if (model.getFlowDistanceFromEndPointPixel() > 0 || model.isDrawArrowheads()) {
            // clip the end if there must be a gap between the end of the 
            // flow line and the end node symbol.
            double gapDistanceToEndNodesPx = model.getFlowDistanceFromEndPointPixel();
            // Compute the radius of the end node (add stroke width / 2 to radius)
            double endNodeRadiusPx = model.getNodeStrokeWidthPx() / 2 + model.getNodeRadiusPx(endPt);
            endNodeClipRadius = (gapDistanceToEndNodesPx + endNodeRadiusPx) / model.getReferenceMapScale();
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
     * Computes the clipping radius around the start or end node. The circle
     * intersects the passed lineString where the start or end clip area
     * intersects the lineString.
     *
     * @param lineString the geometry of this flow converted to straight line
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
     * quadratic Bézier curve. Attention: xy parameter is changed.
     *
     * @param xy Point x and y on input; the closest point on the curve on
     * output.
     * @param tol Tolerance to test whether points are collinear.
     * @return The distance.
     */
    public double distance(double[] xy, double tol) {
        return GeometryUtils.getDistanceToQuadraticBezierCurve(startPt.x, startPt.y,
                cPt.x, cPt.y, endPt.x, endPt.y, tol, xy);
    }

    /**
     * Computes the square of the shortest distance between a point and any
     * point on this quadratic Bézier curve. Attention: xy parameter is changed.
     *
     * @param x point x
     * @param y point y
     * @param tol Tolerance to test whether points are collinear.
     * @return the distance
     */
    public double distanceSq(double x, double y, double tol) {
        return GeometryUtils.getDistanceToQuadraticBezierCurveSq(startPt.x, startPt.y,
                cPt.x, cPt.y, endPt.x, endPt.y, tol, x, y);
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
     * If this and the passed flow share a common start or end node, the shared
     * node is returned. Otherwise null is returned.
     *
     * @param flow flow to test.
     * @return the shared node or null.
     */
    public Point getSharedNode(Flow flow) {
        if (startPt == flow.startPt || startPt == flow.endPt) {
            return startPt;
        }
        if (endPt == flow.startPt || endPt == flow.endPt) {
            return endPt;
        }
        return null;
    }
    
    /**
     * Returns a list of points at regular intervals on the flow curve.
     *
     * @param intervalLength target interval length. The actual length will vary
     * to create an entire number of intervals.
     * @return list of points, including copies of the start and end points of
     * this flow
     */
    public ArrayList<Point> regularIntervals(double intervalLength) {
        ArrayList<Point> intervalPoints = new ArrayList<>();

        // compute size of lookup table
        // The length of the curve is always shorter or equal to the distance 
        // between the start point and the control point plus the distance 
        // between the control point and the end point. Use this longer distance
        // as an approximation for the real curve length to compute the number
        // of points in the lookup table.
        double d1 = getDistanceBetweenStartPointAndControlPoint();
        double d2 = getDistanceBetweenEndPointAndControlPoint();
        int lutSize = (int) ((d1 + d2) / intervalLength) + 1;
        double[] lut = new double[lutSize];

        // fill lookup table with length values for regularly increasing t value
        double x0 = startPt.x;
        double y0 = startPt.y;
        double lineLength = 0;
        lut[0] = 0;
        for (int i = 1; i < lutSize; i++) {
            // t parameter
            double t = 1d / (lutSize - 1) * i;

            // compute position on curve for t
            double t_1 = t - 1;
            double a = t * t;
            double b = 2 * t * t_1;
            double c = t_1 * t_1;
            double x1 = a * endPt.x - b * cPt.x + c * startPt.x;
            double y1 = a * endPt.y - b * cPt.y + c * startPt.y;

            // distance to previous point. Use Eucledian distance.
            double dx = x0 - x1;
            double dy = y0 - y1;
            lineLength += Math.sqrt(dx * dx + dy * dy);
            lut[i] = lineLength;

            x0 = x1;
            y0 = y1;
        }

        // number and length of intervals
        int nbrIntervals = (int) Math.round(lineLength / intervalLength);
        nbrIntervals = Math.max(nbrIntervals, 2);
        intervalLength = lineLength / nbrIntervals;

        // add start point
        //intervalPoints.add(new Point(startPt.x, startPt.y));

        // add intermediate points
        for (int i = 0; i < nbrIntervals; i++) {
            double distance = (i + 0.5) * intervalLength;
            double t = 1d;

            // find t parameter in lookup table for given distance
            for (int lutID = 1; lutID < lutSize; lutID++) {
                if (lut[lutID] > distance) {
                    double t1 = (lutID - 1d) / (lutSize - 1);
                    double dT = 1d / (lutSize - 1);
                    double l1 = lut[lutID - 1];
                    double l2 = lut[lutID];
                    t = dT * (distance - l1) / (l2 - l1) + t1;
                    break;
                }
            }

            t = Math.max(Math.min(t, 1d), 0d);
            intervalPoints.add(pointOnCurve(t));

        }

        // add end point
        //intervalPoints.add(new Point(endPt.x, endPt.y));

        return intervalPoints;
    }
}
