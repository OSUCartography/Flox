package edu.oregonstate.cartography.flox.model;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

/**
 * Two flows with opposing directions connected to the same start node and end
 * node.
 *
 * @author Bernhard Jenny, School of Science, RMIT University, Melbourne
 */
/**
 * Every non static, non transient field in a JAXB-bound class will be
 * automatically bound to XML, unless annotated by @XmlTransient
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class FlowPair extends Flow {

    /**
     * Value of the second flow of this pair.
     */
    private final double value2;

    /**
     * A cached approximation of the flow geometry of the second flow by a
     * straight polyline to avoid repeated expensive conversions to a polyline.
     *
     * <STRONG>The polyline is not initialized or updated by this FlowPair. It
     * is the responsibility of the user to update the polyline when any of the
     * following change: start point, end point, control point, startClipArea,
     * endClipArea, size of nodes, gap between start/end of line and
     * nodes.</STRONG> A change to the arrowhead geometry does not require an
     * update to the polyline.
     */
    private Point[] polyline2;

    /**
     * Constructor without parameters for JAXB.
     */
    private FlowPair() {
        value2 = 0;
    }

    /**
     * Constructor
     *
     * @param flow1 flow connected to same start and end nodes as flow2, but in
     * opposite direction to flow2. flow1 cannot be locked.
     * @param flow2 flow connected to same start and end nodes as flow1, but in
     * opposite direction to flow1. flow2 cannot be locked.
     */
    public FlowPair(Flow flow1, Flow flow2) {
        // create a copy of flow1
        super(flow1.getStartPt(),
                // mean of the two control points
                (flow1.cPtX() + flow2.cPtX()) / 2, (flow1.cPtY() + flow2.cPtY()) / 2,
                flow1.getEndPt(),
                flow1.getValue());
        setStartClipArea(flow1.getStartClipArea());
        setEndClipArea(flow1.getEndClipArea());

        assert (flow1.isLocked() == false);
        assert (flow2.isLocked() == false);

        // store value of flow 2
        value2 = flow2.getValue();
    }

    /**
     * Copy constructor
     *
     * @param flowPair a FlowPair to copy
     */
    public FlowPair(FlowPair flowPair) {
        super(new Point(flowPair.getStartPt()),
                flowPair.cPtX(), flowPair.cPtY(),
                new Point(flowPair.getEndPt()),
                flowPair.getValue1());

        shallowCopyClipAreas(flowPair, this);
        setSelected(flowPair.isSelected());
        setLocked(flowPair.isLocked());
        value2 = flowPair.value2;
    }

    /**
     * Returns a copy of this FlowPair. The id of the new flow is unique.
     *
     * @return a copy
     */
    @Override
    public FlowPair copyFlow() {
        return new FlowPair(this);
    }

    /**
     * Overridden getValue returns sum of the two flow values of this FlowPair.
     *
     * @return
     */
    @Override
    public double getValue() {
        return getValue1() + getValue2();
    }

    /**
     * Value cannot be set for a FlowPair. An exception is thrown when this
     * method is called.
     *
     * @param value
     */
    @Override
    protected void setValue(double value) {
        throw new UnsupportedOperationException();
    }

    /**
     * Returns the value of the first flow of this FlowPair.
     *
     * @return the value
     */
    public double getValue1() {
        return super.getValue();
    }

    /**
     * Returns the value of the second flow of this flow pair.
     *
     * @return the value
     */
    public double getValue2() {
        return value2;
    }

    public Point[] getCachedPolylineApproximation2() {
        return polyline2;
    }

    /**
     * Updates the two cached polylines. The polylines are not initialized or
     * updated by this Flow. It is the responsibility of the user to update the
     * polylines when any of the following change: start point, end point,
     * control point, startClipArea, endClipArea, size of nodes, gap between
     * start/end of line and nodes. A change to the arrowhead geometry does not
     * require an update to the polylines.
     *
     * @param model data model
     * @param segmentLength the target segment length. The actual length will
     * differ.
     */
    @Override
    public void updateCachedPolylineApproximation(Model model, double segmentLength) {
        // FIXME need option to either use central line or offset lines
        Flow flow1 = createParallelFlow1(model);
        Flow flow2 = createParallelFlow2(model);
        flow1.updateCachedPolylineApproximation(model, segmentLength);
        flow2.updateCachedPolylineApproximation(model, segmentLength);
        this.polyline = flow1.polyline;
        this.polyline2 = flow2.polyline;
    }

    /**
     * Tests whether this FlowPair intersects with another Flow. This is an
     * approximate test.
     *
     * <STRONG>updateCachedPolylineApproximation() needs to be called before
     * intersects() can be called.</STRONG>
     *
     * @param flow flow to detect intersection with
     * @return true if the two flows intersect
     */
    @Override
    public boolean intersects(Flow flow) {
        return flow.intersects(this);
    }

    /**
     * Tests whether this FlowPair intersects with another FlowPair. This is an
     * approximate test.
     *
     * <STRONG>updateCachedPolylineApproximation() needs to be called before
     * this method can be called.</STRONG>
     *
     * @param flowPair FlowPair to detect intersection with
     * @return true if the two flows intersect
     */
    @Override
    public boolean intersects(FlowPair flowPair) {
        Point[] thisPolyline1 = getCachedPolylineApproximation();
        Point[] thatPolyline1 = flowPair.getCachedPolylineApproximation();
        if (polylinesIntersect(thisPolyline1, thatPolyline1)) {
            return true;
        }
        Point[] thatPolyline2 = flowPair.getCachedPolylineApproximation2();
        if (polylinesIntersect(thisPolyline1, thatPolyline2)) {
            return true;
        }
        Point[] thisPolyline2 = getCachedPolylineApproximation2();
        if (polylinesIntersect(thisPolyline2, thatPolyline1)) {
            return true;
        }
        return polylinesIntersect(thisPolyline2, thatPolyline2);
    }

    /**
     * Computes the offset in world coordinates for either of the two flows. The
     * offset is such that the sum of width of the two flows and the gap
     * in-between the two flows is centered on the axis of this flow.
     *
     * @param model model
     * @param forFlow1 true for flow 1, false for flow 2.
     * @return offset in world coordinates
     */
    private double offset(Model model, boolean forFlow1) {
        double value1 = getValue1();
        double width1 = model.getFlowWidthPx(value1);
        double width2 = model.getFlowWidthPx(value2);
        double gap = model.getParallelFlowsGapPx();
        double totalWidth = width1 + width2 + gap;
        if (forFlow1) {
            return (totalWidth - width1) / 2 / model.getReferenceMapScale();
        } else {
            return (totalWidth - width2) / 2 / model.getReferenceMapScale();
        }
    }

    /**
     * Returns a new instance of the Flow class that can be used to draw the
     * first of the two flows of this FlowPair. The returned flow is offset from
     * the center line of this flow. <STRONG>The start node and end node of the
     * returned flow do not align with the nodes of this FlowPair. The nodes
     * have been moved to create a nice parallel line.</STRONG>
     *
     * @param model data model
     * @return a new flow
     */
    public Flow createParallelFlow1(Model model) {
        Flow flow = new Flow(this);
        flow.setValue(getValue1());
        flow.offsetFlow(offset(model, true), model);
        return flow;
    }

    /**
     * Returns a new instance of the Flow class that can be used to draw the
     * second of the two flows of this FlowPair. The returned flow is offset
     * from the center line of this flow. <STRONG>The start node and end node of
     * the returned flow do not align with the nodes of this FlowPair. The nodes
     * have been moved to create a nice parallel line.</STRONG>
     *
     * @param model data model
     * @return a new flow
     */
    public Flow createParallelFlow2(Model model) {
        Flow flow = new Flow(this);
        flow.setValue(getValue2());
        flow.reverseFlow(model);
        flow.offsetFlow(offset(model, false), model);
        return flow;
    }

    /**
     * Returns a new instance of the Flow class representing the first flow of
     * this FlowPair. The returned flow is not offset from the center line of
     * this flow.
     *
     * @return a new flow
     */
    public Flow createFlow1() {
        Flow flow = new Flow(this);
        flow.setValue(getValue1());
        return flow;
    }

    /**
     * Returns a new instance of the Flow class representing the second flow of
     * this FlowPair. The returned flow is not offset from the center line of
     * this flow.
     *
     * @param model data model
     * @return a new flow
     */
    public Flow createFlow2(Model model) {
        Flow flow = new Flow(this);
        flow.setValue(getValue2());
        flow.reverseFlow(model);
        return flow;
    }

}
