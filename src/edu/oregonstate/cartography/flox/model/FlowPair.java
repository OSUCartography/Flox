package edu.oregonstate.cartography.flox.model;

/**
 * Two flows with opposing directions connected to the same start node and end
 * node.
 *
 * @author Bernhard Jenny, School of Science, RMIT University, Melbourne
 */
public class FlowPair extends Flow {

    /**
     * This hidden flow is not part of the graph and will not influence layout
     * computations. hiddenFlow references flow2, which is a constructor
     * argument.
     */
    private final Flow hiddenFlow; // FIXME remove

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
        super(flow1.getStartPt(), flow1.getCtrlPt().mean(flow2.getCtrlPt()),
                flow1.getEndPt(), flow1.getValue(),
                flow1.id);
        setStartClipArea(flow1.getStartClipArea());
        setEndClipArea(flow1.getEndClipArea());

        assert (flow1.isLocked() == false);
        assert (flow2.isLocked() == false);

        // store flow2
        hiddenFlow = flow2;
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
        return hiddenFlow.getValue();
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
        double value2 = getValue2();
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
