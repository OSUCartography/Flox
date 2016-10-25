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
    private final Flow hiddenFlow;

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

        // FIXME
//        assert (flow1.getStartPt() == flow2.getEndPt());
//        assert (flow2.getStartPt() == flow1.getEndPt());
//        assert (flow1.getStartClipArea() == flow2.getEndClipArea());
//        assert (flow2.getStartClipArea() == flow1.getEndClipArea());
    }

    /**
     * Overridden getValue returns sum of the two flow values of this FlowPair.
     *
     * @return
     */
    @Override
    public double getValue() {
        return super.getValue() + hiddenFlow.getValue();
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
        double value1 = super.getValue();
        double value2 = hiddenFlow.getValue();
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

    public Flow createFlow1(Model model) {
        Flow flow = new Flow(this);
        flow.setValue(super.getValue());
        flow.offsetFlow(offset(model, true), model);
        return flow;
    }

    public Flow createFlow2(Model model) {
        Flow flow = new Flow(this);
        flow.setValue(hiddenFlow.getValue());
        flow.reverseFlow();
        flow.offsetFlow(offset(model, false), model);
        return flow;
    }

}
