package edu.oregonstate.cartography.flox.model;

/**
 * Experimental. Not currently used.
 *
 * @author Bernhard Jenny, School of Science, RMIT University, Melbourne
 */
public class FlowPair extends Flow {

    private final Flow hiddenFlow;

    public FlowPair(Flow flow1, Flow flow2) {
        super(flow1.getStartPt(), flow1.getCtrlPt().mean(flow2.getCtrlPt()),
                flow1.getEndPt(), flow1.getValue(),
                // FIXME use id of flow1?
                createID());
        hiddenFlow = flow2;

        // FIXME
//        assert (flow1.getStartPt() == flow2.getEndPt());
//        assert (flow2.getStartPt() == flow1.getEndPt());
//        assert (flow1.getStartClipArea() == flow2.getEndClipArea());
//        assert (flow2.getStartClipArea() == flow1.getEndClipArea());
        setLocked(flow1.isLocked() || flow2.isLocked());
        setStartClipArea(flow1.getStartClipArea());
        setEndClipArea(flow1.getEndClipArea());
    }

    @Override
    public double getValue() {
        return super.getValue() + hiddenFlow.getValue();
    }

    public Flow createFlow1(Model model) {
        Point cPt = flowCtrlPt(model, this, 1);
        double value = super.getValue();
        Flow flow = new Flow(startPt, cPt, endPt, value, id);
        flow.setLocked(isLocked());
        flow.setStartClipArea(getStartClipArea());
        flow.setEndClipArea(getEndClipArea());
        return flow;
    }

    public Flow createFlow2(Model model) {
        Point pt1 = new Point(endPt.x, endPt.y);
        Point cPt = flowCtrlPt(model, hiddenFlow, -1);
        Point pt2 = new Point(startPt.x, startPt.y);
        
        Flow flow = new Flow(pt1, cPt, pt2, hiddenFlow.getValue(), hiddenFlow.id);
        flow.setLocked(isLocked());
        flow.setStartClipArea(getEndClipArea());
        flow.setEndClipArea(getStartClipArea());
        return flow;
    }

    private Point flowCtrlPt(Model model, Flow flow, double s) {
        double lineWidth = model.getFlowWidthPx(flow) / model.getReferenceMapScale();
        double [] dir = getDirectionVectorFromBaseLineMidPointToControlPoint();
        Point cPt = new Point(getCtrlPt().x, getCtrlPt().y);
        cPt.x += dir[0] * lineWidth * s;
        cPt.y += dir[1] * lineWidth * s;
        return cPt;
    }
}
