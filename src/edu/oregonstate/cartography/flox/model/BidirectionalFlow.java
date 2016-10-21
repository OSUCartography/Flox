package edu.oregonstate.cartography.flox.model;

/**
 * Experimental. Not currently used.
 *
 * @author Bernhard Jenny, School of Science, RMIT University, Melbourne
 */
public class BidirectionalFlow extends Flow {

    private final Flow hiddenFlow;

    public BidirectionalFlow(Flow flow1, Flow flow2) {
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

    public Flow createFlow1() {
        double value = super.getValue();
        Flow flow = new Flow(startPt, getCtrlPt(), endPt, value, id);
        flow.setLocked(isLocked());
        flow.setStartClipArea(getStartClipArea());
        flow.setEndClipArea(getEndClipArea());
        return flow;
    }

    public Flow createFlow2() {
        Point midPt = getBaseLineMidPoint();
        Point cPt = new Point(2 * midPt.x - getCtrlPt().x, 2 * midPt.y - getCtrlPt().y);
        Flow flow = new Flow(endPt, cPt, startPt, hiddenFlow.getValue(), hiddenFlow.id);
        flow.setLocked(isLocked());
        flow.setStartClipArea(getEndClipArea());
        flow.setEndClipArea(getStartClipArea());
        return flow;
    }
}
