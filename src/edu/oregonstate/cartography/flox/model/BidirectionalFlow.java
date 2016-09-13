package edu.oregonstate.cartography.flox.model;

/**
 *
 * @author Bernhard Jenny, School of Mathematical and Geospatial Sciences, RMIT
 * University, Melbourne
 */
public class BidirectionalFlow extends Flow {
    
    private final double value2;
    private final boolean lock1;
    private final boolean lock2;
    
    public BidirectionalFlow(Flow flow1, Flow flow2) {
        super(flow1.getStartPt(), flow1.getCtrlPt(), flow1.getEndPt(), flow1.getValue(), createID());
        
        assert(flow1.getStartPt() == flow2.getEndPt());
        assert(flow2.getStartPt() == flow1.getEndPt());
        assert(flow1.getStartClipArea() == flow2.getEndClipArea());
        assert(flow2.getStartClipArea() == flow1.getEndClipArea());
        
        value2 = flow2.getValue();
        lock1 = flow1.isLocked();
        lock2 = flow2.isLocked();
        setLocked(lock1 || lock2);
        setStartClipArea(flow1.getStartClipArea());
        setEndClipArea(flow1.getEndClipArea());
    }

    @Override
    public double getValue() {
        return super.getValue() + value2;
    }
    
    public Flow createFlow1() {
        Flow flow = new Flow(startPt, getCtrlPt(), endPt, getValue(), id);
        flow.setLocked(lock1);
        flow.setStartClipArea(getStartClipArea());
        flow.setEndClipArea(getEndClipArea());
        return flow;
    }
    
    public Flow createFlow2() {
        Flow flow = new Flow(endPt, getCtrlPt(), startPt, value2, id);
        flow.setLocked(lock2);
        flow.setStartClipArea(getEndClipArea());
        flow.setEndClipArea(getStartClipArea());
        return flow;
    }
}
