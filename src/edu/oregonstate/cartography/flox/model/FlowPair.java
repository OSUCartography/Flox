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
     * @return 
     */
    @Override
    public double getValue() {
        return super.getValue() + hiddenFlow.getValue();
    }

    public Flow createFlow1(Model model) {
        double value = super.getValue();
        double lineWidth = model.getFlowWidthPx(value) / model.getReferenceMapScale();
        Point cPt = flowCtrlPt(lineWidth, 5); // FIXME

        Flow flow = new Flow(startPt, cPt, endPt, value, id);
        flow.setLocked(isLocked());
        flow.setStartClipArea(getStartClipArea());
        flow.setEndClipArea(getEndClipArea());
        return flow;
    }

    public Flow createFlow2(Model model) {
        double value = hiddenFlow.getValue();
        double lineWidth = model.getFlowWidthPx(value) / model.getReferenceMapScale();
        Point cPt = flowCtrlPt(lineWidth, -5); // FIXME
        Point pt1 = new Point(endPt.x, endPt.y);
        Point pt2 = new Point(startPt.x, startPt.y);

        Flow flow = new Flow(pt1, cPt, pt2, value, hiddenFlow.id);
        flow.setLocked(isLocked());
        flow.setStartClipArea(getEndClipArea());
        flow.setEndClipArea(getStartClipArea());
        return flow;
    }

    private Point flowCtrlPt(double lineWidth, double s) {

        double[] dir = getDirectionVectorFromBaseLineMidPointToControlPoint();

        // FIXME also handle situations where control point is close to base line
        // handle control point on line between start point and end point
        if (dir == null) {
            dir = getDirectionVectorFromStartPointToControlPoint();
            double temp = dir[0];
            dir[0] = -dir[1];
            dir[1] = temp;
        }
        Point cPt = new Point(getCtrlPt().x, getCtrlPt().y);
        cPt.x += dir[0] * lineWidth * s;
        cPt.y += dir[1] * lineWidth * s;
        return cPt;
    }
}
