package edu.oregonstate.cartography.flox;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public abstract class Flow {
    protected Point startPt;
    protected Point endPt;

    /**
     * @return the startPt
     */
    public Point getStartPt() {
        return startPt;
    }

    /**
     * @param startPt the startPt to set
     */
    public void setStartPt(Point startPt) {
        this.startPt = startPt;
    }

    /**
     * @return the endPt
     */
    public Point getEndPt() {
        return endPt;
    }

    /**
     * @param endPt the endPt to set
     */
    public void setEndPt(Point endPt) {
        this.endPt = endPt;
    }
}
