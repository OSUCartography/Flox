package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.utils.GeometryUtils;
import java.awt.geom.Rectangle2D;

/**
 *
 * @author Maccabee
 */
public class RangeboxEnforcer {

    private final Model model;

    public RangeboxEnforcer(Model model) {
        this.model = model;
    }

    /**
     * If the control point of a flow falls outside of the flow's range box,
     * this returns the intersection between a line connecting the control point
     * to the midpoint of the baseline, and the location along the rangebox's
     * border where the line crosses. Checks each side of the range rectangle
     * one at a time.
     *
     * @param flow A Flow
     * @return
     */
    public Point enforceFlowControlPointRange(Flow flow) {

        Point cPt = flow.getCtrlPt();
        Point refPt = flow.getBaseLineMidPoint();

        Point[] box = computeRangebox(flow);

        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                box[0].x, box[0].y,
                box[1].x, box[1].y)) {
            return GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    box[0].x, box[0].y,
                    box[1].x, box[1].y);
        }

        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                box[2].x, box[2].y,
                box[3].x, box[3].y)) {
            return GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    box[2].x, box[2].y,
                    box[3].x, box[3].y);
        }

        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                box[0].x, box[0].y,
                box[2].x, box[2].y)) {
            return GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    box[0].x, box[0].y,
                    box[2].x, box[2].y);
        }

        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                box[1].x, box[1].y,
                box[3].x, box[3].y)) {
            return GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    box[1].x, box[1].y,
                    box[3].x, box[3].y);
        }

        return cPt;
    }

    public Point enforceCanvasBoundingBox(Flow flow, Rectangle2D canvas) {

        double cWidth = canvas.getWidth();
        double cHeight = canvas.getHeight();

        // Outer padding of the canvas bounding box
        // Is a percentage of the canvas size
        double xPad = cWidth * model.getCanvasPadding();
        double yPad = cHeight * model.getCanvasPadding();

        // Get the corner points of the canvas
        Point b1 = new Point(canvas.getX() - xPad, canvas.getY() - yPad);
        Point b2 = new Point(canvas.getMaxX() + xPad, canvas.getY() - yPad);
        Point b3 = new Point(canvas.getX() - xPad, canvas.getMaxY() + yPad);
        Point b4 = new Point(canvas.getMaxX() + xPad, canvas.getMaxY() + yPad);

        Point cPt = flow.getCtrlPt();
        Point refPt = flow.getBaseLineMidPoint();

        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                b1.x, b1.y,
                b2.x, b2.y)) {
            return GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    b1.x, b1.y,
                    b2.x, b2.y);
        }

        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                b3.x, b3.y,
                b4.x, b4.y)) {
            return GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    b3.x, b3.y,
                    b4.x, b4.y);
        }

        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                b1.x, b1.y,
                b3.x, b3.y)) {
            return GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    b1.x, b1.y,
                    b3.x, b3.y);
        }

        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                b2.x, b2.y,
                b4.x, b4.y)) {
            return GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    b2.x, b2.y,
                    b4.x, b4.y);
        }

        return cPt;
    }

    public Point[] computeRangebox(Flow flow) {
        double baseDist = flow.getBaselineLength();
        double boxHeight = model.getFlowRangeboxHeight();

        double x1 = flow.startPt.x;
        double y1 = flow.startPt.y;
        double x2 = flow.endPt.x;
        double y2 = flow.endPt.y;
        double dx = x2 - x1;
        double dy = y2 - y1;
        double l = Math.sqrt(dx * dx + dy * dy);

        // unary vector along base line
        double ux = dx / l;
        double uy = dy / l;
        // vector from start and end points of base line to corners
        double vx = -uy * baseDist * boxHeight;
        double vy = ux * baseDist * boxHeight;

        Point bottomLeft = new Point(x1 - vx, y1 - vy);
        Point bottomRight = new Point(x2 - vx, y2 - vy);
        Point topRight = new Point(x2 + vx, y2 + vy);
        Point topLeft = new Point(x1 + vx, y1 + vy);

        Point[] rangeboxPoints = {bottomLeft, bottomRight, topRight, topLeft};
        return rangeboxPoints;
    }

    public boolean isPointInRangebox(Flow flow, double x, double y) {
        double baseDist = flow.getBaselineLength();
        double boxHeight = model.getFlowRangeboxHeight();

        double x1 = flow.startPt.x;
        double y1 = flow.startPt.y;
        double x2 = flow.endPt.x;
        double y2 = flow.endPt.y;
        double dx = x2 - x1;
        double dy = y2 - y1;
        double l = Math.sqrt(dx * dx + dy * dy);

        // unary vector along base line
        double ux = dx / l;
        double uy = dy / l;
        // vector from start and end points of base line to corners
        double vx = -uy * baseDist * boxHeight;
        double vy = ux * baseDist * boxHeight;
        
        // http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
        double Ax = x1 - vx;
        double Ay = y1 - vy;
        double Bx = x2 - vx;
        double By = y2 - vy;
        double Dx = x1 + vx;
        double Dy = y1 + vy;
        double bax = Bx - Ax;
        double bay = By - Ay;
        double dax = Dx - Ax;
        double day = Dy - Ay;

        if ((x - Ax) * bax + (y - Ay) * bay < 0.0) {
            return false;
        }
        if ((x - Bx) * bax + (y - By) * bay > 0.0) {
            return false;
        }
        if ((x - Ax) * dax + (y - Ay) * day < 0.0) {
            return false;
        }
        if ((x - Dx) * dax + (y - Dy) * day > 0.0) {
            return false;
        }
        return true;
    }
}
