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
     * to the midpoint of the baseline, and the location along the range box's
     * border where the line crosses. Checks each side of the range rectangle
     * one at a time.
     *
     * @param flow a Flow
     */
    public void enforceFlowControlPointRange(Flow flow) {

        Point cPt = flow.getCtrlPt();
        Point refPt = flow.getBaseLineMidPoint();

        Point[] box = computeRangebox(flow);
        // box corners have counter-clockwise order: 
        // bottom left, bottom right, top right, top left.

        // bottom border
        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                box[0].x, box[0].y,
                box[1].x, box[1].y)) {
            GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    box[0].x, box[0].y,
                    box[1].x, box[1].y, cPt);
        }

        // top border
        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                box[2].x, box[2].y,
                box[3].x, box[3].y)) {
            GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    box[2].x, box[2].y,
                    box[3].x, box[3].y, cPt);
        }

        // right border
        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                box[1].x, box[1].y,
                box[2].x, box[2].y)) {
            GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    box[1].x, box[1].y,
                    box[2].x, box[2].y, cPt);
        }

        // left border
        if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                box[0].x, box[0].y,
                box[3].x, box[3].y)) {
            GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    box[0].x, box[0].y,
                    box[3].x, box[3].y, cPt);
        }

    }

    public void enforceCanvasBoundingBox(Flow flow, Rectangle2D canvas) {

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
            GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    b1.x, b1.y,
                    b2.x, b2.y, cPt);
        } else if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                b3.x, b3.y,
                b4.x, b4.y)) {
            GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    b3.x, b3.y,
                    b4.x, b4.y, cPt);
        } else if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                b1.x, b1.y,
                b3.x, b3.y)) {
            GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    b1.x, b1.y,
                    b3.x, b3.y, cPt);
        } else if (GeometryUtils.linesIntersect(
                refPt.x, refPt.y,
                cPt.x, cPt.y,
                b2.x, b2.y,
                b4.x, b4.y)) {
            GeometryUtils.getLineLineIntersection(
                    refPt.x, refPt.y,
                    cPt.x, cPt.y,
                    b2.x, b2.y,
                    b4.x, b4.y, cPt);
        }
    }

    /**
     * Computes the corner points of a range box.
     *
     * @param flow the flow for which to compute a range box
     * @return array with four Point objects. Counter-clockwise order: Bottom
     * left, bottom right, top right, top left.
     */
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

    /**
     * Tests whether a point is inside the range box of a flow.
     *
     * @param flow test with the range box of this flow
     * @param x horizontal coordinate of the point to test
     * @param y vertical coordinate of the point to test
     * @return true if the point is inside the range box, false otherwise.
     */
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

    /**
     * Returns the distance to the one corner point of a range box that is the
     * farthest away from the passed point.
     *
     * @param rangeBoxCorners corners of the range box
     * @param x x coordinate of the point
     * @param y y coordinate of the point
     * @return squared value of the distance
     */
    public double longestDistanceSqToCorner(Point[] rangeBoxCorners, double x, double y) {
        double maxDistSq = 0;
        for (int i = 0; i < rangeBoxCorners.length; i++) {
            Point corner = rangeBoxCorners[i];
            double dx = x - corner.x;
            double dy = y - corner.y;
            double distSq = dx * dx + dy * dy;
            if (distSq > maxDistSq) {
                maxDistSq = distSq;
            }
        }
        return maxDistSq;
    }
}
