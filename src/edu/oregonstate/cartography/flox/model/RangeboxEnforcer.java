/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.utils.GeometryUtils;
import java.awt.geom.Rectangle2D;

/**
 *
 * @author Maccabee
 */
public class RangeboxEnforcer {

    private final Model model;

    /**
     * If the control point of a flow falls outside of the flow's range box,
     * this returns the intersection between a line connecting the control point
     * to the midpoint of the baseline, and the location along the rangebox's
     * border where the line crosses. Checks each side of the range rectangle
     * one at a time.
     *
     * @param flow A QuadraticBezierFlow
     * @return
     */
    public Point enforceFlowControlPointRange(QuadraticBezierFlow flow) {

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

    public Point enforceCanvasBoundingBox(QuadraticBezierFlow flow, Rectangle2D canvas) {

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

    public RangeboxEnforcer(Model model) {
        this.model = model;
    }

    public Point[] computeRangebox(Flow flow) {

        double baseDist = flow.getBaselineLength();
        double baseAzimuth = flow.getBaselineAzimuth();
        Point bPt = new Point(flow.startPt.x + baseDist, flow.startPt.y);
        double boxHeight = model.getFlowRangeboxHeight();

        Point b1 = (new Point(flow.startPt.x, flow.startPt.y + (baseDist * boxHeight)))
                .rotatePoint(flow.startPt, baseAzimuth);
        Point b2 = (new Point(bPt.x, bPt.y + (baseDist * boxHeight)))
                .rotatePoint(flow.startPt, baseAzimuth);
        Point b3 = (new Point(flow.startPt.x, flow.startPt.y - (baseDist * boxHeight)))
                .rotatePoint(flow.startPt, baseAzimuth);
        Point b4 = (new Point(bPt.x, bPt.y - (baseDist * boxHeight)))
                .rotatePoint(flow.startPt, baseAzimuth);
        
        Point[] rangeboxPoints = {b1, b2, b3, b4};
        return rangeboxPoints;
    }
}
