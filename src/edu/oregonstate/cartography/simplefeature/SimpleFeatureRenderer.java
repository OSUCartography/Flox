package edu.oregonstate.cartography.simplefeature;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.awt.geom.GeneralPath;

/**
 * Renders JTS simple features to a Graphics2D context. Also provides access to
 * the Graphics2D context for directly drawing to the context.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public final class SimpleFeatureRenderer {

    /**
     * Radius of a point symbol.
     */
    private static final double POINT_R = 1;

    /**
     * cached Graphics2D context for bufferImage
     */
    private final Graphics2D g2d;

    /**
     * western most point in geometry
     */
    private final double west;

    /**
     * northernmost point in geometry
     */
    private final double north;

    /**
     * scale factor for drawing geometry pixels = world_coordinates * scale;
     */
    private final double scale;

    /**
     *
     * @param g2d
     * @param west
     * @param north
     * @param scale
     */
    public SimpleFeatureRenderer(Graphics2D g2d, double west, double north, double scale) {
        if (g2d == null || scale <= 0) {
            throw new IllegalArgumentException();
        }
        this.g2d = g2d;
        this.west = west;
        this.north = north;
        this.scale = scale;
    }

    /**
     * Fill and stroke a Graphics2D shape
     *
     * @param shape Shape to draw.
     * @param fillColor Color to fill shape. If null, shape is not filled.
     * @param strokeColor Color to stroke shape. If null, shape is not stroked.
     */
    private void fillStroke(Shape shape, Color fillColor, Color strokeColor) {
        if (fillColor != null) {
            g2d.setColor(fillColor);
            g2d.fill(shape);
        }
        if (strokeColor != null) {
            g2d.setColor(strokeColor);
            g2d.draw(shape);
        }
    }

    /**
     * Sets the stroke of the graphics context to aBasicStroke with a specified width.
     * @param strokeWidth The new stroke width
     */
    public void setStrokeWidth(float strokeWidth) {
        g2d.setStroke(new BasicStroke(strokeWidth));
    }
    
    /**
     * Sets the color of the graphics context.
     * @param color The new color.
     */
    public void setColor(Color color) {
        g2d.setColor(color);
    }

    /**
     * Draw an OGC Simple Feature.
     *
     * @param geometry The geometry to draw.
     * @param fillColor Color to fill shape. If null, shape is not filled.
     * @param strokeColor Color to stroke shape. If null, shape is not stroked.
     */
    public void draw(Geometry geometry, Color fillColor, Color strokeColor) {
        if (geometry == null) {
            return;
        }
        if (geometry instanceof LineString) {
            draw((LineString) geometry, fillColor, strokeColor);
        } else if (geometry instanceof Polygon) {
            draw((Polygon) geometry, fillColor, strokeColor);
        } else if (geometry instanceof Point) {
            draw((Point) geometry, fillColor, strokeColor);
        } else if (geometry instanceof GeometryCollection) {
            draw((GeometryCollection) geometry, fillColor, strokeColor);
        }
    }

    /**
     * Draw an OGC Simple Feature geometry collection.
     *
     * @param collection The geometry to draw.
     * @param fillColor Color to fill shape. If null, shape is not filled.
     * @param strokeColor Color to stroke shape. If null, shape is not stroked.
     */
    public void draw(GeometryCollection collection,
            Color fillColor, Color strokeColor) {
        int nbrObj = collection.getNumGeometries();
        for (int i = 0; i < nbrObj; i++) {
            Geometry geom = collection.getGeometryN(i);
            draw(geom, fillColor, strokeColor);
        }
    }

    /**
     * Draws a circle.
     *
     * @param x X position of center in world coordinates
     * @param y Y position of center in world coordinates
     * @param r Radius.
     * @param fill Fill color
     * @param stroke Stroke color
     */
    public void drawCircle(double x, double y, double r, Color fill, Color stroke) {
        x = xToPx(x);
        y = yToPx(y);
        Ellipse2D circle = new Ellipse2D.Double(x - r, y - r, r * 2, r * 2);
        g2d.setColor(fill);
        g2d.fill(circle);
        g2d.setColor(stroke);
        g2d.draw(circle);
    }

    /**
     * Draw an OGC Simple Feature point.
     *
     * @param point The geometry to draw.
     * @param fillColor Color to fill shape. If null, shape is not filled.
     * @param strokeColor Color to stroke shape. If null, shape is not stroked.
     */
    public void draw(Point point, Color fillColor, Color strokeColor) {
        double d = 2 * POINT_R;
        Ellipse2D circle = new Ellipse2D.Double(xToPx(point.getX()) - POINT_R,
                yToPx(point.getY()) - POINT_R, d, d);
        fillStroke(circle, fillColor, strokeColor);
    }

    /**
     * Add a OGC Simple Feature line string to a Swing path. The path is not
     * closed.
     *
     * @param lineString The line string to add.
     * @param path The Swing path.
     */
    private void addLineStringToGeneralPath(LineString lineString, GeneralPath path) {
        int nPts = lineString.getNumPoints();
        if (nPts < 2) {
            return;
        }

        Point point = lineString.getStartPoint();
        path.moveTo(xToPx(point.getX()), yToPx(point.getY()));

        for (int i = 1; i < nPts; i++) {
            point = lineString.getPointN(i);
            path.lineTo(xToPx(point.getX()), yToPx(point.getY()));
        }
    }

    /**
     * Draw an OGC Simple Feature line string.
     *
     * @param lineString The geometry to draw.
     * @param fillColor Color to fill shape. If null, shape is not filled.
     * @param strokeColor Color to stroke shape. If null, shape is not stroked.
     */
    public void draw(LineString lineString, Color fillColor, Color strokeColor) {
        GeneralPath path = new GeneralPath();
        addLineStringToGeneralPath(lineString, path);
        fillStroke(path, fillColor, strokeColor);
    }

    /**
     * Draw an OGC Simple Feature polygon.
     *
     * @param polygon The geometry to draw.
     * @param fillColor Color to fill shape. If null, shape is not filled.
     * @param strokeColor Color to stroke shape. If null, shape is not stroked.
     */
    public void draw(Polygon polygon, Color fillColor, Color strokeColor) {
        LineString exteriorRing = polygon.getExteriorRing();
        GeneralPath path = new GeneralPath();
        addLineStringToGeneralPath(exteriorRing, path);
        path.closePath();

        int nbrInteriorRings = polygon.getNumInteriorRing();
        for (int i = 0; i < nbrInteriorRings; i++) {
            addLineStringToGeneralPath(polygon.getInteriorRingN(i), path);
            path.closePath();
        }
        fillStroke(path, fillColor, strokeColor);
    }

    /**
     * Transforms a horizontal x coordinate (usually in meters) to pixels. Takes
     * the scale and bounding boundingBox defined by the PageFormat into
     * account.
     *
     * @param x The horizontal coordinate.
     * @return Returns the coordinate in pixels.
     */
    public double xToPx(double x) {
        return (x - west) * scale;
    }

    /**
     * Transforms a vertical y coordinate to the scale and bounding boundingBox
     * defined by the PageFormat.
     *
     * @param y The vertical coordinate.
     * @return Returns the coordinate in the page coordinate system.
     */
    public double yToPx(double y) {
        return (north - y) * scale;
    }

    /**
     * Returns the graphics destination context.
     *
     * @return The rendering context.
     */
    public Graphics2D getGraphics2D() {
        return g2d;
    }

}
