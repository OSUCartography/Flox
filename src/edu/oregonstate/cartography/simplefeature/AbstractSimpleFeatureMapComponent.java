package edu.oregonstate.cartography.simplefeature;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.RenderingHints;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.geom.GeneralPath;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import javax.swing.JComponent;

/**
 * Abstract base class for a map component that draws JTS simple features.
 * Concrete implementations must implement getBoundingBox and override
 * paintComponent.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public abstract class AbstractSimpleFeatureMapComponent extends JComponent {

    /**
     * image buffer
     */
    protected BufferedImage bufferImage = null;

    /**
     * cached Graphics2D context for bufferImage
     */
    private Graphics2D g2dBuffer;

    /**
     * western most point in geometry
     */
    private double west;

    /**
     * northernmost point in geometry
     */
    private double north;

    /**
     * Drawing offset from left border in pixel.
     */
    private double xOffsetPx;

    /**
     * Drawing offset from top border in pixel.
     */
    private double yOffsetPx;

    /**
     * scale factor for drawing geometry
     */
    private double scale;

    /**
     * Stroke and fill flag for drawing geometry.
     */
    public enum Draw {

        STROKE, FILL
    };

    public AbstractSimpleFeatureMapComponent() {
        addComponentListener(new ComponentAdapter() {
            @Override
            public void componentResized(ComponentEvent e) {
                showAll();
            }
        });
    }

    /**
     * Returns the bounding box of the geometry that is drawn by the map.
     *
     * @return The bounding box.
     */
    protected abstract Rectangle2D getBoundingBox();

    /**
     * Returns a Graphics2D context for a buffer window. Geometry should be
     * rendered to this context. The returned canvas is cleared.
     *
     * @return The Graphics2D context to draw to. This is cached, so do not
     * dispose it.
     */
    protected Graphics2D getGraphics2DBuffer() {

        Insets insets = getInsets();
        int w = getWidth() - insets.left - insets.right;
        int h = getHeight() - insets.top - insets.bottom;

        // make sure the bufferImage image is allocated and has same size
        if (bufferImage == null
                || bufferImage.getWidth() != w
                || bufferImage.getHeight() != h) {

            if (g2dBuffer != null) {
                g2dBuffer.dispose();
            }

            bufferImage = (BufferedImage) createImage(w, h);
            g2dBuffer = bufferImage.createGraphics();

            // enable antialiasing
            g2dBuffer.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                    RenderingHints.VALUE_ANTIALIAS_ON);
            // enable high quality rendering
            g2dBuffer.setRenderingHint(RenderingHints.KEY_RENDERING,
                    RenderingHints.VALUE_RENDER_QUALITY);
            g2dBuffer.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,
                    RenderingHints.VALUE_STROKE_PURE);
            // enable bicubic interpolation of images
            g2dBuffer.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                    RenderingHints.VALUE_INTERPOLATION_BICUBIC);
            // set default appearance of vector elements
            g2dBuffer.setStroke(new BasicStroke(1));
            g2dBuffer.setColor(Color.black);
            g2dBuffer.setBackground(Color.WHITE);
        }

        // erase background
        g2dBuffer.clearRect(0, 0, w, h);

        return g2dBuffer;
    }

    /**
     * Draw an OGC Simple Feature.
     *
     * @param geometry The geometry to draw.
     * @param g2d The graphics context to draw to.
     * @param drawMode Fill or stroke drawing flag
     */
    protected void draw(Geometry geometry, Graphics2D g2d, Draw drawMode) {
        if (geometry == null) {
            return;
        }
        if (geometry instanceof LineString) {
            draw((LineString) geometry, g2d, drawMode);
        } else if (geometry instanceof Polygon) {
            draw((Polygon) geometry, g2d, drawMode);
        } else if (geometry instanceof Point) {
            draw((Point) geometry, g2d, drawMode);
        } else if (geometry instanceof GeometryCollection) {
            draw((GeometryCollection) geometry, g2d, drawMode);
        }
    }

    /**
     * Draw an OGC Simple Feature geometry collection.
     *
     * @param collection The geometry to draw.
     * @param g2d The graphics context to draw to.
     * @param drawMode Fill or stroke drawing flag
     */
    protected void draw(GeometryCollection collection, Graphics2D g2d, Draw drawMode) {
        int nbrObj = collection.getNumGeometries();
        for (int i = 0; i < nbrObj; i++) {
            Geometry geom = collection.getGeometryN(i);
            draw(geom, g2d, drawMode);
        }
    }

    /**
     * Draw an OGC Simple Feature point.
     *
     * @param point The geometry to draw.
     * @param g2d The graphics context to draw to.
     * @param drawMode Fill or stroke drawing flag
     */
    protected void draw(Point point, Graphics2D g2d, Draw drawMode) {
        // TODO
        throw new InternalError();
    }

    /**
     * Add a OGC Simple Feature line string to a Swing path.
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
     * @param g2d The graphics context to draw to.
     * @param drawMode Fill or stroke drawing flag
     */
    protected void draw(LineString lineString, Graphics2D g2d, Draw drawMode) {
        GeneralPath path = new GeneralPath();
        addLineStringToGeneralPath(lineString, path);
        if (drawMode == Draw.STROKE) {
            g2d.draw(path);
        } else {
            g2d.fill(path);
        }
    }

    /**
     * Draw an OGC Simple Feature polygon.
     *
     * @param polygon The geometry to draw.
     * @param g2d The graphics context to draw to.
     * @param drawMode Fill or stroke drawing flag
     */
    protected void draw(Polygon polygon, Graphics2D g2d, Draw drawMode) {
        LineString exteriorRing = polygon.getExteriorRing();
        GeneralPath path = new GeneralPath();
        addLineStringToGeneralPath(exteriorRing, path);

        int nbrInteriorRings = polygon.getNumInteriorRing();
        for (int i = 0; i < nbrInteriorRings; i++) {
            addLineStringToGeneralPath(polygon.getInteriorRingN(i), path);
        }
        if (drawMode == Draw.STROKE) {
            g2d.draw(path);
        } else {
            g2d.fill(path);
        }
    }

    /**
     * Transforms a horizontal x coordinate (usually in meters) to pixels. Takes
     * the scale and bounding box defined by the PageFormat into account.
     *
     * @param x The horizontal coordinate.
     * @return Returns the coordinate in pixels.
     */
    protected double xToPx(double x) {
        return (x - west) / scale + xOffsetPx;
    }

    /**
     * Transforms a vertical y coordinate to the scale and bounding box defined
     * by the PageFormat.
     *
     * @param y The vertical coordinate.
     * @return Returns the coordinate in the page coordinate system.
     */
    protected double yToPx(double y) {
        return (north - y) / scale + yOffsetPx;
    }

    /**
     * Inform Swing that this JComponent is opaque, i.e. we are drawing the
     * whole area of this Component. This accelerates the drawing of the
     * component.
     *
     * @return true if opaque.
     */
    @Override
    public boolean isOpaque() {
        return true;
    }

    /**
     * Adjust scale and offset to show the entire geometry in the available
     * canvas space, and repaint the map.
     */
    public void showAll() {
        Rectangle2D bb = getBoundingBox();

        if (bb == null) {
            scale = 1;
            west = 0;
            north = 0;
            xOffsetPx = 0;
            yOffsetPx = 0;
        } else {
            Insets insets = getInsets();
            int w = getWidth() - insets.left - insets.right;
            int h = getHeight() - insets.top - insets.bottom;
            double vScale = bb.getHeight() / h;
            double hScale = bb.getWidth() / w;
            scale = vScale > hScale ? vScale : hScale;
            west = bb.getMinX();
            north = bb.getMaxY();
            xOffsetPx = (w - bb.getWidth() / scale) / 2;
            yOffsetPx = (h - bb.getHeight() / scale) / 2;
        }
        repaint();
    }

}
