package edu.oregonstate.cartography.simplefeature;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import edu.oregonstate.cartography.map.MapEventHandler;
import edu.oregonstate.cartography.map.MapTool;
import edu.oregonstate.cartography.map.MapToolMouseMotionListener;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.RenderingHints;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.geom.GeneralPath;
import java.awt.geom.Point2D;
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
     * The amount by which the scale factor changes on a simple zoom-in or
     * zoom-out command without specifying the exact new scale factor.
     */
    private static final double ZOOM_STEP = 1. / 3.;
    private static final double MIN_SCALE = 0.0000001;

    /**
     * Stroke and fill flag for drawing geometry.
     */
    public enum Draw {

        STROKE, FILL
    };

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
     * scale factor for drawing geometry pixels = world_coordinates * scale;
     */
    private double scale;

    /**
     * The MapEventHandler is responsible for treating all key and mouse events
     * for this MapComponent. The whole functionality of MapEventHandler could
     * have been integrated into MapComponent. By separating the two, the
     * MapComponent is easier to program and extend.
     */
    private final MapEventHandler mapEventHandler = new MapEventHandler(this);

    public AbstractSimpleFeatureMapComponent() {
        addComponentListener(new ComponentAdapter() {
            @Override
            public void componentResized(ComponentEvent e) {
                showAll();
            }
        });
    }

    /**
     * Returns the bounding box of the geometry that is drawn by the
     * map.
     *
     * @return The bounding boundingBox.
     */
    protected abstract Rectangle2D getBoundingBox();

    /**
     * Converts from the coordinate system of this map component to the world
     * coordinate system used by the geographic features.
     *
     * @param pt The point to convert.
     * @return The convert point in world coordinates.
     */
    public final Point2D.Double userToWorldSpace(java.awt.Point pt) {
        // compensate for border around the map
        Insets insets = this.getInsets();
        double x = (pt.getX() - insets.left) / scale + west;
        double y = -(pt.getY() - insets.top) / scale + north;
        return new Point2D.Double(x, y);
    }

    /**
     * Returns the extension of the visible area in world coordinates (the
     * coordinate system used by the geographic features).
     *
     * @return The currently visible area in world coordinates.
     */
    public Rectangle2D getVisibleArea() {
        double w = getVisibleWidth();
        double h = getVisibleHeight();
        return new Rectangle2D.Double(west, north - h, w, h);
    }

    /**
     * Returns the widthPx of the currently visible area in world coordinates
     * (the coordinate system used by the GeoObjects).
     *
     * @return The widthPx of the currently visible area in world coordinates.
     */
    public double getVisibleWidth() {
        Insets insets = getInsets();
        double width = getWidth() - insets.left - insets.right;
        return width / scale;
    }

    /**
     * Returns the heightPx of the currently visible area in world coordinates
     * (the coordinate system used by the GeoObjects).
     *
     * @return The heightPx of the currently visible area in world coordinates.
     */
    public double getVisibleHeight() {
        Insets insets = getInsets();
        double height = getHeight() - insets.top - insets.bottom;
        return height / scale;
    }

    /**
     * Centers the map display on the passed point and repaints the map.
     *
     * @param cx The new center of the visible area in world coordinates.
     * @param cy The new center of the visible area in world coordinates.
     */
    public void centerOnPoint(double cx, double cy) {
        west = cx - getVisibleWidth() / 2;
        north = cy + getVisibleHeight() / 2;
        repaint();
    }

    /**
     * Shifts the currently displayed area of the map in horizontal and vertical
     * direction and repaints the map.
     *
     * @param dx Offset in horizontal direction in world coordinates.
     * @param dy Offset in vertical direction in world coordinates.
     */
    public void offsetVisibleArea(double dx, double dy) {
        west += dx;
        north += dy;
        repaint();
    }

    /**
     * Zooms into the map. The currently visible center of the map is
     * maintained.
     */
    public void zoomIn() {
        zoom(1. + ZOOM_STEP);
    }

    /**
     * Zooms into the map. The passed fixPoint is to remain a its current
     * location.
     *
     ** @param fixPoint The point that shall not move in world coordinates.
     */
    public void zoomIn(Point2D.Double fixPoint) {
        zoomOnPoint(1. + ZOOM_STEP, fixPoint);
    }

    /**
     * Zooms out of the map. The currently visible center is retained.
     */
    public void zoomOut() {
        zoom(1. / (1. + ZOOM_STEP));
    }

    /**
     * Zooms out. The passed fixPoint is to remain a its current location
     *
     * @param fixPoint The point that shall not move in world coordinates.
     */
    public void zoomOut(Point2D.Double fixPoint) {
        zoomOnPoint(1. / (1. + ZOOM_STEP), fixPoint);
    }

    /**
     * Zooms in or out and retains the current center point.
     *
     * @param zoomFactor The new zoom factor.
     */
    public void zoom(double zoomFactor) {
        double newScale = scale * zoomFactor;
        if (newScale < MIN_SCALE) {
            newScale = MIN_SCALE;
        }

        Rectangle2D visibleRect = getVisibleArea();
        double cx = visibleRect.getCenterX();
        double cy = visibleRect.getCenterY();
        double dx = cx - west;
        double dy = cy - north;

        dx *= this.scale / newScale;
        dy *= this.scale / newScale;
        west = cx - dx;
        north = cy - dy;
        scale = newScale;
        repaint();
    }

    /**
     * Changes the current scale by a specified factor. The passed fixPoint is
     * to remain a its current location.
     *
     * @param zoomFactor The new zoom factor.
     * @param fixPoint The point that shall not move in world coordinates.
     */
    public void zoomOnPoint(double zoomFactor, Point2D.Double fixPoint) {
        double dx = fixPoint.x - west;
        double dy = north - fixPoint.y;
        scale *= zoomFactor;
        west = fixPoint.x - dx / zoomFactor;
        north = fixPoint.y + dy / zoomFactor;
        repaint();
    }

    /**
     * Zooms on passed rectangle. The area contained in the rectangle becomes
     * entirely visible. The map is repainted.
     *
     * @param r The area that will be at least visible.
     */
    public void zoomOnRectangle(Rectangle2D r) {
        // an empty border on each side of the map as a percentage of the map size
        final double BORDER_PERCENTAGE = 2;
        
        if (r == null || r.getWidth() <= 0 || r.getHeight() <= 0) {
            scale = 1;
            west = 0;
            north = 0;
            return;
        }

        Insets insets = this.getInsets();
        double widthPx = this.getWidth() - insets.left - insets.right;
        double heightPx = this.getHeight() - insets.top - insets.bottom;
        double borderScale = 1. / (1 + 2 * BORDER_PERCENTAGE / 100d);
        if (r.getWidth() == 0 && r.getHeight() == 0) {
            scale = MIN_SCALE;
        } else if (r.getWidth() == 0) {
            scale = heightPx / r.getHeight() * borderScale;
        } else if (r.getHeight() == 0) {
            scale = widthPx / r.getWidth() * borderScale;
        } else {
            double horScale = widthPx / r.getWidth();
            double verScale = heightPx / r.getHeight();
            scale = Math.min(horScale, verScale) * borderScale;
        }
        centerOnPoint(r.getCenterX(), r.getCenterY());
    }
    
    /**
     * Adjust scale and center of the map to show the entire geometry in the 
     * available canvas space, and repaint the map.
     */
    public void showAll() {
        zoomOnRectangle(getBoundingBox());
    }

    /**
     * Returns the current MapTool
     *
     * @return The currently active MapTool.
     */
    public MapTool getMapTool() {
        return mapEventHandler.getMapTool();
    }

    /**
     * Sets the current MapTool.
     *
     * @param mapTool The new MapTool
     */
    public void setMapTool(MapTool mapTool) {
        mapEventHandler.setMapTool(mapTool, false);
    }

    public void removeMouseMotionListener(MapToolMouseMotionListener listener) {
        this.mapEventHandler.removeMouseMotionListener(listener);
    }

    public void addMouseMotionListener(MapToolMouseMotionListener listener) {
        this.mapEventHandler.addMouseMotionListener(listener);
    }

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
        path.closePath();

        int nbrInteriorRings = polygon.getNumInteriorRing();
        for (int i = 0; i < nbrInteriorRings; i++) {
            addLineStringToGeneralPath(polygon.getInteriorRingN(i), path);
            path.closePath();
        }
        if (drawMode == Draw.STROKE) {
            g2d.draw(path);
        } else {
            g2d.fill(path);
        }
    }

    /**
     * Transforms a horizontal x coordinate (usually in meters) to pixels. Takes
     * the scale and bounding boundingBox defined by the PageFormat into
     * account.
     *
     * @param x The horizontal coordinate.
     * @return Returns the coordinate in pixels.
     */
    protected double xToPx(double x) {
        return (x - west) * scale;
    }

    /**
     * Transforms a vertical y coordinate to the scale and bounding boundingBox
     * defined by the PageFormat.
     *
     * @param y The vertical coordinate.
     * @return Returns the coordinate in the page coordinate system.
     */
    protected double yToPx(double y) {
        return (north - y) * scale;
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
}
