package edu.oregonstate.cartography.gui;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.RenderingHints;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.geom.GeneralPath;
import java.awt.image.BufferedImage;
import javax.swing.JComponent;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class MapComponent extends JComponent {

    /**
     * image buffer
     */
    private BufferedImage bufferImage = null;

    /**
     * Geometry to draw
     */
    private Geometry geometry;

    /**
     * western most point in geometry
     */
    private double west;

    /**
     * northernmost point in geometry
     */
    private double north;

    /**
     * scale factor for drawing geometry
     */
    private double scale;

    public MapComponent() {
        addComponentListener(new ComponentAdapter() {
            @Override
            public void componentResized(ComponentEvent e) {
                showAll();
            }
        });
    }

    protected Graphics2D initBufferImage() {

        Insets insets = getInsets();
        int w = getWidth() - insets.left - insets.right;
        int h = getHeight() - insets.top - insets.bottom;

        // make sure the bufferImage image is allocated and has same size
        if (bufferImage == null
                || bufferImage.getWidth() != w
                || bufferImage.getHeight() != h) {
            bufferImage = (BufferedImage) createImage(w, h);

            Graphics2D g2d = (Graphics2D) bufferImage.getGraphics();

            g2d.setBackground(Color.WHITE);
            g2d.clearRect(0, 0, w, h);

            // enable antialiasing
            g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                    RenderingHints.VALUE_ANTIALIAS_ON);
            // enable high quality rendering
            g2d.setRenderingHint(RenderingHints.KEY_RENDERING,
                    RenderingHints.VALUE_RENDER_QUALITY);
            g2d.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,
                    RenderingHints.VALUE_STROKE_PURE);

            // set default appearance of vector elements
            g2d.setStroke(new BasicStroke(1));
            g2d.setColor(Color.black);

            return g2d;
        } else {
            return (Graphics2D) bufferImage.getGraphics();
        }
    }

    protected void draw(Geometry geometry, Graphics2D g2d) {
        if (geometry instanceof LineString) {
            draw((LineString) geometry, g2d);
        } else if (geometry instanceof Polygon) {
            draw((Polygon) geometry, g2d);
        } else if (geometry instanceof Point) {
            draw((Point) geometry, g2d);
        } else if (geometry instanceof GeometryCollection) {
            draw((GeometryCollection) geometry, g2d);
        }
    }

    protected void draw(GeometryCollection collection, Graphics2D g2d) {
        int nbrObj = collection.getNumGeometries();
        for (int i = 0; i < nbrObj; i++) {
            Geometry geom = collection.getGeometryN(i);
            draw(geom, g2d);
        }
    }

    protected void draw(Point point, Graphics2D g2d) {
    }

    protected void draw(LineString lineString, Graphics2D g2d) {
        int nPts = lineString.getNumPoints();
        if (nPts < 2) {
            return;
        }

        GeneralPath path = new GeneralPath();
        Point point = lineString.getStartPoint();
        path.moveTo(xToPx(point.getX()), yToPx(point.getY()));

        for (int i = 1; i < nPts; i++) {
            point = lineString.getPointN(i);
            path.lineTo(xToPx(point.getX()), yToPx(point.getY()));
        }

        g2d.draw(path);
    }

    protected void draw(Polygon polygon, Graphics2D g2d) {
        LineString exteriorRing = polygon.getExteriorRing();
        draw(exteriorRing, g2d);
    }

    /**
     * Transforms a horizontal x coordinate (usually in meters) to pixels. Takes
     * the scale and bounding box defined by the PageFormat into account.
     *
     * @param x The horizontal coordinate.
     * @return Returns the coordinate in pixels.
     */
    protected double xToPx(double x) {
        return (x - west) / scale;
    }

    /**
     * Transforms a vertical y coordinate to the scale and bounding box defined
     * by the PageFormat.
     *
     * @param y The vertical coordinate.
     * @return Returns the coordinate in the page coordinate system.
     */
    protected double yToPx(double y) {
        return (north - y) / scale;
    }

    /**
     * Override paintComponent of JComponent for custom drawing.
     *
     * @param g The destination to draw to.
     */
    @Override
    protected void paintComponent(Graphics g) {
        Graphics2D g2d = initBufferImage();
        draw(geometry, g2d);
        g2d.dispose();

        Insets insets = getInsets();
        ((Graphics2D) g).drawImage(bufferImage, insets.left, insets.top, this);
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
     * @return the geometry
     */
    public Geometry getGeometry() {
        return geometry;
    }

    /**
     * @param geometry the geometry to set
     */
    public void setGeometry(Geometry geometry) {
        this.geometry = geometry;
        showAll();
        bufferImage = null;
        repaint();
    }

    /**
     * Adjust scale and offset to show the entire geometry in the available
     * space.
     */
    public void showAll() {
        if (geometry == null) {
            scale = 1;
            west = 0;
            north = 0;
        } else {
            Insets insets = getInsets();
            int w = getWidth() - insets.left - insets.right;
            int h = getHeight() - insets.top - insets.bottom;
            Envelope bb = geometry.getEnvelopeInternal();
            double vScale = bb.getHeight() / h;
            double hScale = bb.getWidth() / w;
            scale = vScale > hScale ? vScale : hScale;
            west = bb.getMinX();
            north = bb.getMaxY();
        }
    }

}
