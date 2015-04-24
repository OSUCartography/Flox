package edu.oregonstate.cartography.simplefeature;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Insets;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class SimpleFeatureMapComponent extends AbstractSimpleFeatureMapComponent {

    /**
     * Geometry to draw
     */
    private Geometry geometry;

    public SimpleFeatureMapComponent() {
    }

    /**
     * Returns the bounding box of the geometry to draw.
     *
     * @return The bounding box.
     */
    @Override
    public Envelope getBoundingBox() {
        return geometry != null ? geometry.getEnvelopeInternal() : null;
    }

    /**
     * Override paintComponent of JComponent for custom drawing.
     *
     * @param g The destination to draw to.
     */
    @Override
    protected void paintComponent(Graphics g) {
        if (geometry != null) {
            Graphics2D g2d = initBufferImage();
            draw(geometry, g2d);
            g2d.dispose();

            Insets insets = getInsets();
            ((Graphics2D) g).drawImage(bufferImage, insets.left, insets.top, this);
        }
    }

    /**
     * Returns the geometry that is drawn by this map.
     *
     * @return the geometry
     */
    public Geometry getGeometry() {
        return geometry;
    }

    /**
     * Set the geometry to draw in this map.
     *
     * @param geometry the geometry to set
     */
    public void setGeometry(Geometry geometry) {
        this.geometry = geometry;
        bufferImage = null;
        showAll();
    }

}
