/*
 * ColorUtils.java
 *
 * Created on May 16, 2005, 8:47 PM
 */
package edu.oregonstate.cartography.utils;

import java.awt.*;

/**
 * Utility methods for color related stuff.
 *
 * @author Bernhard Jenny, Institute of Cartography, ETH Zurich.
 */
public class ColorUtils {

    public static final Color highlightColor = new Color(75, 123, 181);

    /**
     * Converts a Color to a CSS string of the format "rgb(12,34,56)"
     *
     * @param color The color to convert.
     * @return The CSS string.
     */
    public static String colorToCSSString(Color color) {
        StringBuilder str = new StringBuilder();
        str.append("rgb(");
        str.append(color.getRed());
        str.append(",");
        str.append(color.getGreen());
        str.append(",");
        str.append(color.getBlue());
        str.append(")");
        return str.toString();
    }

    /**
     * Converts a Color to a hexadecimal string of the format "#773300"
     *
     * @param color The color to convert.
     * @return The hexadecimal string with a leading '#'.
     */
    public static String colorToRGBHexString(Color color) {

        String rgb = Integer.toHexString(color.getRGB());
        // cut the leading alpha value (2 characters) and add '#'
        return "#" + rgb.substring(2, rgb.length());
    }

    /**
     * Returns a highlight-color that can be used to draw selected objects.
     *
     * @return The color to use for selected objects.
     */
    public static final java.awt.Color getSelectionColor() {
        return java.awt.Color.red;
    }

    /**
     * Returns a highlight-color that can be used to draw selected objects.
     *
     * @return The color to use for selected objects.
     */
    public static final java.awt.Color getHighlightColor() {
        return highlightColor;
    }

    /**
     * Blend two colors
     *
     * @param c1 color 1
     * @param c2 color 2
     * @param ratio blending ratio
     * @return
     */
    public static final Color blend(Color c1, Color c2, double ratio) {
        if (ratio > 1d) {
            ratio = 1d;
        } else if (ratio < 0d) {
            ratio = 0d;
        }

        int i1 = c1.getRGB();
        int i2 = c2.getRGB();

        int a1 = i1 >>> 24;
        int r1 = (i1 & 0xff0000) >> 16;
        int g1 = (i1 & 0xff00) >> 8;
        int b1 = i1 & 0xff;

        int a2 = i2 >>> 24;
        int r2 = (i2 & 0xff0000) >> 16;
        int g2 = (i2 & 0xff00) >> 8;
        int b2 = i2 & 0xff;

        int a = (int) (a1 + (a2 - a1) * ratio);
        int r = (int) (r1 + (r2 - r1) * ratio);
        int g = (int) (g1 + (g2 - g1) * ratio);
        int b = (int) (b1 + (b2 - b1) * ratio);
        
        return new Color(a << 24 | r << 16 | g << 8 | b);
    }
    
}
