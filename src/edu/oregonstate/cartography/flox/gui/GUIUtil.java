/*
 * GUIUtil.java
 *
 * Created on September 11, 2007, 10:54 PM
 *
 */
package edu.oregonstate.cartography.flox.gui;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.Frame;
import javax.swing.JComponent;
import javax.swing.JSlider;

/**
 *
 * @author Bernhard Jenny, Institute of Cartography, ETH Zurich
 */
public class GUIUtil {

    public static Frame getOwnerFrame(Component component) {
        while ((component != null) && !(component instanceof Frame)) {
            component = component.getParent();
        }
        return ((Frame) component);
    }

    /**
     * Returns the front most Frame. If possible, the Frame that currently has
     * the focus and is visible is returned.
     *
     * @return The frontmost Frame or null if no Frame can be found.
     */
    public static Frame getFrontMostFrame() {

        // search visible window which is focused
        Frame[] frames = Frame.getFrames();
        for (int i = 0; i < frames.length; i++) {
            Frame frame = frames[i];
            if (frame != null && frame.isVisible() && frame.isFocused()) {
                return frame;
            }
        }

        // search visible window
        frames = Frame.getFrames();
        for (int i = 0; i < frames.length; i++) {
            Frame frame = frames[i];
            if (frame != null && frame.isVisible()) {
                return frame;
            }
        }

        // search window
        frames = Frame.getFrames();
        for (int i = 0; i < frames.length; i++) {
            Frame frame = frames[i];
            if (frame != null) {
                return frame;
            }
        }

        return null;
    }

    /**
     * Returns the preferred dimensions of a component, using the current height
     * and a custom width.
     *
     * @param component The component providing the preferred height.
     * @param width The preferred width.
     * @return The preferred dimension.
     */
    public static Dimension getPreferredSize(JComponent component, int width) {
        Dimension dim = component.getPreferredSize();
        return new Dimension(width, dim.height);
    }

    public static void setPreferredWidth(JComponent component, int width) {
        component.setPreferredSize(getPreferredSize(component, width));
    }

    public static void setMinimumWidth(JComponent component, int minWidth) {
        Dimension dim = component.getMinimumSize();
        dim = new Dimension(minWidth, dim.height);
        component.setMinimumSize(dim);
    }

    /**
     * Adds a string to each major tick label of a slider. Useful to append
     * percentage signs.
     *
     * @param slider the slider
     * @param extension the extension to append.
     */
    public static void extendSliderLabels(JSlider slider, String extension) {
        java.util.Hashtable labels = slider.createStandardLabels(slider.getMajorTickSpacing());
        java.util.Enumeration e = labels.elements();
        while (e.hasMoreElements()) {
            javax.swing.JComponent comp = (javax.swing.JComponent) e.nextElement();
            if (comp instanceof javax.swing.JLabel) {
                javax.swing.JLabel label = (javax.swing.JLabel) (comp);
                label.setText(label.getText() + extension);
            }
        }
        slider.setLabelTable(labels);
    }
}
