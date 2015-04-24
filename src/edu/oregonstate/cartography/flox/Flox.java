package edu.oregonstate.cartography.flox;

import edu.oregonstate.cartography.flow.gui.MainWindow;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import javax.swing.UIManager;

/**
 * Main class for Flox.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Flox {

    /**
     * Flox model
     */
    private static final Model initModel() {
        Model model = new Model();
        BezierFlow flow = new BezierFlow(0, 0, 0, 100, 100, 100, 200, 0);
        BezierFlow flow2 = new BezierFlow(10, 10, 10, 110, 110, 110, 210, 10);
        model.addFlow(flow);
        model.addFlow(flow2);
        return model;
    }

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {

        System.setProperty("apple.laf.useScreenMenuBar", "true");
        System.setProperty("com.apple.mrj.application.apple.menu.about.name", "Flox");
        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (Exception ex) {
        }

        java.awt.EventQueue.invokeLater(new Runnable() {
            @Override
            public void run() {
                MainWindow window = new MainWindow();
                window.setSize(800, 600);
                window.setVisible(true);
                window.setModel(initModel());
                window.openShapefile();
            }
        });

    }

}
