package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.flox.gui.MainWindow;
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
        
        Point p1 = new Point(0, 0);
        Point p2 = new Point(100, 30);
        
        CubicBezierFlow flow = new CubicBezierFlow(p1, p2);
        flow.setValue(1);
        //model.addFlow(flow);
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

        // initialize model
        Model model = initModel();
        
        // initialize GUI
        java.awt.EventQueue.invokeLater(new Runnable() {
            @Override
            public void run() {
                MainWindow window = new MainWindow();
                window.setSize(800, 600);
                window.setVisible(true);
                window.setModel(model);
                window.openFlowsCSVFile();
            }
        });

    }

}
