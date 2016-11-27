package edu.oregonstate.cartography.app;

import edu.oregonstate.cartography.flox.gui.MainWindow;
import edu.oregonstate.cartography.flox.model.Model;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import javax.swing.UIManager;

/**
 * Main class for Flox.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Flox {

    /**
     * Main entry point for Flox.
     *
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
        Model model = new Model();

        // initialize GUI
        java.awt.EventQueue.invokeLater(new Runnable() {
            @Override
            public void run() {
                MainWindow window = new MainWindow();
                window.setModel(model);

                // find available screen real estate (without taskbar, etc.)
                Rectangle screen = GraphicsEnvironment.
                        getLocalGraphicsEnvironment().getMaximumWindowBounds();
                window.setSize((int) screen.getWidth(), (int) screen.getHeight());
                window.setLocation((int) screen.getMinX(), (int) screen.getMinY());
                window.setExtendedState(javax.swing.JFrame.MAXIMIZED_BOTH);
                window.setVisible(true);
                window.openComputationPalette();
                loadTestData(window);

                window.addWindowListener(new WindowAdapter() {
                    @Override
                    public void windowClosing(WindowEvent e) {
                        if (window.canDocumentBeClosed()) {
                            window.dispose();
                        }
                    }
                });
            }
        });
    }

    private static void loadTestData(MainWindow window) {
        //window.testOffsetting();
        //window.openXMLFile("/Users/jennyb/Dropbox/Flow Maps/Data/Flox Data/_Swiss_migration/swiss_flows_and_nodes.xml");
        //window.openXMLFile("/Users/jennyb/Desktop/CH Flows no arrows 5.xml");
        //window.openXMLFile("/Users/jennyb/Desktop/Geneva.xml");
        //window.layout("Test Layout");
    }
}
