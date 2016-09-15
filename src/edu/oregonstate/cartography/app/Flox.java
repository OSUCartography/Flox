package edu.oregonstate.cartography.app;

import edu.oregonstate.cartography.flox.gui.MainWindow;
import edu.oregonstate.cartography.flox.model.Model;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import javax.swing.JOptionPane;
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
                //window.openFlowsCSVFile();

                window.addWindowListener(new WindowAdapter() {
                    @Override
                    public void windowClosing(WindowEvent e) {
                        String msg = "This will exit Flox and all unsaved edits will be lost.";
                        String title = "Flox";
                        int res = JOptionPane.showConfirmDialog(window, msg, title, 
                                JOptionPane.OK_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE);
                        if (res == JOptionPane.YES_OPTION) {
                            window.dispose();
                        }
                    }
                });
            }
        });
    }
}
