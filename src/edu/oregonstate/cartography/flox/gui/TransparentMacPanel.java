package edu.oregonstate.cartography.flox.gui;

import edu.oregonstate.cartography.utils.Sys;
import java.awt.LayoutManager;
import javax.swing.JPanel;

/**
 * Makes a JPanel transparent when on Mac OS X or newer. This is needed for
 * panels in JTabbedPanes.
 * @author Bernhard Jenny, Institute of Cartography, ETH Zurich.
 */
public class TransparentMacPanel extends JPanel {

    private void conditionalTransparency() {
        if (Sys.isMacOSX_10_5_orHigherWithJava5()) {
            this.setOpaque(false);
        }
    }

    public TransparentMacPanel(LayoutManager layout, boolean isDoubleBuffered) {
        super(layout, isDoubleBuffered);
        conditionalTransparency();
    }

    public TransparentMacPanel(LayoutManager layout) {
        super(layout);
        conditionalTransparency();
    }

    public TransparentMacPanel(boolean isDoubleBuffered) {
        super(isDoubleBuffered);
        conditionalTransparency();
    }

    public TransparentMacPanel() {
        conditionalTransparency();
    }
}
