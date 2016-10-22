package edu.oregonstate.cartography.flox.gui;

import edu.oregonstate.cartography.flox.model.ForceLayouter;
import java.awt.geom.Rectangle2D;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JProgressBar;
import javax.swing.SwingWorker;
import edu.oregonstate.cartography.flox.model.ProcessMonitor;

/**
 * SwingWorker for creating a new flow map layout. This is the controller
 * bridging between a ForceLayouter and the GUI. Triggers individual iterations
 * for the force-based layout computation.
 *
 * @author Bernhard Jenny, School of Science, RMIT University, Melbourne
 */
class LayoutWorker extends SwingWorker<Void, Void> implements ProcessMonitor {

    private final ForceLayouter layouter;
    private final JProgressBar progressBar;
    private final FloxMapComponent mapComponent;

    public LayoutWorker(ForceLayouter layouter, JProgressBar progressBar,
            FloxMapComponent mapComponent) {

        this.layouter = layouter;
        this.progressBar = progressBar;
        this.mapComponent = mapComponent;

        this.layouter.setProcessMonitor(this);
        
        this.addPropertyChangeListener(new PropertyChangeListener() {

            @Override
            public void propertyChange(PropertyChangeEvent evt) {
                if ("progress".equals(evt.getPropertyName())) {
                    progressBar.setValue((Integer) evt.getNewValue());
                }
            }
        });
    }

    /**
     * Apply layout iterations to all non-locked flows. This is run in the
     * worker thread.
     */
    private void layout() {
        final int nbrIterations = layouter.getModel().getNbrIterations();
        
        // After 10% of all iterations the first flow is moved away from
        // obstacles like arrowheads and unconnected nodes. This gives flows
        // a chance to stabilize before the first one is moved.
        int iterBeforeMovingFlows = nbrIterations / 10;

        // store initial lock flags of all flows
        boolean[] initialLocks = layouter.getModel().getLocks();
        layouter.getModel().toBidirectionalFlows();
        if (isCancelled()) {
            return;
        }
        
        Rectangle2D canvas = layouter.getModel().getCanvas();
        for (int i = 0; i < nbrIterations; i++) {
            if (isCancelled()) {
                break;
            }
            //long startTime = System.nanoTime();    

            iterBeforeMovingFlows = layouter.layoutIteration(i, 
                    iterBeforeMovingFlows, canvas);
            
            //long estimatedTime = System.nanoTime() - startTime;
            //System.out.format("%d %.3f seconds\n", i, estimatedTime / 1000d / 1000d / 1000d);
            
            // update progress indicator
            double progress = 100d * i / nbrIterations;
            setProgress((int) Math.round(progress));
        }
        layouter.getModel().toUnidirectionalFlows();
        // reset lock flags to initial values
        layouter.getModel().applyLocks(initialLocks);
    }

    @Override
    public Void doInBackground() {
        setProgress(0);
        layout();
        return null;
    }

    /**
     * Finished computations. This is invoked on the Event Dispatch Thread.
     */
    @Override
    public void done() {
        try {
            if (!isCancelled()) {
                get();
                layouter.applyChangesToModel(mapComponent.getModel());
                mapComponent.eraseBufferImage();
                mapComponent.repaint();
                progressBar.setValue(0);
                progressBar.setEnabled(false);
            }
        } catch (Throwable t) {
            Logger.getLogger(MainWindow.class.getName()).log(Level.SEVERE, null, t);
            String title = "Flox Error";
            String msg = "An error occured while computing a new layout.";
            ErrorDialog.showErrorDialog(msg, title, t, null);
        }
    }

    /**
     * Process intermediate results. This is invoked on the Event Dispatch
     * Thread.
     */
    @Override
    protected void process(List<Void> ignore) {
        // draw the new graph on the map
        mapComponent.eraseBufferImage();
        mapComponent.repaint();
    }

}
