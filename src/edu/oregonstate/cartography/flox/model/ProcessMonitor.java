package edu.oregonstate.cartography.flox.model;

/**
 * Inform a lengthy process about a cancellation status.
 * 
 * @author Bernhard Jenny, School of Science - Geospatial Science, RMIT
 * University, Melbourne
 */
public interface ProcessMonitor {
    
    /**
     * True if the process should be canceled.
     * 
     * @return true if process should be canceled.
     */
    public boolean isCancelled();
}
