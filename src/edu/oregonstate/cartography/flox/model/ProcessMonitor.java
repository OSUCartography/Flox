package edu.oregonstate.cartography.flox.model;

/**
 * Inform a lengthy operation about a cancellation status.
 * 
 * @author Bernhard Jenny, School of Science - Geospatial Science, RMIT
 * University, Melbourne
 */
public interface ProcessMonitor {
    
    /**
     * True if the process should be canceled.
     * 
     * @return true if the operation should be canceled.
     */
    public boolean isCancelled();
}
