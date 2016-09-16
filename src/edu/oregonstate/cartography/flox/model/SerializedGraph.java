package edu.oregonstate.cartography.flox.model;

import java.util.ArrayList;
import java.util.List;

/**
 * Helper class to serialize Graph with JAXB. JAXB can serialize list, so
 * GraphSerializer converts a graph to lists of flows and unconnected nodes.
 *
 * @author Bernhard Jenny, School of Science, RMIT University, Melbourne
 */
public class SerializedGraph {

    public List<Point> unconnectedNodes = new ArrayList<>();
    public List<Flow> flows = new ArrayList<>();
}
