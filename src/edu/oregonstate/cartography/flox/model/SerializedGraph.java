package edu.oregonstate.cartography.flox.model;

import java.util.ArrayList;
import java.util.List;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlElements;

/**
 * Helper class to serialize Graph with JAXB. JAXB can serialize list, so
 * GraphSerializer converts a graph to lists of flows and unconnected nodes.
 *
 * @author Bernhard Jenny, School of Science, RMIT University, Melbourne
 */
public class SerializedGraph {
   
    @XmlElement(name="unconnectedNode")
    public List<Point> unconnectedNodes = new ArrayList<>();

    
    @XmlElements({        
        @XmlElement(name = "flow", type = Flow.class),
        @XmlElement(name = "flowPair", type = FlowPair.class)
    }
    )
    public List<Flow> flows = new ArrayList<>();
}
