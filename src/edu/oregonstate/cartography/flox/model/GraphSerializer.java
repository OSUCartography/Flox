package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.io.ParseException;
import java.io.IOException;
import java.util.Iterator;
import javax.xml.bind.annotation.adapters.XmlAdapter;

public class GraphSerializer extends XmlAdapter<SerializedGraph, Graph> {

    @Override
    public Graph unmarshal(SerializedGraph sg) throws IOException, ParseException {
        Graph graph = new Graph();
        
        // flows
        for (Flow flow : sg.flows) {
            graph.addFlow(flow);
        }
        
        // unconnected nodes
        for (Point unconnectedNode : sg.unconnectedNodes) {
            graph.addNode(unconnectedNode);
        }
        
        return graph;
    }

    @Override
    public SerializedGraph marshal(Graph graph) {
        SerializedGraph sg = new SerializedGraph();        
        
        // flows
        Iterator<Flow> flowIterator = graph.flowIterator();
        while (flowIterator.hasNext()) {
            sg.flows.add(flowIterator.next());
        }
        
        // unconnected nodes
        Iterator<Point> nodeIterator = graph.nodeIterator();
        while (nodeIterator.hasNext()) {
            Point node = nodeIterator.next();
            if (graph.getFlowsForNode(node).isEmpty()) {
                sg.unconnectedNodes.add(node);
            }
        }
        
        return sg;
    }

}
