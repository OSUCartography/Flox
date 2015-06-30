package edu.oregonstate.cartography.flox.model;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import javax.xml.bind.annotation.adapters.XmlAdapter;

public class GraphSerializer extends XmlAdapter<String, Graph> {

    @Override
    public Graph unmarshal(String s) throws IOException {
        BufferedReader reader = new BufferedReader(new StringReader(s));
        ArrayList<Flow> flows = FlowImporter.readFlows(reader);
        Graph graph = new Graph();
        for (Flow flow : flows) {            
            graph.addFlow(flow);
        }
        return graph;
    }

    @Override
    public String marshal(Graph graph) {
        return CSVFlowExporter.exportToString(graph.flowIterator());
    }
}
