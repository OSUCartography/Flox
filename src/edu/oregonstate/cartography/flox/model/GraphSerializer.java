package edu.oregonstate.cartography.flox.model;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.xml.bind.annotation.adapters.XmlAdapter;
import org.jgrapht.DirectedGraph;
import org.jgrapht.graph.DirectedMultigraph;

public class GraphSerializer extends XmlAdapter<String, DirectedGraph<Point, Flow>> {

    @Override
    public DirectedGraph<Point, Flow> unmarshal(String s) {
        try {
            BufferedReader reader = new BufferedReader(new StringReader(s));
            ArrayList<Flow> flows = FlowImporter.readFlows(reader);
            DirectedGraph<Point, Flow> graph = new DirectedMultigraph<>(Flow.class);
            for (Flow flow : flows) {
                Model.addFlow(flow, graph);
            }
            return graph;
        } catch (IOException ex) {
            Logger.getLogger(GraphSerializer.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }

    @Override
    public String marshal(DirectedGraph<Point, Flow> graph) {
        CSVFlowExporter exporter = new CSVFlowExporter(graph.edgeSet().iterator());
        return exporter.exportToString();
    }
}
