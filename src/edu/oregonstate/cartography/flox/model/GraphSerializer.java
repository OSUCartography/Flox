package edu.oregonstate.cartography.flox.model;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
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
    
    
    /*
    @Override
    public String marshal(Graph graph) {
        
        // A map of the nodes, with the node itself as the key.
        // The string will be a new ID number.
        HashMap<Point, String> nodeMap = new HashMap<>();
        
        // Use this to populate the nodeMap. Maybe. Might not need it.
        ArrayList<Point> nodes = graph.getNodes();
        
        // Get the flows
        Iterator flows = graph.flowIterator();
        
        StringBuilder nodeStr = new StringBuilder();
        StringBuilder flowStr = new StringBuilder();
        
        flowStr.append("flows,\n");
        
        int key = 0;
        while(flows.hasNext()) {
            
            QuadraticBezierFlow flow = (QuadraticBezierFlow) flows.next();
            
            // Check to see if flow.stPoint is in nodeMap
            if(nodeMap.containsKey(flow.getStartPt())) {
                // Append the key and a comma to flowStr
                flowStr.append(nodeMap.get(flow.getStartPt()));
                flowStr.append(",");
            } else {
                // Add startPt to nodeMap, give it a value of key.
                nodeMap.put(flow.getStartPt(), Integer.toString(key));
                flowStr.append(Integer.toString(key));
                flowStr.append(",");
                key += 1;
            }
            
            // Same thing for endPoint
            // Check to see if flow.stPoint is in nodeMap
            if(nodeMap.containsKey(flow.getEndPt())) {
                // Append the key and a comma to flowStr
                flowStr.append(nodeMap.get(flow.getEndPt()));
                flowStr.append(",");
            } else {
                // Add startPt to nodeMap, give it a value of key.
                nodeMap.put(flow.getEndPt(), Integer.toString(key));
                flowStr.append(Integer.toString(key));
                flowStr.append(",");
                key += 1;
            }
            
            // Now the control point coordinates
            flowStr.append(flow.getCtrlPt().x);
            flowStr.append(",");
            flowStr.append(flow.getCtrlPt().y);
            flowStr.append(",");
            
            flowStr.append(flow.getValue());
            flowStr.append(",");
            flowStr.append("\n");
        }
        
        // Make a string of the nodes in nodeMap
        // Key first, then coordinates
        // ...how to loop through a HashMap?
        for(Map.Entry<Point, String> entry : nodeMap.entrySet()) {
            String id = entry.getValue();
            double x = entry.getKey().x;
            double y = entry.getKey().y;
            double val = entry.getKey().getValue();
            
            nodeStr.append(id);
            nodeStr.append(",");
            nodeStr.append(x);
            nodeStr.append(",");
            nodeStr.append(y);
            nodeStr.append(",");
            nodeStr.append(val);
            nodeStr.append("\n");
            
            
        }
        
        return nodeStr.append(flowStr.toString()).toString();
        */
    }
    
    
    
    
}
