package edu.oregonstate.cartography.flox.model;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
import java.util.StringTokenizer;
import javax.xml.bind.annotation.adapters.XmlAdapter;

public class GraphSerializer extends XmlAdapter<String, Graph> {

    @Override
    public Graph unmarshal(String s) throws IOException {
        
        // Create a reader to interpret the xml
        BufferedReader reader = new BufferedReader(new StringReader(s));

        // Create a hashmap to store the points with keys
        HashMap<String, Point> points = new HashMap<>();
        
        // Empty arraylist to store flows
        ArrayList<Flow> flows = new ArrayList<>();
        
        // Empty graph to add nodes and flows
        Graph graph = new Graph();
        
        // The first line of the xml file is the number of nodes in the graph
        int numberOfNodes = Integer.parseInt(reader.readLine());

        // If there are no nodes, return an empty graph
        if(numberOfNodes == 0) {
            return graph;
        }
        
        // Create a String object to store text from the xml file
        String l;

        // Read in node data, add nodes to the points HashMap
        for (int i = 0; i < numberOfNodes; i++) {
            l = reader.readLine();
            StringTokenizer tokenizer = new StringTokenizer(l, " ,\t");
            String id = tokenizer.nextToken();
            double x = Double.parseDouble(tokenizer.nextToken());
            double y = Double.parseDouble(tokenizer.nextToken());
            double nodeValue = Double.parseDouble(tokenizer.nextToken());
            Point point = new Point(x, y);
            point.setValue(nodeValue);
            points.put(id, point);
        }

        // Add all the points to the graph
        for (Map.Entry<String, Point> entry : points.entrySet()) {
            graph.addNode(entry.getValue());
        }
        
        // Read in flow data, add them to the flows ArrayList
        while ((l = reader.readLine()) != null) {

            StringTokenizer tokenizer = new StringTokenizer(l, " ,\t");
            boolean locked = false;

            String startPtID = tokenizer.nextToken();
            String endPtID = tokenizer.nextToken();
            double cPtX = Double.parseDouble(tokenizer.nextToken());
            double cPtY = Double.parseDouble(tokenizer.nextToken());
            double flowValue = Double.parseDouble(tokenizer.nextToken());

            if (Double.parseDouble(tokenizer.nextToken()) == 1) {
                locked = true;
            }

            Point startPoint = points.get(startPtID);
            Point endPoint = points.get(endPtID);
            Point cPoint = new Point(cPtX, cPtY);
            QuadraticBezierFlow flow = new QuadraticBezierFlow(startPoint, endPoint);
            flow.setValue(flowValue);
            flow.setControlPoint(cPoint);
            flow.setLocked(locked);
            flows.add(flow);
        }

        // Add all the flows to the graph
        for (Flow flow : flows) {
            graph.addFlow(flow);
        }

        return graph;
    }

    @Override
    public String marshal(Graph graph) {

        // A map of the nodes, with the node itself as the key.
        // The string will be a new ID number.
        HashMap<Point, String> points = new HashMap<>();

        // Get the flows
        Iterator flows = graph.flowIterator();
        
        // Get the nodes
        Iterator nodes = graph.nodeIterator();

        // Make stringbuilders for nodes and flows
        StringBuilder nodeStr = new StringBuilder();
        StringBuilder flowStr = new StringBuilder();

        // Populate the points HashMap with all the nodes in the graph.
        int key = 0;
        while (nodes.hasNext()) {
            Point node = (Point) nodes.next();
            points.put(node, Integer.toString(key));
            key +=1;
        }
        
        // Make a string of all the flows
        while (flows.hasNext()) {

            QuadraticBezierFlow flow = (QuadraticBezierFlow) flows.next();

            flowStr.append(points.get(flow.getStartPt()));
            flowStr.append(",");
            
            // Append the key and a comma to flowStr
            flowStr.append(points.get(flow.getEndPt()));
            flowStr.append(",");
             
            // Append the control point coordinates
            flowStr.append(flow.getCtrlPt().x);
            flowStr.append(",");
            flowStr.append(flow.getCtrlPt().y);
            flowStr.append(",");

            flowStr.append(flow.getValue());
            flowStr.append(",");

            // Append the locked status
            if (flow.isLocked()) {
                flowStr.append(1);
            } else {
                flowStr.append(0);
            }
            flowStr.append("\n");
        }

        // Make a string of the nodes in nodeMap
        // Key first, then coordinates
        nodeStr.append(points.size());
        nodeStr.append("\n");
        for (Map.Entry<Point, String> entry : points.entrySet()) {
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

        // Append the flowStr to the end of nodeStr and return the whole thing
        return nodeStr.append(flowStr.toString()).toString();

    }

}
