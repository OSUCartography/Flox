package edu.oregonstate.cartography.flox.model;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.StringTokenizer;

/**
 * Importer for text file with flows.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class FlowImporter {

    private FlowImporter() {
    }

    /**
     * Import flows from a reader. One flow per line. Values are separated by
     * space, coma or tab. Format: startX, startY, endX, endY, [flow value],
     * [start node value], [end node value] [controlX], [controlY], [lock flag].
     * Optional values are indicated with [].
     *
     * @param reader read from this reader
     * @return The imported flows.
     * @throws IOException
     */
    public static ArrayList<Flow> readFlows(BufferedReader reader) throws IOException {
        ArrayList<Flow> flows = new ArrayList<>();

        String l;
        while ((l = reader.readLine()) != null) {
            boolean locked = false;
            StringTokenizer tokenizer = new StringTokenizer(l, " ,\t");
            double x1 = Double.parseDouble(tokenizer.nextToken());
            double y1 = Double.parseDouble(tokenizer.nextToken());
            double x2 = Double.parseDouble(tokenizer.nextToken());
            double y2 = Double.parseDouble(tokenizer.nextToken());

            // flow value
            double value = Model.DEFAULT_FLOW_VALUE;
            if (tokenizer.hasMoreTokens()) {
                value = Double.parseDouble(tokenizer.nextToken());
            }

            // node values
            double startNodeValue = Model.DEFAULT_NODE_VALUE;
            double endNodeValue = Model.DEFAULT_NODE_VALUE;
            if (tokenizer.hasMoreTokens()) {
                startNodeValue = Double.parseDouble(tokenizer.nextToken());
            }
            if (tokenizer.hasMoreTokens()) {
                endNodeValue = Double.parseDouble(tokenizer.nextToken());
            }

            // control point coordinates
            Point cPt = null;
            if (tokenizer.hasMoreTokens()) {
                double cx = Double.parseDouble(tokenizer.nextToken());
                double cy = Double.parseDouble(tokenizer.nextToken());
                cPt = new Point(cx, cy);
            }

            if (tokenizer.hasMoreTokens()) {
                locked = (Double.parseDouble(tokenizer.nextToken()) == 1);
            }

            Point startNode = new Point(x1, y1, startNodeValue);
            Point endNode = new Point(x2, y2, endNodeValue);
            Flow flow = new Flow(startNode, endNode, value);

            if (cPt != null) {
                flow.setControlPoint(cPt);
            }
            flow.setLocked(locked);

            flows.add(flow);
        }

        return flows;
    }

    /**
     * Import file with flows. One flow per line. Format: startX, startY, endX,
     * endY Values are separated by space, coma or tab.
     *
     * @param filePath
     * @return The imported flows.
     * @throws IOException
     */
    public static ArrayList<Flow> readFlows(String filePath) throws IOException {
        try (BufferedReader inputStream = new BufferedReader(new FileReader(filePath))) {
            return readFlows(inputStream);
        }
    }

    /**
     * Read flows from a file with points and a file with flows connecting the
     * points.
     *
     * @param pointsFilePath Path for the points file.
     * @param flowsFilePath Path for the flows file.
     * @return The imported flows.
     * @throws IOException
     */
    public static ArrayList<Flow> readFlows(String pointsFilePath, String flowsFilePath) throws IOException {
        HashMap<String, Point> points = new HashMap<>();
        ArrayList<Flow> flows = new ArrayList<>();

        // read points
        try (BufferedReader inputStream = new BufferedReader(new FileReader(pointsFilePath))) {
            String l;
            while ((l = inputStream.readLine()) != null) {
                StringTokenizer tokenizer = new StringTokenizer(l, " ,\t");
                String id = tokenizer.nextToken();
                double x = Double.parseDouble(tokenizer.nextToken());
                double y = Double.parseDouble(tokenizer.nextToken());
                double val = 1;

                if (tokenizer.hasMoreTokens()) {
                    val = Double.parseDouble(tokenizer.nextToken());
                };

                Point point = new Point(x, y, val);
                points.put(id, point);
            }
        }

        // read flows
        try (BufferedReader inputStream = new BufferedReader(new FileReader(flowsFilePath))) {
            String l;
            while ((l = inputStream.readLine()) != null) {
                StringTokenizer tokenizer = new StringTokenizer(l, " ,\t");
                String startPtID = tokenizer.nextToken();
                String endPtID = tokenizer.nextToken();
                double value = Double.parseDouble(tokenizer.nextToken());
                Point startPoint = points.get(startPtID);
                Point endPoint = points.get(endPtID);
                Flow flow = new Flow(startPoint, endPoint, value);
                flows.add(flow);
            }
        }

        return flows;
    }
}
