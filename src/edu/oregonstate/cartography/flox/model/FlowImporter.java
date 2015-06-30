package edu.oregonstate.cartography.flox.model;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
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
     * Import flows from a reader. One flow per line.Format: startX, startY, endX,
     * endY Values are separated by space, coma or tab.
     *
     * @param reader
     * @return The imported flows.
     * @throws IOException
     */
    public static ArrayList<Flow> readFlows(BufferedReader reader) throws IOException {
        ArrayList<Flow> flows = new ArrayList<>();

        String l;
        boolean locked = false;
        while ((l = reader.readLine()) != null) {
            StringTokenizer tokenizer = new StringTokenizer(l, " ,\t");
            double x1 = Double.parseDouble(tokenizer.nextToken());
            double y1 = Double.parseDouble(tokenizer.nextToken());
            double x2 = Double.parseDouble(tokenizer.nextToken());
            double y2 = Double.parseDouble(tokenizer.nextToken());
            double value = Double.parseDouble(tokenizer.nextToken());
            
            // Are there control point coordinates?
            Point cPt = null;
            if (tokenizer.hasMoreTokens()) {
                double cx = Double.parseDouble(tokenizer.nextToken());
                double cy = Double.parseDouble(tokenizer.nextToken());
                cPt = new Point(cx, cy);
            }

            if (tokenizer.hasMoreTokens()) {
                if (Double.parseDouble(tokenizer.nextToken()) == 1) {
                    locked = true;
                }
            }

            QuadraticBezierFlow flow = new QuadraticBezierFlow(new Point(x1, y1), new Point(x2, y2));
            flow.setValue(value);

            if (cPt != null) {                
                flow.setControlPoint(cPt);
            }
            flow.setLocked(locked);

            flows.add(flow);
        }

        return flows;
    }

    /**
     * Import file with flows. One flow per line.Format: startX, startY, endX,
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
                Point point = new Point(x, y);
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
                Flow flow = new QuadraticBezierFlow(startPoint, endPoint);
                flow.setValue(value);
                flows.add(flow);
            }
        }

        return flows;
    }
}
