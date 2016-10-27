package edu.oregonstate.cartography.flox.model;

import edu.oregonstate.cartography.flox.gui.ErrorDialog;
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
                flow.setCtrlPt(cPt.x, cPt.y);
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
        String line = "";

        // read nodes
        int lineCounter = 0;
        try (BufferedReader inputStream = new BufferedReader(new FileReader(pointsFilePath))) {

            while ((line = inputStream.readLine()) != null) {
                ++lineCounter;
                StringTokenizer tokenizer = new StringTokenizer(line, " ,\t");
                String id = tokenizer.nextToken();
                double x = Double.parseDouble(tokenizer.nextToken());
                double y = Double.parseDouble(tokenizer.nextToken());
                double val = 1;

                if (tokenizer.hasMoreTokens()) {
                    val = Double.parseDouble(tokenizer.nextToken());
                }

                Point point = new Point(x, y, val);
                points.put(id, point);
            }
        } catch (Throwable exc) {
            line = line.trim();
            if (line.length() > 200) {
                line = line.substring(200) + "\u2026";
            }
            String msg = "<html>Could not read the node at line " + lineCounter + ".<br>"
                    + "Line: " + line + "/<html>";
            String title = "Node Reading Error";
            ErrorDialog.showErrorDialog(msg, title, exc, null);
        }

        // read flows
        lineCounter = 0;
        try (BufferedReader inputStream = new BufferedReader(new FileReader(flowsFilePath))) {
            while ((line = inputStream.readLine()) != null) {
                ++lineCounter;
                StringTokenizer tokenizer = new StringTokenizer(line, " ,\t");
                String startPtID = tokenizer.nextToken();
                String endPtID = tokenizer.nextToken();
                double value = Double.parseDouble(tokenizer.nextToken());
                Point startPoint = points.get(startPtID);
                if (startPoint == null) {
                    showMissingNodeError(lineCounter, line, startPtID);
                    return flows;
                }
                Point endPoint = points.get(endPtID);
                if (endPoint == null) {
                    showMissingNodeError(lineCounter, line, endPtID);
                    return flows;
                }
                Flow flow = new Flow(startPoint, endPoint, value);
                flows.add(flow);
            }
        } catch (Throwable exc) {
            line = line.trim();
            if (line.length() > 200) {
                line = line.substring(200) + "\u2026";
            }
            String msg = "<html>Could not read the flow at line " + lineCounter + ".<br>"
                    + "Line: " + line + "/<html>";
            String title = "Flow Reading Error";
            ErrorDialog.showErrorDialog(msg, title, exc, null);
        }

        return flows;
    }

    private static void showMissingNodeError(int lineCounter, String line, String missingNodeID) {
        line = line.trim();
        if (line.length() > 200) {
            line = line.substring(200) + "\u2026";
        }
        if (missingNodeID.length() > 100) {
            missingNodeID = missingNodeID.substring(100) + "\u2026";
        }
        String msg = "<html>Line " + lineCounter + " of the flows file references "
                + "a node with the identifier \"" + missingNodeID + "\".<br>"
                + "The nodes file does not contain a node with this identifier.<br>"
                + "Line: " + line + "/<html>";
        String title = "Missing Node Error";
        ErrorDialog.showErrorDialog(msg, title);
    }
}
