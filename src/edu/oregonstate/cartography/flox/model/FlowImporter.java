package edu.oregonstate.cartography.flox.model;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.StringTokenizer;

/**
 *  Importer for text file with flows. One flow per line.
 *  Format: startX, startY, endX, endY
 *  Values are separated by space, coma or tab.
 * 
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class FlowImporter {

    private FlowImporter() {
    }

    public static ArrayList<Flow> readFlows(String filePath) throws IOException {
        ArrayList<Flow> flows = new ArrayList<>();

        try (BufferedReader inputStream = new BufferedReader(new FileReader(filePath))) {
            String l;
            while ((l = inputStream.readLine()) != null) {
                StringTokenizer tokenizer = new StringTokenizer(l, " ,\t");
                double x1 = Double.parseDouble(tokenizer.nextToken());
                double y1 = Double.parseDouble(tokenizer.nextToken());
                double x2 = Double.parseDouble(tokenizer.nextToken());
                double y2 = Double.parseDouble(tokenizer.nextToken());
                double value = Double.parseDouble(tokenizer.nextToken());
                // Flow flow = new BezierFlow(new Point(x1, y1), new Point(x2, y2));
                Flow flow = new QuadraticBezierFlow(new Point(x1, y1), new Point(x2, y2));
                flow.setValue(value);
                flows.add(flow);
            }
        }

        return flows;
    }
}
