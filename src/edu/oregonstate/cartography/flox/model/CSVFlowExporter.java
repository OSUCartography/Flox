package edu.oregonstate.cartography.flox.model;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Iterator;

/**
 *
 * @author danielstephen
 */
public class CSVFlowExporter {

    private CSVFlowExporter() {
    }

    public static String exportToString(Iterator flows) {
        StringBuilder str = new StringBuilder();

        // Get a flow iterator
        while (flows.hasNext()) {
            Flow flow = (Flow) flows.next();
            if (flow instanceof CubicBezierFlow) {
                CubicBezierFlow cFlow = (CubicBezierFlow) flow;

                str.append(cFlow.startPt.x);
                str.append(",");
                str.append(cFlow.startPt.y);
                str.append(",");

                str.append(cFlow.endPt.x);
                str.append(",");
                str.append(cFlow.endPt.y);
                str.append(",");

                str.append(cFlow.getValue());
                str.append(",");

                if (cFlow.isLocked()) {
                    str.append(1);
                } else {
                    str.append(0);
                }
                str.append("\n");

            } else {
                QuadraticBezierFlow qFlow = (QuadraticBezierFlow) flow;

                str.append(qFlow.startPt.x);
                str.append(",");
                str.append(qFlow.startPt.y);
                str.append(",");

                str.append(qFlow.endPt.x);
                str.append(",");
                str.append(qFlow.endPt.y);
                str.append(",");

                str.append(qFlow.getValue());
                str.append(",");

                str.append(qFlow.getCtrlPt().x);
                str.append(",");
                str.append(qFlow.getCtrlPt().y);
                str.append(",");

                if (qFlow.isLocked()) {
                    str.append(1);
                } else {
                    str.append(0);
                }

                str.append("\n");
            }
        }

        return str.toString();
    }

    /**
     * Export a CSV file containing the current flow model
     *
     * @param outFilePath The file to be exported to. This will be built by
     * FileUtils.askFile in the MainWindow.
     * @throws java.io.IOException
     */
    public static void export(String outFilePath, Iterator flows) throws IOException {

        FileWriter fileWriter = null;

        try {
            fileWriter = new FileWriter(outFilePath);
            fileWriter.append(exportToString(flows));
        } catch (Exception e) {
            String msg = e.getMessage() != null ? e.getMessage() : e.getClass().toString();
            throw new IOException("Export to CSV not possible. " + msg);
        } finally {
            try {
                fileWriter.flush();
                fileWriter.close();
            } catch (Exception e) {
                String msg = e.getMessage() != null ? e.getMessage() : e.getClass().toString();
                throw new IOException("Errow while flushing/clising "
                        + "fileWriter");
            }
        }

    }

}
