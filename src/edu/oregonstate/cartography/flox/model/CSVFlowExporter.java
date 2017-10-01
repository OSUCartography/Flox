package edu.oregonstate.cartography.flox.model;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;

/**
 *
 * @author danielstephen
 */
public class CSVFlowExporter {

    private CSVFlowExporter() {
    }

    public static String exportToString(Iterator flowsToExport) {
        StringBuilder str = new StringBuilder();

        // Get a flow iterator
        while (flowsToExport.hasNext()) {
            
            ArrayList<Flow> flows = new ArrayList<>();
            Flow f = (Flow) flowsToExport.next();
            
            if (f instanceof FlowPair) {
                FlowPair biFlow = (FlowPair) f;
                flows.add(biFlow.createFlow1());
                flows.add(biFlow.createFlow2());
            } else {
                flows.add(f);
            }
            
            for (Flow flow : flows) {
                str.append(flow.getStartPt().x);
                str.append(",");
                str.append(flow.getStartPt().y);
                str.append(",");

                str.append(flow.getEndPt().x);
                str.append(",");
                str.append(flow.getEndPt().y);
                str.append(",");

                str.append(flow.getValue());
                str.append(",");

                str.append(flow.getStartPt().getValue());
                str.append(",");

                str.append(flow.getEndPt().getValue());
                str.append(",");

                str.append(flow.cPtX());
                str.append(",");
                str.append(flow.cPtY());
                str.append(",");

                if (flow.isLocked()) {
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
     * @param outFilePath The file to export to.
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
