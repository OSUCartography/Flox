package edu.oregonstate.cartography.flox.model;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Iterator;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author danielstephen
 */
public class CSVFlowExporter {
    
    private final Model model;
    
    public CSVFlowExporter(Model model) {
        this.model = model;
    }
    
    
    /**
     * Export a CSV file containing the current flow model
     * @param exportPath The file to be exported to. This will be built by
     * FileUtils.askFile in the MainWindow.
     */
    public void export(String outFilePath) throws IOException {
        
        FileWriter fileWriter = null;
        
        try {
            fileWriter = new FileWriter(outFilePath);
            StringBuilder str = new StringBuilder();
            
            // Get a flow iterator
            Iterator flows = model.flowIterator();
            while(flows.hasNext()) {
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
                    
                    str.append(cFlow.value);
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
                    
                    str.append(qFlow.value);
                    str.append(",");
                    
                    str.append(qFlow.cPt.x);
                    str.append(",");
                    str.append(qFlow.cPt.y);
                    
                    str.append("\n");
                }
            }
            
            fileWriter.append(str.toString());
            
        } catch (Exception e) {
            String msg = e.getMessage() != null ? e.getMessage() : e.getClass().toString();
            throw new IOException("Export to CSV not possible. " + msg);
        } finally {
            
            try {
                fileWriter.flush();
                fileWriter.close();
            } catch (Exception e){
                String msg = e.getMessage() != null ? e.getMessage() : e.getClass().toString();
                throw new IOException("Errow while flushing/clising "
                        + "fileWriter");
            }
        }
        
    }
    
}
