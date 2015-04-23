package edu.oregonstate.cartography.flox;

import com.vividsolutions.jts.geom.GeometryCollection;
import edu.oregonstate.cartography.utils.FileUtils;
import edu.oregonstate.cartography.gui.ErrorDialog;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Flox {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {

        java.awt.EventQueue.invokeLater(new Runnable() {
            @Override
            public void run() {
                try {
                    // ask for import file
                    String inFilePath = FileUtils.askFile("Shapefile", true);
                    if (inFilePath == null) {
                        // user canceled
                        System.exit(0);
                    }

                    // read shapefile
                    GeometryCollection collection = new ShapeGeometryImporter().read(inFilePath);

                    // ask for export file
                    String outFilePath = FileUtils.askFile("SVG File", false);
                    if (outFilePath == null) {
                        // user canceled
                        System.exit(0);
                    }
                    outFilePath = FileUtils.forceFileNameExtension(outFilePath, "svg");
                    Collection<BezierFlow> flows = new ArrayList<BezierFlow>();
                    BezierFlow flow = new BezierFlow(0,0, 0, 100, 100, 100, 200, 0);
                    BezierFlow flow2 = new BezierFlow(10, 10, 10, 110, 110, 110, 210, 10);
                    flows.add(flow);
                    flows.add(flow2);
                    
                    // export to SVG
                    SVGFlowExporter exporter = new SVGFlowExporter(collection, flows,
                            "OSU Cartography Group", "Flox");
                    exporter.setSVGCanvasSize(800, 550);
                    OutputStream outputStream = new FileOutputStream(outFilePath);
                    exporter.export(outputStream);
                    
                    System.exit(0);
                } catch (IOException ex) {
                    Logger.getLogger(Flox.class.getName()).log(Level.SEVERE, null, ex);
                    ErrorDialog.showErrorDialog("An error occured.", "Flox Error", ex, null);
                }
            }
        });

    }

}
