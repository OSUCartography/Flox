package edu.oregonstate.cartography.flox;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.io.ParseException;
import com.vividsolutions.jts.io.WKTReader;
import edu.oregonstate.cartography.utils.FileUtils;
import edu.oregonstate.cartography.gui.ErrorDialog;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
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
                    String inFilePath = "/Users/jennyb/Desktop/test.wkt";
                    // String inFilePath = FileUtils.askFile("WKT File", true);
                    if (inFilePath == null) {
                        // user canceled
                        System.exit(0);
                    }

                    // read WKT file
                    CharSequence chars = FileUtils.charSequenceFromFile(inFilePath);
                    String wktString = /*"GEOMETRYCOLLECTION(LINESTRING (0 0, 1000 1000, 1000 2000))"; */ chars.toString();
                    Geometry geometry = new WKTReader().read(wktString);
                    GeometryCollection collection;
                    if (geometry instanceof GeometryCollection) {
                        collection = (GeometryCollection)geometry;
                    } else {
                        collection = new GeometryCollection(new Geometry[] {geometry}, null);
                    }
                    
                    // ask for export file
                    String outFilePath = "/Users/jennyb/Desktop/out.svg";
                    // String outFilePath = FileUtils.askFile("SVG File", false);
                    if (outFilePath == null) {
                        // user canceled
                        System.exit(0);
                    }
                    
                    System.out.println(inFilePath);
                    System.out.println(outFilePath);
                    
                    // export to SVG
                    SVGExporter exporter = new SVGExporter(collection, "OSU Cartography Group", "Flox");
                    OutputStream outputStream = new FileOutputStream(outFilePath);
                    exporter.export(outputStream);

                } catch (IOException | ParseException ex) {
                    Logger.getLogger(Flox.class.getName()).log(Level.SEVERE, null, ex);
                    ErrorDialog.showErrorDialog("An error occured.", "Flox Error", ex, null);
                } finally {
                    System.exit(0);
                }
            }
        });

    }

}
