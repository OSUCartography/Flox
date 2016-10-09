package edu.oregonstate.cartography.simplefeature;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiPolygon;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import edu.oregonstate.cartography.utils.ColorUtils;
import java.awt.Color;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.text.DecimalFormat;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Source;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

/**
 * Exporter for SVG format.
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class SVGExporter {

    protected static final String SVGNAMESPACE = "http://www.w3.org/2000/svg";
    private static final String XLINKNAMESPACE = "http://www.w3.org/1999/xlink";
    private static final String XMLEVENTSNAMESPACE = "http://www.w3.org/2001/xml-events";

    /**
     * id of point symbol used by SVG use elements for Points
     */
    private static final String POINT_SYMBOL_ID = "PointSymbol";
    
    /**
     * Radius of point symbol for SVG use element for Points
     */
    private static final double POINT_SYMBOL_RADIUS = 5;

    /**
     * Factor to convert from millimeter to pixels. Assumes 72 pixels per inch.
     */
    protected static final double MM2PX = 72. / 2.54 / 10.;

    /**
     * rounding of coordinates
     */
    protected final DecimalFormat df = new DecimalFormat("#.##");

    /**
     * Name of the author creating the SVG document
     */
    private final String authorName;

    /**
     * Name of the application creating the SVG document
     */
    private final String applicationName;

    /**
     * Geometry features to export.
     */
    private final GeometryCollection collection;

    /**
     * Bounding box of the geometry collection that is exported.
     */
    protected final Envelope bb;

    /**
     * Scale factor to fit geometry to SVG canvas.
     */
    protected double scale;

    /**
     * Width of the SVG canvas in pixel.
     */
    protected double canvasWidth;

    /**
     * Height of the SVG canvas in pixel.
     */
    protected double canvasHeight;

    /**
     * Creates a new SVGExporter.
     *
     * @param collection The geometries to export.
     * @param authorName Name of the author creating the SVG document.
     * @param applicationName Name of the application creating the SVG document.
     */
    public SVGExporter(GeometryCollection collection,
            String authorName, String applicationName) {
        this.collection = collection;
        this.authorName = authorName;
        this.applicationName = applicationName;
        bb = collection.getEnvelopeInternal();
        if (bb == null) {
            throw new IllegalArgumentException("SVG export: Empty bounding box.");
        }
        setSVGCanvasSize(600, 450);
    }

    /**
     *
     * @param width canvas width in pixels
     * @param height canvas height in pixels
     */
    public final void setSVGCanvasSize(double width, double height) {
        canvasWidth = width;
        canvasHeight = height;
        double vScale = bb.getHeight() / (height / 1000 / MM2PX);
        double hScale = bb.getWidth() / (width / 1000 / MM2PX);
        scale = vScale > hScale ? vScale : hScale;
    }

    /**
     * Exports a GeometryCollection to a SVG document.
     *
     * @param outputStream The OutputStream that will be used to export to.
     * @throws java.io.IOException Throws an exception if export is not
     * possible.
     */
    public void export(OutputStream outputStream)
            throws IOException {
        try {
            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document document = builder.newDocument();

            // construct the SVG root element
            Element svgRootElement = createSVGRootElement(document);
            document.appendChild(svgRootElement);

            // add a description element
            appendDescription(svgRootElement, document);

            // convert GeometryCollection to SVG DOM
            // appendGeometryCollection(this.collection, svgRootElement, document);
            append(svgRootElement, document);

            // Prepare the output file
            OutputStreamWriter outputStreamWriter
                    = new OutputStreamWriter(outputStream, "utf-8");
            StreamResult result = new StreamResult(outputStreamWriter);

            // Write the DOM document to the file
            /*
             There is a bug in Java 1.5: XML output is not indented.
             To work around this bug:
             http://bugs.sun.com/bugdatabase/view_bug.do?bug_id=6296446
             (1)set the indent-number in the transformerfactory
             TransformerFactory tf = new TransformerFactory.newInstance();
             tf.setAttribute("indent-number", new Integer(2));
             (2)enable the indent in the transformer
             Transformer t = tf.newTransformer();
             t.setOutputProperty(OutputKeys.INDENT, "yes");
             (3)wrap the otuputstream with a writer (or bufferedwriter)
             t.transform(new DOMSource(doc),
             new StreamResult(new OutputStreamWriter(out, "utf-8"));
             You must do (3) to workaround a "buggy" behavior of the
             xml handling code.
             */
            TransformerFactory tf = TransformerFactory.newInstance();
            tf.setAttribute("indent-number", new Integer(2));
            Transformer xformer = tf.newTransformer();
            xformer.setOutputProperty(OutputKeys.INDENT, "yes");
            Source source = new DOMSource(document);
            xformer.transform(source, result);

            // don't add doctype to SVG files. see http://jwatt.org/svg/authoring/
            /*
             xformer.setOutputProperty(OutputKeys.DOCTYPE_PUBLIC, svgIdentifier);
             xformer.setOutputProperty(OutputKeys.DOCTYPE_SYSTEM, svgDTD);
             */
            outputStreamWriter.flush();

        } catch (Exception e) {
            String msg = e.getMessage() != null ? e.getMessage() : e.getClass().toString();
            throw new IOException("Export to SVG not possible. " + msg);
        }
    }

    /**
     * Creates the top level SVG element.
     *
     * @param document The document to export to.
     * @return
     */
    protected Element createSVGRootElement(Document document) {
        // create the main svg element
        Element svg = (Element) document.createElementNS(SVGNAMESPACE, "svg");

        // specify a namespace prefix on the 'svg' element, which means that
        // SVG is the default namespace for all elements within the scope of
        // the svg element with the xmlns attribute:
        // See http://www.w3.org/TR/SVG11/struct.html#SVGElement
        // and http://jwatt.org/svg/authoring/
        svg.setAttribute("xmlns", SVGNAMESPACE);
        svg.setAttribute("xmlns:xlink", XLINKNAMESPACE);
        svg.setAttribute("xmlns:ev", XMLEVENTSNAMESPACE);
        svg.setAttribute("version", "1.0");
        svg.setAttribute("preserveAspectRatio", "xMinYMin");

        String wStr = df.format(canvasWidth);
        String hStr = df.format(canvasHeight);
        svg.setAttribute("width", wStr);
        svg.setAttribute("height", hStr);

        // Define the viewBox.
        String viewBoxStr = "0 0 " + wStr + " " + hStr;
        svg.setAttribute("viewBox", viewBoxStr);
        return svg;
    }

    /**
     * Append author and application name to SVG document.
     *
     * @param svgRootElement Destination element.
     * @param document SVG document.
     */
    protected void appendDescription(Element svgRootElement, Document document) {

        StringBuilder str = new StringBuilder();
        if (authorName != null && authorName.length() > 0) {
            str.append("Author:");
            str.append(authorName);
            str.append(" - ");
        }

        if (applicationName != null && applicationName.length() > 0) {
            str.append("Generator:");
            str.append(applicationName);
            str.append(" - ");
        }
        str.append("Date:");
        str.append(java.util.Calendar.getInstance().getTime());

        // create a description element
        Element desc = (Element) document.createElementNS(SVGNAMESPACE, "desc");
        desc.appendChild(document.createTextNode(str.toString()));

        // append description element
        svgRootElement.appendChild(desc);
    }

    /**
     * Appends a GeometryCollection to a DOM element.
     *
     * @param collection The GeometryCollection to convert.
     * @param parent The parent element that will contain the passed
     * GeometryCollection. This is not always the SVG root element.
     * @param svgRootElement SVG root element containing all other elements
     * @param document The DOM.
     * @return The element containing all geometry features.
     */
    protected Element appendGeometryCollection(GeometryCollection collection,
            Element parent, Element svgRootElement, Document document) {

        Element g = document.createElementNS(SVGNAMESPACE, "g");
        if (g == null) {
            return null;
        }
        setVectorStyle(g, Color.BLACK, 1, null);
        parent.appendChild(g);

        final int nbrObj = collection.getNumGeometries();
        for (int i = 0; i < nbrObj; i++) {
            Geometry geometry = collection.getGeometryN(i);
            String svgPath = null;
            if (geometry instanceof LineString) {
                svgPath = convertToSVGPath((LineString) geometry);
            } else if (geometry instanceof Polygon) {
                svgPath = convertToSVGPath((Polygon) geometry);
            } else if (geometry instanceof MultiPolygon) {
                svgPath = convertToSVGSymbol((MultiPolygon) geometry);
            } else if (geometry instanceof Point) {
                addPoint(document, svgRootElement, g, (Point)geometry);
            } else if (geometry instanceof GeometryCollection) {
                appendGeometryCollection((GeometryCollection) geometry, svgRootElement, g, document);
            } else {
                System.err.println("SVG exporter found unsupported geometry type");
            }
            if (svgPath != null) {
                Element pathElement = (Element) document.createElementNS(SVGNAMESPACE, "path");
                pathElement.setAttribute("d", svgPath);
                g.appendChild(pathElement);
            }
        }
        return g;
    }

    /**
     * Converts a LineString to a SVG string for the d attribute of a path
     * element.
     *
     * @param lineString The LineString to convert.
     * @return A string for the d attribute of a SVG path element.
     */
    private String convertToSVGPath(LineString lineString) {
        StringBuilder str = new StringBuilder();
        Point startPoint = lineString.getStartPoint();
        str.append("M");
        str.append(df.format(xToSVGCanvas(startPoint.getX())));
        str.append(" ");
        str.append(df.format(yToSVGCanvas(startPoint.getY())));

        int numPoints = lineString.getNumPoints();
        for (int i = 1; i < numPoints; i++) {
            Point point = lineString.getPointN(i);
            str.append(" L");
            str.append(df.format(xToSVGCanvas(point.getX())));
            str.append(" ");
            str.append(df.format(yToSVGCanvas(point.getY())));
        }

        return str.toString();
    }

    /**
     * Converts a Polygon to a SVG string for the d attribute of a path element.
     *
     * @param polygon The geometry to convert.
     * @return A string for the d attribute of a SVG path element.
     */
    private String convertToSVGPath(Polygon polygon) {
        String svgPath = convertToSVGPath(polygon.getExteriorRing());
        int numInteriorRings = polygon.getNumInteriorRing();
        for (int i = 0; i < numInteriorRings; i++) {
            LineString interiorRing = polygon.getInteriorRingN(i);
            svgPath += " ";
            svgPath += convertToSVGPath(interiorRing);
        }
        return svgPath;
    }

    /**
     * Add a symbol element to the SVG root element if no such symbol exists
     * yet.
     *
     * @param document document
     * @param svgRootElement SVG root element
     */
    private void addSVGPointSymbol(Document document, Element svgRootElement, Element parent) {
        Element pointSymbol = document.getElementById(POINT_SYMBOL_ID);
        if (pointSymbol == null) {
            pointSymbol = (Element) document.createElementNS(SVGNAMESPACE, "symbol");
            pointSymbol.setAttribute("id", POINT_SYMBOL_ID);
            pointSymbol.setIdAttribute("id", true);
            double r = POINT_SYMBOL_RADIUS;
            String viewBoxStr = String.format("%f %f %f %f", -r, -r, 2 * r, 2 * r);
            pointSymbol.setAttribute("viewBox", viewBoxStr);
            Element circle = (Element) document.createElementNS(SVGNAMESPACE, "circle");
            circle.setAttribute("r", Double.toString(POINT_SYMBOL_RADIUS));
            pointSymbol.appendChild(circle);
            svgRootElement.insertBefore(pointSymbol, parent);
        }
    }

    /**
     * Creates a SVG use element for a Point and adds it to the parent element.
     * @param document SVG document
     * @param parent parent element to receive the use element
     * @param point point to convert to use element
     */
    private void addPoint(Document document, Element svgRootElement, Element parent, Point point) {
        
        // add SVG symbol if this is the first point
        addSVGPointSymbol(document, svgRootElement, parent);
        
        // create a SVG use element in the format used by Illustrator CS6
        // <use xlink:href="#PointSymbol"  width="10" height="10" x="-5" y="-5" transform="matrix(1 0 0 -1 100 50)" overflow="visible"/>
        Element useElement = (Element) document.createElementNS(SVGNAMESPACE, "use");
        useElement.setAttribute("xlink:href", "#" + POINT_SYMBOL_ID);
        useElement.setAttribute("width", Double.toString(POINT_SYMBOL_RADIUS * 2));
        useElement.setAttribute("height", Double.toString(POINT_SYMBOL_RADIUS * 2));
        useElement.setAttribute("x", Double.toString(POINT_SYMBOL_RADIUS));
        useElement.setAttribute("y", Double.toString(POINT_SYMBOL_RADIUS));
        useElement.setAttribute("overflow", "visible");

        String xStr = df.format(xToSVGCanvas(point.getX()));
        String yStr = df.format(yToSVGCanvas(point.getY()));
        String trasnformStr = String.format("matrix(1 0 0 -1 %s %s)", xStr, yStr);
        useElement.setAttribute("transform", trasnformStr);
        
        parent.appendChild(useElement);
    }

    /**
     * Converts a MultiPolygon to a SVG string for the d attribute of a path
     * element.
     *
     * @param multiPolygon The geometry to convert.
     * @return A string for the d attribute of a SVG path element.
     */
    private String convertToSVGSymbol(MultiPolygon multiPolygon) {
        int numPolygons = multiPolygon.getNumGeometries();
        String svgPath = "";
        for (int i = 0; i < numPolygons; i++) {
            Polygon polygon = (Polygon) multiPolygon.getGeometryN(i);
            svgPath += " ";
            svgPath += convertToSVGPath(polygon);
        }
        return svgPath;
    }

    /**
     * Define styling of an element
     *
     * @param element Element to set vector style for.
     * @param strokeColor The color for lines. If null, no stroke is drawn.
     * @param strokeWidth The width of lines.
     * @param fillColor The fill color.
     */
    protected void setVectorStyle(Element element,
            Color strokeColor, double strokeWidth, Color fillColor) {

        assert (element != null);

        // stroke color
        String strokeColorStr;
        if (strokeColor == null) {
            strokeColorStr = "none";
        } else {
            strokeColorStr = ColorUtils.colorToCSSString(strokeColor);
        }

        // stroke width
        element.setAttribute("stroke", strokeColorStr);
        if (strokeColor != null) {
            strokeWidth = Double.max(0, strokeWidth);
            element.setAttribute("stroke-width", Double.toString(strokeWidth));
        }

        // fill color
        String fillColorStr;
        if (fillColor == null) {
            fillColorStr = "none";
        } else {
            fillColorStr = ColorUtils.colorToCSSString(fillColor);
        }
        element.setAttribute("fill", fillColorStr);
    }

    /**
     * Transforms a horizontal x coordinate (usually in meters) to pixels. Takes
     * the scale and bounding box defined by the PageFormat into account.
     *
     * @param x The horizontal coordinate.
     * @return Returns the coordinate in pixels.
     */
    protected double xToSVGCanvas(double x) {
        double west = bb.getMinX();
        return (x - west) / scale * 1000 * MM2PX;
    }

    /**
     * Transforms a vertical y coordinate to the scale and bounding box defined
     * by the PageFormat.
     *
     * @param y The vertical coordinate.
     * @return Returns the coordinate in the page coordinate system.
     */
    protected double yToSVGCanvas(double y) {
        double north = bb.getMaxY();
        return (north - y) / scale * 1000 * MM2PX;
    }

    /**
     * Give derived classes opportunity to add custom data.
     *
     * @param svgRootElement
     * @param document
     */
    protected void append(Element svgRootElement, Document document) {

    }

}
