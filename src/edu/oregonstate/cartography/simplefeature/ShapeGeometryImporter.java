package edu.oregonstate.cartography.simplefeature;

import edu.oregonstate.cartography.utils.MixedEndianDataInputStream;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.PrecisionModel;
import com.vividsolutions.jts.operation.polygonize.Polygonizer;
import edu.oregonstate.cartography.utils.URLUtils;
import java.io.BufferedInputStream;
import java.io.EOFException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;

/**
 * An importer for ESRI shape files. This importer only reads geometry from .shp
 * files.
 *
 * @author Bernhard Jenny
 */
public class ShapeGeometryImporter {

    /**
     * Identifiers for different shape types.
     */
    private static final int NULLSHAPE = 0;
    private static final int POINT = 1;
    private static final int POLYLINE = 3;
    private static final int POLYGON = 5;
    private static final int MULTIPOINT = 8;
    private static final int POINTZ = 11;
    private static final int POLYLINEZ = 13;
    private static final int POLYGONZ = 15;
    private static final int MULTIPOINTZ = 18;
    private static final int POINTM = 21;
    private static final int POLYLINEM = 23;
    private static final int POLYGONM = 25;
    private static final int MULTIPOINTM = 28;
    private static final int MULTIPATCH = 31;   // not supported yet

    /**
     * ESRI shapefile magic code at the beginning of the .shp file.
     */
    private static final int FILE_CODE = 9994;

    private final GeometryFactory geometryFactory = new GeometryFactory();
    private final PrecisionModel precisionModel = geometryFactory.getPrecisionModel();

    ;

    /**
     * Creates a new instance of ShapeGeometryImporter
     */
    public ShapeGeometryImporter() {
    }

    protected java.net.URL findDataURL(java.net.URL url) {

        if (url == null || url.getPath().length() < 5) {
            return null;
        }

        String dataFileExtension = this.getLowerCaseDataFileExtension();
        String lowerCaseFilePath = url.getPath().toLowerCase();
        if (lowerCaseFilePath.endsWith("." + dataFileExtension)) {
            return url;
        }

        final boolean is_shp_sibling
                = lowerCaseFilePath.endsWith(".dbf")
                || lowerCaseFilePath.endsWith(".prj")
                || lowerCaseFilePath.endsWith(".sbn")
                || lowerCaseFilePath.endsWith(".sbx")
                || lowerCaseFilePath.endsWith(".shx");

        if (!is_shp_sibling) {
            return null;
        }

        url = URLUtils.replaceFileExtension(url, dataFileExtension);
        if (URLUtils.resourceExists(url)) {
            return url;
        }
        url = URLUtils.replaceFileExtension(url, dataFileExtension.toUpperCase());
        return URLUtils.resourceExists(url) ? url : null;
    }

    protected String getLowerCaseDataFileExtension() {
        return "shp";
    }

    private java.net.URL findSHXURL(java.net.URL url) {
        if (url == null || url.getPath().length() < 5) {
            return null;
        }
        url = URLUtils.replaceFileExtension(url, "shx");
        if (!URLUtils.resourceExists(url)) {
            url = URLUtils.replaceFileExtension(url, "SHX");
        }
        return URLUtils.resourceExists(url) ? url : null;
    }

    protected BufferedInputStream findInputStream(java.net.URL url) throws IOException {
        BufferedInputStream bis = new BufferedInputStream(url.openStream());
        return bis;
    }

    public GeometryCollection read(String filePath) throws IOException {
        return read(URLUtils.filePathToURL(filePath));
    }

    public GeometryCollection read(java.net.URL url) throws IOException {
        MixedEndianDataInputStream is = null;
        try {
            url = this.findDataURL(url);
            if (url == null) {
                return null;
            }

            BufferedInputStream bis = findInputStream(url);
            is = new MixedEndianDataInputStream(bis);

            // magic code is 9994
            int fileCode = is.readInt();
            if (fileCode != FILE_CODE) {
                return null;
            }

            is.skipBytes(5 * 4);
            int fileLength = is.readInt() * 2;

            // read version and shape type
            int version = is.readLittleEndianInt();
            int shapeType = is.readLittleEndianInt();

            // skip bounding box and four double values
            is.skipBytes(8 * 8);

            // Read all features stored in records. The shp file does not contain
            // the number of records present in the file. The shx file can be 
            // used to extract this information.
            // If the shx file cannot be found, -1 is returned.
            final int recordCount = this.readRecordCountFromSHXFile(url);
            final int[] recOffsets = readSHXFile(url);

            // Read until as many records as specified in the shx file are 
            // imported or until the end of file is reached and an EOFException
            // is thrown.
            int currentRecord = 0;
            int currentPos = 100;

            ArrayList<Geometry> geometries = new ArrayList<>();
            try {
                while (true) {

                    // move to beginning of record
                    is.skipBytes(recOffsets[currentRecord] - currentPos);
                    currentPos = recOffsets[currentRecord];
                    currentPos += readRecord(is, geometries);

                    if (++currentRecord == recordCount) {
                        break;
                    }
                }
            } catch (EOFException ignore) {
                // EOFException indicates that all records have been read.
            }

            Geometry[] array = new Geometry[geometries.size()];
            return geometryFactory.createGeometryCollection((Geometry[]) geometries.toArray(array));
        } finally {
            if (is != null) {
                is.close();
            }
        }
    }

    public String getImporterName() {
        return "Shape Importer";
    }

    private int readRecord(MixedEndianDataInputStream is,
            ArrayList<Geometry> geometries)
            throws IOException {

        final int recordNumber = is.readInt();
        final int contentLength = is.readInt() * 2;
        // content is at least one int (i.e. the ShapeType)
        if (contentLength < 4) {
            throw new EOFException("Negative record length");
        }

        final int shapeType = is.readLittleEndianInt();
        int recordBytesRead = 8 + 4; // header is 8 bytes, shapeType is 4 bytes

        switch (shapeType) {
            case NULLSHAPE:
                break;
            case POINT:
            case POINTZ:
            case POINTM:
                recordBytesRead += readPoint(is, geometries);
                break;
            case MULTIPOINT:
            case MULTIPOINTZ:
            case MULTIPOINTM:
                recordBytesRead += readMultipoint(is, geometries);
                break;
            case POLYLINE:
            case POLYLINEZ:
            case POLYLINEM:
                recordBytesRead += readPolyline(is, geometries);
                break;
            case POLYGON:
            case POLYGONZ:
            case POLYGONM:
                recordBytesRead += readPolygon(is, geometries);
                break;
            case MULTIPATCH:
                throw new IOException("Multipatch shapefiles are not supported.");
            default:
                throw new IOException("Shapefile contains unsupported "
                        + "geometry type: " + shapeType);
        }

        return recordBytesRead;
    }

    private Coordinate toPreciseCoordinate(double x, double y)
            throws IOException {

        Coordinate coord = new Coordinate(x, y);
        precisionModel.makePrecise(coord);
        return coord;
    }

    private int readPoint(MixedEndianDataInputStream is, ArrayList<Geometry> geometries)
            throws IOException {

        final double x = is.readLittleEndianDouble();
        final double y = is.readLittleEndianDouble();
        Point point = geometryFactory.createPoint(toPreciseCoordinate(x, y));
        geometries.add(point);
        return 2 * 8;
    }

    private int readMultipoint(MixedEndianDataInputStream is,
            ArrayList<Geometry> geometries) throws IOException {

        is.skipBytes(4 * 8); // skip bounding box
        final int numPoints = is.readLittleEndianInt();
        for (int ptID = 0; ptID < numPoints; ptID++) {
            readPoint(is, geometries);
        }
        return 4 * 8 + 4 + numPoints * 2 * 8;
    }

    private void readLineStrings(MixedEndianDataInputStream is,
            int numParts, int numPoints, ArrayList<Geometry> geometries) throws IOException {

        // read indices into point array
        int[] pointIds = new int[numParts];
        for (int partID = 0; partID < numParts; partID++) {
            pointIds[partID] = is.readLittleEndianInt();
        }

        // read point array
        double[] x = new double[numPoints];
        double[] y = new double[numPoints];
        for (int ptID = 0; ptID < numPoints; ptID++) {
            x[ptID] = is.readLittleEndianDouble();
            y[ptID] = is.readLittleEndianDouble();
        }

        LineString[] lineStrings = new LineString[numParts];
        
        for (int partID = 0; partID < numParts; partID++) {

            int firstPtID = pointIds[partID];
            int lastPtID = partID + 1 < numParts ? pointIds[partID + 1] : numPoints;

            // part must have at least two points
            if ((lastPtID - firstPtID) < 2) {
                continue;
            }

            ArrayList<Coordinate> coordinates = new ArrayList<>();
            for (int ptID = firstPtID; ptID < lastPtID; ptID++) {
                coordinates.add(toPreciseCoordinate(x[ptID], y[ptID]));
            }
            Coordinate[] array = new Coordinate[coordinates.size()];
            array = (Coordinate[]) coordinates.toArray(array);
            lineStrings[partID] = geometryFactory.createLineString(array);
        }
        if (numParts == 1) {
            geometries.add(lineStrings[0]);
        } else {
            MultiLineString multiLineString = geometryFactory.createMultiLineString(lineStrings);
            geometries.add(multiLineString);
        }        
    }

    private int readPolyline(MixedEndianDataInputStream is,
            ArrayList<Geometry> geometries) throws IOException {

        is.skipBytes(4 * 8); // skip bounding box
        int numParts = is.readLittleEndianInt();
        int numPoints = is.readLittleEndianInt();
        readLineStrings(is, numParts, numPoints, geometries);
        
        return 4 * 8 + 4 + 4 + numParts * 4 + numPoints * 2 * 8;
    }

    private int readPolygon(MixedEndianDataInputStream is, 
            ArrayList<Geometry> geometries) throws IOException {

        is.skipBytes(4 * 8); // skip bounding box

        final int numParts = is.readLittleEndianInt();
        final int numPoints = is.readLittleEndianInt();

        ArrayList<Geometry> lineStrings = new ArrayList<>();
        readLineStrings(is, numParts, numPoints, lineStrings);
        Polygonizer polygonizer = new Polygonizer();
        polygonizer.add(lineStrings);
        Collection polys = polygonizer.getPolygons();
        geometries.addAll(polys);
        
        return 4 * 8 + 4 + 4 + numParts * 4 + numPoints * 2 * 8;
    }

    /**
     * Reads the number of records from the shx file.
     *
     * @param shapeURL The URL of the data shape file.
     * @return The number of records or -1 if the shx file cannot be found.
     */
    private int readRecordCountFromSHXFile(java.net.URL shapeURL) {

        MixedEndianDataInputStream is = null;
        try {
            java.net.URL shxURL = findSHXURL(shapeURL);
            if (shxURL == null) {
                return -1;
            }
            BufferedInputStream bis = new BufferedInputStream(shxURL.openStream());
            is = new MixedEndianDataInputStream(bis);
            is.skipBytes(24);
            final int fileLength = is.readInt() * 2;
            final int recordsCount = (fileLength - 100) / 8;
            return recordsCount;
        } catch (java.io.IOException e) {
            return -1;
        } finally {
            if (is != null) {
                try {
                    is.close();
                } catch (java.io.IOException e) {
                }
            }
        }
    }

    private int[] readSHXFile(java.net.URL shapeURL) {

        MixedEndianDataInputStream is = null;
        try {
            java.net.URL shxURL = findSHXURL(shapeURL);
            if (shxURL == null) {
                return null;
            }
            BufferedInputStream bis = new BufferedInputStream(shxURL.openStream());
            is = new MixedEndianDataInputStream(bis);
            is.skipBytes(24);
            final int fileLength = is.readInt() * 2;
            final int recordsCount = (fileLength - 100) / 8;
            int[] offsets = new int[recordsCount];
            is.skipBytes(72);
            int[] recOffsets = new int[recordsCount];

            for (int i = 0; i < recordsCount; i++) {
                recOffsets[i] = is.readInt() * 2;
                // skip length
                is.readInt();
            }

            return recOffsets;
        } catch (java.io.IOException e) {
            return null;
        } finally {
            if (is != null) {
                try {
                    is.close();
                } catch (java.io.IOException e) {
                }
            }
        }

    }
}
