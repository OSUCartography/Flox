package edu.oregonstate.cartography.flox.model;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.io.ParseException;
import com.vividsolutions.jts.io.WKTReader;
import com.vividsolutions.jts.io.WKTWriter;
import java.io.IOException;
import javax.xml.bind.annotation.adapters.XmlAdapter;

public class GeometrySerializer extends XmlAdapter<String, Geometry> {

    @Override
    public Geometry unmarshal(String s) throws IOException, ParseException {
        return new WKTReader().read(s);
    }

    @Override
    public String marshal(Geometry geometry) {
        return new WKTWriter().write(geometry);
    }

}
