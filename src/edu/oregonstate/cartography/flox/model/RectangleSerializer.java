package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.StringTokenizer;
import javax.xml.bind.annotation.adapters.XmlAdapter;

/**
 *
 * @author danielstephen
 */
public class RectangleSerializer extends XmlAdapter<String, Rectangle2D> {

    @Override
    public Rectangle2D unmarshal(String s) throws IOException {
        BufferedReader reader = new BufferedReader(new StringReader(s));
        String l = reader.readLine();
        StringTokenizer tokenizer = new StringTokenizer(l, ",\t");
        double x = Double.parseDouble(tokenizer.nextToken());
        double y = Double.parseDouble(tokenizer.nextToken());
        double w = Double.parseDouble(tokenizer.nextToken());
        double h = Double.parseDouble(tokenizer.nextToken());
        return new Rectangle2D.Double(x, y, w, h);
    }

    @Override
    public String marshal(Rectangle2D rect) {

        // Create a string that has everything needed to construct the rectangle
        // This would be the origin point of the rectangle, the width, and height
        StringBuilder str = new StringBuilder();
        str.append(rect.getX());
        str.append(",");
        str.append(rect.getY());
        str.append(",");
        str.append(rect.getWidth());
        str.append(",");
        str.append(rect.getHeight());

        return str.toString();
    }

}
