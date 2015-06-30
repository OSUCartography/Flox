/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.oregonstate.cartography.flox.model;

import java.awt.geom.Rectangle2D;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.StringTokenizer;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.xml.bind.annotation.adapters.XmlAdapter;

/**
 *
 * @author danielstephen
 */
public class RectangleSerializer extends XmlAdapter<String, Rectangle2D>{

    @Override
    public Rectangle2D unmarshal(String s) throws Exception {
        try {
            String l;
            Rectangle2D canvas = null;
            BufferedReader reader = new BufferedReader(new StringReader(s));
            while ((l = reader.readLine()) != null) {
                StringTokenizer tokenizer = new StringTokenizer(l, ",\t");
                double x = Double.parseDouble(tokenizer.nextToken());
                double y = Double.parseDouble(tokenizer.nextToken());
                double w = Double.parseDouble(tokenizer.nextToken());
                double h = Double.parseDouble(tokenizer.nextToken());
                canvas = new Rectangle2D.Double (x, y, w, h);
            }
            return (Rectangle2D) canvas;
        } catch (IOException ex) {
            Logger.getLogger(GraphSerializer.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
        
    }

    @Override
    public String marshal(Rectangle2D canvas) throws Exception {
        
        // Create a string that has everything needed to construct the canvas
        // This would be the origin point of the rectangle, the width, and height
        StringBuilder str = new StringBuilder();
        str.append(canvas.getX());
        str.append(",");
        str.append(canvas.getY());
        str.append(",");
        str.append(canvas.getWidth());
        str.append(",");
        str.append(canvas.getHeight());
        
        return str.toString();
    }
    
}
