package edu.oregonstate.cartography.simplefeature;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.operation.polygonize.Polygonizer;
import com.vividsolutions.jts.operation.union.UnaryUnionOp;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.Callable;

/**
 *
 * @author dbaston
 */
public class PolygonBuilder implements Callable<Geometry> {
    private final Polygonizer polygen;
    
    public PolygonBuilder(Collection<Geometry> inputs) {
        this.polygen = new Polygonizer();
        
        // Inputs that are incorrectly noded or may have duplicate geometry
        // can cause the polygonizer to fail.  Since polygonization is often
        // at the end of a long process, the price of UnaryUnionOp.Union()
        // is well worth it.
        //this.polygen.add(inputs);
        this.polygen.add(UnaryUnionOp.union(inputs));
    }
    
    @Override
    public Geometry call() {
        return this.getResult();
    }
    
    /** Given a collection of JTS Geometry objects, returns the subset of
     * Geometries that have at least one vertex that is not shared by
     * any other Geometry in the input set.
     * @param polys
     * @return 
     */
    public static Collection<Geometry> getNonRedundantGeom(Collection<Geometry> polys) {
        HashMap<Coordinate, Integer> vertexCounts = new HashMap<>();
        
        for (Geometry p : polys) {
            for (Coordinate c : p.getCoordinates()) {
                if (!vertexCounts.containsKey(c)) {
                    vertexCounts.put(c, 0);
                }
                vertexCounts.put(c, vertexCounts.get(c) + 1);
            }
        }
        
        Set<Geometry> exteriorPolys = new HashSet<>();
        
        POLYLOOP:
        for (Geometry p : polys) {
            for (Coordinate c : p.getCoordinates()) {
                if (vertexCounts.get(c) == 1) {
                    exteriorPolys.add(p);
                    continue POLYLOOP;
                }
            }
        }
        
        return exteriorPolys;
    }
    
    /** Uses a JTS Polygonizer to identify all possible polygons from a set
     * of input linework, then uses an iterative algorithm to determine which
     * of the returned polygons represent holes.  The algorithm is:
     * 1.  Identify the outermost polygons in the returned set (those polygons
     *        that have at least one unique vertex).  These polygons cannot
     *        be holes.
     * 2.  Classify as a "hole" any polygon that has a linear intersection with
     *        a non-hole polygon.
     * 3.  Classify as "non-hole" any polygon that has a linear intersection
     *        with a hole polygon.
     * 4.  Repeat steps 2-3 until all polygons have been classified.
     * 
     * @return A single Geometry representing the union of all non-hole polygons.
     */
    public Geometry getResult() {
        Collection<Geometry> polys = polygen.getPolygons();
        
        Set<Geometry> exteriorPolys = new HashSet<>(getNonRedundantGeom(polys));
        Set<Geometry> unknownPolys  = new HashSet<>(polys);
        Set<Geometry> interiorPolys = new HashSet<>();
        unknownPolys.removeAll(exteriorPolys);
        
        Geometry mainPoly = UnaryUnionOp.union(exteriorPolys);
        Geometry holePoly = null;
        
        boolean changed = true;
        while (!unknownPolys.isEmpty() && changed) {
            interiorPolys.clear();
            exteriorPolys.clear();
            changed = false;
            
            for (Geometry p : unknownPolys) {
                if (p.intersects(mainPoly) && p.intersection(mainPoly).getDimension() > 0) {
                    changed = true;
                    interiorPolys.add(p);
                } else if (holePoly != null && p.intersects(holePoly) && p.intersection(holePoly).getDimension() > 0) {
                    changed = true;
                    exteriorPolys.add(p);
                }
            }

            unknownPolys.removeAll(exteriorPolys);
            unknownPolys.removeAll(interiorPolys);
            
            if (!exteriorPolys.isEmpty()) {
                exteriorPolys.add(mainPoly);
                mainPoly = UnaryUnionOp.union(exteriorPolys);
            }
    
            if (!interiorPolys.isEmpty()) {
                if (holePoly != null) {
                    interiorPolys.add(holePoly);
                }
                holePoly = UnaryUnionOp.union(interiorPolys);
            }
        }
        
        return mainPoly;
    }
    
    
}