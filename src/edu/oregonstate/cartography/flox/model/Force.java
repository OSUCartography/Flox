package edu.oregonstate.cartography.flox.model;

/**
 *
 * @author Bernhard Jenny, Cartography and Geovisualization Group, Oregon State
 * University
 */
public class Force {

    public double fx;
    public double fy;

    public Force() {
        fx = fy = 0;
    }

    public Force(double fx, double fy) {
        this.fx = fx;
        this.fy = fy;
    }

    public double length() {
        return Math.sqrt(fx * fx + fy * fy);
    }

    public void scale(double scale) {
        fx *= scale;
        fy *= scale;
    }

    public void normalize() {
        double l = Math.sqrt(fx * fx + fy * fy);
        fx /= l;
        fy /= l;
    }

    public void add(Force f) {
        fx += f.fx;
        fy += f.fy;
    }

    public static Force add(Force f1, Force f2) {
        return new Force(f1.fx + f2.fx, f1.fy + f2.fy);
    }

}
