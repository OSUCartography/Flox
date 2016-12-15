package edu.oregonstate.cartography.flox.model;

import java.util.Arrays;
import net.jafama.FastMath;

/**
 * Original JavaScript by Kevin Lindsey:
 * http://www.kevlindev.com/gui/math/intersection/index.htm
 *
 * Ported to Java by Bernhard Jenny, School of Science - Geospatial Science,
 * RMIT University, Melbourne
 */
public class Polynomial {

    private static final double SQRT3 = Math.sqrt(3d);
    private static final double TOLERANCE = 1e-6;
    private static final double[] EMPTY_ARRAY = new double[]{};

    /**
     * polynomial coefficients from lowest to highest
     */
    private double[] coefs;

    /**
     * Construct new Polynomial of arbitrary degree.
     *
     * @param coefs highest degree first, lowest degree last
     */
    public Polynomial(double ... coefs) {
        this.coefs = coefs;
        // invert order of ceofficients
        for (int i = 0; i < coefs.length / 2; i++) {
            double temp = coefs[i];
            coefs[i] = coefs[coefs.length - i - 1];
            coefs[coefs.length - i - 1] = temp;
        }
    }
    
    @Override
    public String toString(){
        StringBuilder sb = new StringBuilder();
        sb.append("Polynomial degree ").append(getDegree()).append(" [");
        for (double c : coefs) {
            sb.append(c).append(",");
        }
        sb.append("]");
        return sb.toString();
    }
    
    /**
     * Returns the polynomial's degree.
     *
     * @return degree
     */
    public int getDegree() {
        return coefs.length - 1;
    }

    /**
     * Evaluate the polynomial at x.
     *
     * @param x
     * @return y
     */
    public double eval(double x) {
        double result = 0;
        for (int i = coefs.length - 1; i >= 0; i--) {
            result = result * x + coefs[i];
        }
        return result;
    }

    /**
     * Returns a new Polynomial with derivative of this Polynomial.
     *
     * @return new Polynomial with one degree less
     */
    public Polynomial getDerivative() {
        double[] derCoefs = new double[coefs.length - 1];
        for (int i = 1; i < coefs.length; i++) {
            derCoefs[i - 1] = i * coefs[i];
        }
        return new Polynomial(derCoefs);
    }

    /**
     * Remove the last few coefficients if they are smaller than
     * Polynomial.TOLERANCE. This changes this.coefs.
     */
    private void simplify() {
        for (int i = getDegree(); i >= 0; i--) {
            if (Math.abs(coefs[i]) <= Polynomial.TOLERANCE) {
                // remove last element
                coefs = Arrays.copyOf(coefs, coefs.length - 1);
            } else {
                break;
            }
        }
    }

    private double[] getLinearRoot() {
        assert (getDegree() == 1);

        double a = coefs[1];
        if (a != 0) {
            return new double[]{(-coefs[0] / a)};
        }
        return EMPTY_ARRAY;
    }

    private double[] getQuadraticRoots() {
        assert (getDegree() == 2);

        double a = coefs[2];
        double b = coefs[1] / a;
        double c = coefs[0] / a;
        double d = b * b - 4 * c;
        if (d > 0) {
            double e = Math.sqrt(d);
            return new double[]{0.5 * (-b + e), 0.5 * (-b - e)};
        } else if (d == 0) {
            return new double[]{0.5 * -b};
        }
        return EMPTY_ARRAY;
    }
    
    private double[] getCubicRoots() {
        assert (getDegree() == 3);

        double c3 = coefs[3];
        double c2 = coefs[2] / c3;
        double c1 = coefs[1] / c3;
        double c0 = coefs[0] / c3;
        double a = (3 * c1 - c2 * c2) / 3;
        double b = (2 * c2 * c2 * c2 - 9 * c1 * c2 + 27 * c0) / 27;
        double offset = c2 / 3;
        double discrim = b * b / 4 + a * a * a / 27;
        double halfB = b / 2;
        
        if ( Math.abs(discrim) <= Polynomial.TOLERANCE ) {
            discrim = 0;
        }
        
        if (discrim > 0) {
            double e = Math.sqrt(discrim);
            double root = FastMath.cbrt(-halfB + e) + FastMath.cbrt(-halfB - e);
            return new double[]{root - offset};
        } else if (discrim < 0) {
            double distance = Math.sqrt(-a / 3);
            double angle = FastMath.atan2(Math.sqrt(-discrim), -halfB) / 3;
            double cos = FastMath.cos(angle);
            double sin = FastMath.sin(angle);
            double r1 = 2 * distance * cos - offset;
            double r2 = -distance * (cos + SQRT3 * sin) - offset;
            double r3 = -distance * (cos - SQRT3 * sin) - offset;
            return new double[]{r1, r2, r3};
        } else {
            double tmp = FastMath.cbrt(halfB);
            return new double[]{2 * tmp - offset, -tmp - offset};
        }
    }

    private double[] getQuarticRoots() {
        assert (getDegree() == 4);

        double c4 = coefs[4];
        double c3 = coefs[3] / c4;
        double c2 = coefs[2] / c4;
        double c1 = coefs[1] / c4;
        double c0 = coefs[0] / c4;
        double[] resolveRoots = new Polynomial(1, -c2, c3 * c1 - 4 * c0, -c3 * c3 * c0 + 4 * c2 * c0 - c1 * c1).getCubicRoots();
        double y = resolveRoots[0];
        double discrim = c3 * c3 / 4 - c2 + y;
        if (Math.abs(discrim) <= Polynomial.TOLERANCE) {
            discrim = 0;
        }
        if (discrim > 0) {
            double e = Math.sqrt(discrim);
            double t1 = 3 * c3 * c3 / 4 - e * e - 2 * c2;
            double t2 = (4 * c3 * c2 - 8 * c1 - c3 * c3 * c3) / (4 * e);
            double plus = t1 + t2;
            double minus = t1 - t2;
            if (Math.abs(plus) <= Polynomial.TOLERANCE) {
                plus = 0;
            }
            if (Math.abs(minus) <= Polynomial.TOLERANCE) {
                minus = 0;
            }
            int n = 0;
            if (plus >= 0) {
                n += 2;
            }
            if (minus >= 0) {
                n += 2;
            }
            if (n == 0) {
                return EMPTY_ARRAY;
            }
            double[] results = new double[n];
            int i = 0;
            if (plus >= 0) {
                double f = Math.sqrt(plus);
                results[0] = (-c3 / 4 + (e + f) / 2);
                results[1] = (-c3 / 4 + (e - f) / 2);
                i = 2;
            }
            if (minus >= 0) {
                double f = Math.sqrt(minus);
                results[i] = (-c3 / 4 + (f - e) / 2);
                results[i + 1] = (-c3 / 4 - (f + e) / 2);
            }
            return results;
        } else if (discrim < 0) {
            return EMPTY_ARRAY;
        } else {
            double t2 = y * y - 4 * c0;
            if (t2 >= -Polynomial.TOLERANCE) {
                if (t2 < 0) {
                    t2 = 0;
                }
                t2 = 2 * Math.sqrt(t2);
                double t1 = 3 * c3 * c3 / 4 - 2 * c2;
                boolean case1 = t1 + t2 >= Polynomial.TOLERANCE;
                boolean case2 = t1 - t2 >= Polynomial.TOLERANCE;
                int n = 0;
                if (case1) {
                    n += 2;
                }
                if (case2) {
                    n += 2;
                }
                if (n == 0) {
                    return EMPTY_ARRAY;
                }
                double[] results = new double[n];
                int i = 0;
                if (case1) {
                    double d = Math.sqrt(t1 + t2);
                    results[0] = (-c3 / 4 + d / 2);
                    results[1] = (-c3 / 4 - d / 2);
                    i = 2;
                }
                if (case2) {
                    double d = Math.sqrt(t1 - t2);
                    results[i] = (-c3 / 4 + d / 2);
                    results[i + 1] = (-c3 / 4 - d / 2);
                }
                return results;
            }
        }
        return EMPTY_ARRAY;
    }

    /**
     * Returns the real roots of this Polynomial.
     * 
     * @return an array with 0 or more roots.
     */
    public double[] getRoots() {
        simplify();
        switch (getDegree()) {
            case 1:
                return getLinearRoot();
            case 2:
                return getQuadraticRoots();
            case 3:
                return getCubicRoots();
            case 4:
                return getQuarticRoots();
        }
        return EMPTY_ARRAY;
    }

}
