/*
 * GeoGrid.java
 *
 * Created on August 26, 2015
 *
 */
package edu.oregonstate.cartography.flox.model;

import java.awt.geom.*;
import java.text.DecimalFormat;

/**
 * A georeferenced boolean grid.
 *
 * @author Bernhard Jenny.
 */
public final class BooleanGrid extends AbstractRaster {

    private boolean[][] grid;

    /**
     * Creates a new instance of GeoGrid
     *
     * @param cols Number of columns
     * @param rows Number of rows
     * @param cellSize Size of a grid cell.
     */
    public BooleanGrid(int cols, int rows, double cellSize) {
        this.initGrid(cols, rows, cellSize);
    }

    public BooleanGrid(int cols, int rows, double cellSize, boolean initialValue) {
        this.initGrid(cols, rows, cellSize);
        for (int r = 0; r < rows; ++r) {
            java.util.Arrays.fill(this.grid[r], initialValue);
        }
    }

    public BooleanGrid(boolean[][] grid, double cellSize) {
        if (grid == null
                || grid.length < 2
                || grid[0] == null
                || grid[0].length < 2
                || cellSize < 0) {
            throw new IllegalArgumentException();
        }

        this.grid = grid;
        this.cellSize = cellSize;
    }

    private void initGrid(int cols, int rows, double cellSize) {
        this.cellSize = cellSize;
        this.grid = new boolean[rows][cols];
    }

    public boolean hasSameExtensionAndResolution(BooleanGrid grid) {

        if (grid == null) {
            return false;
        }
        return getCols() == grid.getCols()
                && getRows() == grid.getRows()
                && getWest() == grid.getWest()
                && getNorth() == grid.getNorth()
                && getCellSize() == grid.getCellSize();

    }

    public Rectangle2D getBounds2D(double scale) {
        final double width = cellSize * (getCols() - 1);
        final double height = cellSize * (getRows() - 1);
        final double x = west;
        final double y = north - height;
        return new Rectangle2D.Double(x, y, width, height);
    }

    public final boolean getValue(int col, int row) {
        return grid[row][col];
    }

    public final boolean getNearestNeighbor(double x, double y) {
        // round to nearest neighbor
        int col = (int) ((x - this.west) / this.cellSize + 0.5);
        int row = (int) ((this.north - y) / this.cellSize + 0.5);
        if (col < 0 || col >= getCols() || row < 0 || row >= getRows()) {
            return false;
        }
        return grid[row][col];
    }

    /**
     * Change a value in the grid.
     * <B>Important: This will not generate a MapChange event!</B>
     *
     * @param value The new value
     * @param col The column of the value to change.
     * @param row The row of the value to change
     */
    public void setValue(boolean value, int col, int row) {
        grid[row][col] = value;
    }

    @Override
    public int getCols() {
        return (grid != null && grid.length > 0) ? grid[0].length : 0;
    }

    @Override
    public int getRows() {
        return grid != null ? grid.length : 0;
    }

    @Override
    public double getSouth() {
        return this.north - (getRows() - 1) * this.cellSize;
    }

    @Override
    public double getEast() {
        return this.west + (getCols() - 1) * this.cellSize;
    }

    public boolean[][] getGrid() {
        return grid;
    }

    /**
     * Converts an horizontal x coordinate to the next column id to the left of
     * x.
     *
     * @param x
     * @return The column (starting with 0) or -1 if x is not on the grid.
     */
    public int xToColumn(double x) {
        if (x < this.west || x > this.getEast()) {
            return -1;
        }
        return (int) ((x - this.west) / this.cellSize);
    }

    /**
     * Converts an vertical y coordinate to the next row id above y.
     *
     * @param y
     * @return The row (starting with 0) or -1 if y is not on the grid.
     */
    public int yToRow(double y) {
        if (y > this.north || y < this.getSouth()) {
            return -1;
        }
        return (int) ((this.north - y) / this.cellSize);
    }

    public long countTrueValues() {
        int rows = getRows();
        int cols = getCols();
        long counter = 0;
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                if (getValue(c, r)) {
                    counter++;
                }
            }
        }
        return counter;
    }

    @Override
    public String toString() {
        return this.toString("\n");
    }

    public String toString(String newLine) {
        DecimalFormat f = new DecimalFormat(cellSize < 1 ? "#,##0.0#####" : "#,##0.0");
        DecimalFormat intFormat = new DecimalFormat("#,###");
        StringBuilder sb = new StringBuilder();
        sb.append(super.toString());
        sb.append(newLine);
        sb.append("Dimension: ");
        sb.append(intFormat.format(this.getCols()));
        sb.append("x");
        sb.append(intFormat.format(this.getRows()));
        sb.append(newLine);
        sb.append("Cell size: ");
        sb.append(f.format(cellSize));
        sb.append(newLine);
        sb.append("West: ");
        sb.append(f.format(this.getWest()));
        sb.append(newLine);
        sb.append("North: ");
        sb.append(f.format(this.getNorth()));
        sb.append(newLine);

        long nbrValues = getCols() * getRows();
        long trueValues = countTrueValues();
        double perc = 100d * trueValues / nbrValues;
        sb.append("Number of true values: ");
        sb.append(trueValues);
        sb.append(" (");
        sb.append(intFormat.format(perc));
        sb.append("%)");
        sb.append(newLine);
        sb.append("Number of false values: ");
        sb.append(nbrValues - trueValues);
        sb.append(" (");
        sb.append(intFormat.format(100 - perc));
        sb.append("%)");
        return sb.toString();
    }

}
