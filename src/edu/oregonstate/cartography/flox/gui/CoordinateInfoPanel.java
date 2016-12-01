/*
 * CoordinateInfoPanel.java
 *
 * Created on July 1, 2005, 3:05 PM
 */

package edu.oregonstate.cartography.flox.gui;

import edu.oregonstate.cartography.flox.model.Model;
import edu.oregonstate.cartography.map.MapToolMouseMotionListener;
import edu.oregonstate.cartography.map.MeasureToolListener;
import edu.oregonstate.cartography.simplefeature.AbstractSimpleFeatureMapComponent;
import java.text.DecimalFormat;



/**
 *
 * @author  Bernhard Jenny, Institute of Cartography, ETH Zurich.
 */
public class CoordinateInfoPanel extends javax.swing.JPanel
        implements MeasureToolListener, 
        MapToolMouseMotionListener {
    
    private static final DecimalFormat angleFormatter =
            new DecimalFormat("###,##0.0");
    
    private Model model;
    
    /** Creates new form CoordinateInfoPanel */
    public CoordinateInfoPanel() {
        initComponents();
    }
    
    public void setModel(Model model) {
        this.model = model;
    }
    
    public void setCoordinatesVisible(boolean visible) {
        xCoordLabel.setVisible(visible);
        yCoordLabel.setVisible(visible);
        xTextLabel.setVisible(visible);
        yTextLabel.setVisible(visible);
    }
    
    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {
        java.awt.GridBagConstraints gridBagConstraints;

        xCoordLabel = new javax.swing.JLabel();
        yCoordLabel = new javax.swing.JLabel();
        xTextLabel = new javax.swing.JLabel();
        yTextLabel = new javax.swing.JLabel();
        distTextLabel = new javax.swing.JLabel();
        distLabel = new javax.swing.JLabel();
        angleTextLabel = new javax.swing.JLabel();
        angleLabel = new javax.swing.JLabel();

        setLayout(new java.awt.GridBagLayout());

        xCoordLabel.setFont(new java.awt.Font("Lucida Grande", 0, 10)); // NOI18N
        xCoordLabel.setHorizontalAlignment(javax.swing.SwingConstants.RIGHT);
        xCoordLabel.setPreferredSize(new java.awt.Dimension(80, 13));
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 0;
        add(xCoordLabel, gridBagConstraints);

        yCoordLabel.setFont(new java.awt.Font("Lucida Grande", 0, 10)); // NOI18N
        yCoordLabel.setHorizontalAlignment(javax.swing.SwingConstants.RIGHT);
        yCoordLabel.setPreferredSize(new java.awt.Dimension(80, 13));
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 1;
        add(yCoordLabel, gridBagConstraints);

        xTextLabel.setFont(new java.awt.Font("Lucida Grande", 0, 10)); // NOI18N
        xTextLabel.setText("X : ");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        add(xTextLabel, gridBagConstraints);

        yTextLabel.setFont(new java.awt.Font("Lucida Grande", 0, 10)); // NOI18N
        yTextLabel.setText("Y : ");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        add(yTextLabel, gridBagConstraints);

        distTextLabel.setFont(new java.awt.Font("Lucida Grande", 0, 10)); // NOI18N
        distTextLabel.setText("Distance : ");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 15, 0, 0);
        add(distTextLabel, gridBagConstraints);

        distLabel.setFont(new java.awt.Font("Lucida Grande", 0, 10)); // NOI18N
        distLabel.setText("-");
        distLabel.setPreferredSize(new java.awt.Dimension(80, 13));
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 3;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.weightx = 1.0;
        add(distLabel, gridBagConstraints);

        angleTextLabel.setFont(new java.awt.Font("Lucida Grande", 0, 10)); // NOI18N
        angleTextLabel.setText("Angle : ");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 15, 0, 0);
        add(angleTextLabel, gridBagConstraints);

        angleLabel.setFont(new java.awt.Font("Lucida Grande", 0, 10)); // NOI18N
        angleLabel.setText("-");
        angleLabel.setPreferredSize(new java.awt.Dimension(80, 13));
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 3;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.weightx = 1.0;
        add(angleLabel, gridBagConstraints);
    }// </editor-fold>//GEN-END:initComponents
    
    @Override
    public void clearDistance() {
        this.distLabel.setText("-");
        this.angleLabel.setText("-");
    }
    
    @Override
    public void distanceChanged(double distance, double angle,
            AbstractSimpleFeatureMapComponent mapComponent) {
        
        // distance
        if (model != null) {
            distance *= model.getReferenceMapScale();
        }
        CoordinateFormatter coordFormatter = mapComponent.getCoordinateFormatter();
        this.distLabel.setText(coordFormatter.format(distance) + "px");
        
        // angle
        double azimuth = -Math.toDegrees(angle) + 90.;
        if (azimuth < 0)
            azimuth += 360;
        String angleStr = angleFormatter.format(azimuth);
        this.angleLabel.setText("<HTML>"+angleStr+"&#176</HTML>");
    }
    
    protected void updateCoordinates(java.awt.geom.Point2D.Double point,
            AbstractSimpleFeatureMapComponent mapComponent) {
        if (point != null) {
            CoordinateFormatter formatter = mapComponent.getCoordinateFormatter();
            String xStr = formatter.format(point.getX());
            String yStr = formatter.format(point.getY());
            this.xCoordLabel.setText(xStr);
            this.yCoordLabel.setText(yStr);
        } else {
            this.xCoordLabel.setText("-");
            this.yCoordLabel.setText("-");
        }
    }
    
    @Override
    public void mouseMoved(java.awt.geom.Point2D.Double point,
            AbstractSimpleFeatureMapComponent mapComponent) {
        this.updateCoordinates(point, mapComponent);
    }
    
    public void registerWithMapComponent(AbstractSimpleFeatureMapComponent mapComponent) {
        mapComponent.addMouseMotionListener(this);
    }

    @Override
    public void newDistance(double distance, double angle, AbstractSimpleFeatureMapComponent mapComponent) {
        this.distanceChanged(distance, angle, mapComponent);
    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JLabel angleLabel;
    private javax.swing.JLabel angleTextLabel;
    private javax.swing.JLabel distLabel;
    private javax.swing.JLabel distTextLabel;
    private javax.swing.JLabel xCoordLabel;
    private javax.swing.JLabel xTextLabel;
    private javax.swing.JLabel yCoordLabel;
    private javax.swing.JLabel yTextLabel;
    // End of variables declaration//GEN-END:variables
    
}
