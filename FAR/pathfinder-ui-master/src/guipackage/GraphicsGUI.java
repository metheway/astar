package guipackage;

import graphicspackage.*;
import adtpackage.*;
import generalpackage.*;
import apackage.*;
import farpackage.*;
/*
 * GraphicsGUI.java
 *
 * Created on Apr 8, 2013
 */

/**
 *
 * @author Zbyněk Stara
 */
public class GraphicsGUI extends GUI {
    private GraphicsPanel gp;

    protected void refreshField() {
        gp = new GraphicsPanel(numElements+2, RepaintState.REPAINT_OBSTACLES,  field); // numElements is withou boundaries, we want them too
        gp.setPreferredSize(new java.awt.Dimension(gp.getDimension(), gp.getDimension()));

        scrollPane.setViewportView(gp);
    }

    protected void refreshPathEndpoints() {
        gp.redrawPanel(numElements+2, RepaintState.REPAINT_GOALS, field);
    }

    protected void refreshPaths() {
        gp.redrawPanel(numElements+2, RepaintState.REPAINT_PATHS, field, choosePathfinder(), choosePathfinder().FAILURE_CRITERION);
    }

    protected void refreshAgents() {
        gp.redrawPanel(numElements+2, RepaintState.REPAINT_AGENTS, field, choosePathfinder(), stepCounter);
    }

    /** Creates new form GraphicsGUI */
    public GraphicsGUI() {
        initComponents();
    }

    public javax.swing.JDialog getInfoDialog() {
        return infoDialog;
    }

    public javax.swing.JTextField getInfoStepTF() {
        return infoStepTF;
    }

    public javax.swing.JTextField getInfoSuccessTF() {
        return infoSuccessTF;
    }

    public javax.swing.JTextField getInfoTimeTF() {
        return infoTimeTF;
    }

    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        infoDialog = new javax.swing.JDialog();
        jLabel6 = new javax.swing.JLabel();
        infoStepTF = new javax.swing.JTextField();
        jLabel7 = new javax.swing.JLabel();
        infoTimeTF = new javax.swing.JTextField();
        jLabel8 = new javax.swing.JLabel();
        jLabel9 = new javax.swing.JLabel();
        infoSuccessTF = new javax.swing.JTextField();
        fieldTF = new javax.swing.JTextField();
        generateFieldButton = new javax.swing.JButton();
        agentsTF = new javax.swing.JTextField();
        generateAgentsButton = new javax.swing.JButton();
        scrollPane = new javax.swing.JScrollPane();
        panel = new javax.swing.JPanel();
        pathfinderComboBox = new javax.swing.JComboBox();
        applyPathfinderButton = new javax.swing.JButton();
        stepTF = new javax.swing.JTextField();
        minusOneButton = new javax.swing.JButton();
        plusButton = new javax.swing.JButton();
        zeroButton = new javax.swing.JButton();
        jLabel1 = new javax.swing.JLabel();
        jLabel2 = new javax.swing.JLabel();
        jLabel3 = new javax.swing.JLabel();
        jLabel4 = new javax.swing.JLabel();
        minusButton = new javax.swing.JButton();
        jLabel5 = new javax.swing.JLabel();
        failuresTF = new javax.swing.JTextField();
        menuBar = new javax.swing.JMenuBar();
        fileMenu = new javax.swing.JMenu();
        openMenuItem = new javax.swing.JMenuItem();
        saveMenuItem = new javax.swing.JMenuItem();
        saveAsMenuItem = new javax.swing.JMenuItem();
        exitMenuItem = new javax.swing.JMenuItem();
        editMenu = new javax.swing.JMenu();
        cutMenuItem = new javax.swing.JMenuItem();
        copyMenuItem = new javax.swing.JMenuItem();
        pasteMenuItem = new javax.swing.JMenuItem();
        deleteMenuItem = new javax.swing.JMenuItem();
        helpMenu = new javax.swing.JMenu();
        contentsMenuItem = new javax.swing.JMenuItem();
        aboutMenuItem = new javax.swing.JMenuItem();

        infoDialog.setDefaultCloseOperation(javax.swing.WindowConstants.DO_NOTHING_ON_CLOSE);
        infoDialog.setTitle("Information");
        infoDialog.setModalityType(java.awt.Dialog.ModalityType.APPLICATION_MODAL);
        infoDialog.setPreferredSize(new java.awt.Dimension(400, 190));
        infoDialog.setSize(new java.awt.Dimension(400, 190));

        jLabel6.setText("Finding agent reservations, please wait...");

        infoStepTF.setFocusable(false);

        jLabel7.setText("Current simulation step:");

        infoTimeTF.setFocusable(false);

        jLabel8.setText("Time elapsed:");

        jLabel9.setText("Simulation successful for:");

        infoSuccessTF.setFocusable(false);

        org.jdesktop.layout.GroupLayout infoDialogLayout = new org.jdesktop.layout.GroupLayout(infoDialog.getContentPane());
        infoDialog.getContentPane().setLayout(infoDialogLayout);
        infoDialogLayout.setHorizontalGroup(
            infoDialogLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(infoDialogLayout.createSequentialGroup()
                .addContainerGap()
                .add(infoDialogLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(infoDialogLayout.createSequentialGroup()
                        .add(infoDialogLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                            .add(infoDialogLayout.createSequentialGroup()
                                .add(jLabel7)
                                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                                .add(infoStepTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 40, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                            .add(infoDialogLayout.createSequentialGroup()
                                .add(jLabel9)
                                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                                .add(infoSuccessTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 40, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)))
                        .addContainerGap(166, Short.MAX_VALUE))
                    .add(infoDialogLayout.createSequentialGroup()
                        .add(jLabel8)
                        .add(16, 16, 16)
                        .add(infoTimeTF, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, 40, Short.MAX_VALUE)
                        .add(237, 237, 237))
                    .add(infoDialogLayout.createSequentialGroup()
                        .add(jLabel6)
                        .addContainerGap(123, Short.MAX_VALUE))))
        );
        infoDialogLayout.setVerticalGroup(
            infoDialogLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(infoDialogLayout.createSequentialGroup()
                .addContainerGap()
                .add(jLabel6)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(infoDialogLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                    .add(jLabel7)
                    .add(infoStepTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(infoDialogLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                    .add(jLabel9)
                    .add(infoSuccessTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(infoDialogLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                    .add(infoTimeTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                    .add(jLabel8))
                .add(36, 36, 36))
        );

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setMinimumSize(new java.awt.Dimension(1153, 523));

        generateFieldButton.setText("Generate");
        generateFieldButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                generateFieldButtonMouseReleased(evt);
            }
        });

        generateAgentsButton.setText("Generate");
        generateAgentsButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                generateAgentsButtonMouseReleased(evt);
            }
        });

        scrollPane.setColumnHeaderView(null);
        scrollPane.setPreferredSize(new java.awt.Dimension(1113, 400));
        scrollPane.setRowHeaderView(null);
        scrollPane.getVerticalScrollBar().addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                scrollPaneScrollBarAdjustmentValueChanged(evt);
            }
        });
        scrollPane.getHorizontalScrollBar().addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                scrollPaneScrollBarAdjustmentValueChanged(evt);
            }
        });

        panel.setPreferredSize(new java.awt.Dimension(1113, 400));

        org.jdesktop.layout.GroupLayout panelLayout = new org.jdesktop.layout.GroupLayout(panel);
        panel.setLayout(panelLayout);
        panelLayout.setHorizontalGroup(
            panelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(0, 1113, Short.MAX_VALUE)
        );
        panelLayout.setVerticalGroup(
            panelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(0, 400, Short.MAX_VALUE)
        );

        scrollPane.setViewportView(panel);

        pathfinderComboBox.setModel(new javax.swing.DefaultComboBoxModel(new String[] { "A*", "LRA*", "CA*", "WHCA*", "FAR" }));
        pathfinderComboBox.addItemListener(new java.awt.event.ItemListener() {
            public void itemStateChanged(java.awt.event.ItemEvent evt) {
                pathfinderComboBoxItemStateChanged(evt);
            }
        });

        applyPathfinderButton.setText("Apply");
        applyPathfinderButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                applyPathfinderButtonMouseReleased(evt);
            }
        });

        stepTF.setEditable(false);
        stepTF.setAutoscrolls(false);
        stepTF.setFocusTraversalKeysEnabled(false);
        stepTF.setFocusable(false);
        stepTF.setRequestFocusEnabled(false);
        stepTF.setVerifyInputWhenFocusTarget(false);

        minusOneButton.setText("-1");
        minusOneButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                minusOneButtonMouseReleased(evt);
            }
        });

        plusButton.setText("+");
        plusButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                plusButtonMouseReleased(evt);
            }
        });

        zeroButton.setText("0");
        zeroButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                zeroButtonMouseReleased(evt);
            }
        });

        jLabel1.setText("Field:");

        jLabel2.setText("Agents:");

        jLabel3.setText("Pathfinder:");

        jLabel4.setText("Step:");

        minusButton.setText("-");
        minusButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                minusButtonMouseReleased(evt);
            }
        });

        jLabel5.setText("Failures:");

        failuresTF.setEditable(false);
        failuresTF.setAutoscrolls(false);
        failuresTF.setFocusTraversalKeysEnabled(false);
        failuresTF.setFocusable(false);
        failuresTF.setRequestFocusEnabled(false);
        failuresTF.setVerifyInputWhenFocusTarget(false);

        fileMenu.setText("File");

        openMenuItem.setText("Open");
        fileMenu.add(openMenuItem);

        saveMenuItem.setText("Save");
        fileMenu.add(saveMenuItem);

        saveAsMenuItem.setText("Save As ...");
        fileMenu.add(saveAsMenuItem);

        exitMenuItem.setText("Exit");
        exitMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                exitMenuItemActionPerformed(evt);
            }
        });
        fileMenu.add(exitMenuItem);

        menuBar.add(fileMenu);

        editMenu.setText("Edit");

        cutMenuItem.setText("Cut");
        editMenu.add(cutMenuItem);

        copyMenuItem.setText("Copy");
        editMenu.add(copyMenuItem);

        pasteMenuItem.setText("Paste");
        editMenu.add(pasteMenuItem);

        deleteMenuItem.setText("Delete");
        editMenu.add(deleteMenuItem);

        menuBar.add(editMenu);

        helpMenu.setText("Help");

        contentsMenuItem.setText("Contents");
        helpMenu.add(contentsMenuItem);

        aboutMenuItem.setText("About");
        helpMenu.add(aboutMenuItem);

        menuBar.add(helpMenu);

        setJMenuBar(menuBar);

        org.jdesktop.layout.GroupLayout layout = new org.jdesktop.layout.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(layout.createSequentialGroup()
                .add(layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(layout.createSequentialGroup()
                        .addContainerGap()
                        .add(scrollPane, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, 1113, Short.MAX_VALUE))
                    .add(layout.createSequentialGroup()
                        .add(20, 20, 20)
                        .add(jLabel1)
                        .add(10, 10, 10)
                        .add(fieldTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 38, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .add(9, 9, 9)
                        .add(generateFieldButton)
                        .add(9, 9, 9)
                        .add(jLabel2)
                        .add(10, 10, 10)
                        .add(agentsTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 38, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .add(9, 9, 9)
                        .add(generateAgentsButton)
                        .add(9, 9, 9)
                        .add(jLabel3)
                        .add(5, 5, 5)
                        .add(pathfinderComboBox, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .add(6, 6, 6)
                        .add(applyPathfinderButton)
                        .add(9, 9, 9)
                        .add(jLabel4)
                        .add(10, 10, 10)
                        .add(stepTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 38, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(minusOneButton, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 40, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(zeroButton, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 40, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(plusButton, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 40, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(minusButton, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 40, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(jLabel5)
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                        .add(failuresTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 38, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)))
                .addContainerGap())
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(layout.createSequentialGroup()
                .add(layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(layout.createSequentialGroup()
                        .add(20, 20, 20)
                        .add(layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                            .add(layout.createSequentialGroup()
                                .add(6, 6, 6)
                                .add(jLabel1))
                            .add(fieldTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                            .add(layout.createSequentialGroup()
                                .add(1, 1, 1)
                                .add(generateFieldButton))
                            .add(layout.createSequentialGroup()
                                .add(6, 6, 6)
                                .add(jLabel2))
                            .add(agentsTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                            .add(layout.createSequentialGroup()
                                .add(1, 1, 1)
                                .add(generateAgentsButton))
                            .add(layout.createSequentialGroup()
                                .add(6, 6, 6)
                                .add(jLabel3))
                            .add(layout.createSequentialGroup()
                                .add(2, 2, 2)
                                .add(pathfinderComboBox, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                            .add(layout.createSequentialGroup()
                                .add(1, 1, 1)
                                .add(applyPathfinderButton))
                            .add(layout.createSequentialGroup()
                                .add(6, 6, 6)
                                .add(jLabel4))
                            .add(stepTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)))
                    .add(layout.createSequentialGroup()
                        .add(21, 21, 21)
                        .add(layout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                            .add(minusOneButton)
                            .add(zeroButton)
                            .add(plusButton)
                            .add(minusButton)
                            .add(jLabel5)
                            .add(failuresTF, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))))
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(scrollPane, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, 399, Short.MAX_VALUE)
                .addContainerGap())
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void exitMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_exitMenuItemActionPerformed
        System.exit(0);
    }//GEN-LAST:event_exitMenuItemActionPerformed

    private void scrollPaneScrollBarAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
        scrollPane.repaint();
    }

    private void generateFieldButtonMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_generateFieldButtonMouseReleased
        numElements = Integer.parseInt(fieldTF.getText());

        isFieldSet = true;
        areAgentsSet = false;
        arePathsSet = false;

        stepCounter = -1;
        stepTF.setText(stepCounter + "");

        field = new Field(numElements+2, numElements+2);

        field.generateObstacles();
        field.assignFreeGroups();
        field.fillClosedGroups();

        refreshField();
    }//GEN-LAST:event_generateFieldButtonMouseReleased

    private void generateAgentsButtonMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_generateAgentsButtonMouseReleased
        if (isFieldSet) {
            stepCounter = -1;
            stepTF.setText(stepCounter + "");

            field.assignAgents(Integer.parseInt(agentsTF.getText()));
            areAgentsSet = true;

            refreshPathEndpoints();
        }
    }//GEN-LAST:event_generateAgentsButtonMouseReleased

    private void pathfinderComboBoxItemStateChanged(java.awt.event.ItemEvent evt) {//GEN-FIRST:event_pathfinderComboBoxItemStateChanged
        if (isFieldSet && areAgentsSet) {
            arePathsSet = false;

            stepCounter = -1;
            stepTF.setText(stepCounter + "");

            refreshPathEndpoints();
        }
    }//GEN-LAST:event_pathfinderComboBoxItemStateChanged

    private void applyPathfinderButtonMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_applyPathfinderButtonMouseReleased
        if (isFieldSet && areAgentsSet) {
            algorithmToUse = pathfinderComboBox.getSelectedIndex();

            initializePathfinder(this);

            choosePathfinder().findAgentPaths();
            arePathsSet = true;
            failuresTF.setText(choosePathfinder().getNumFailures() + "");

            stepCounter = -1;
            stepTF.setText(stepCounter + "");

            refreshPaths();
        }
    }//GEN-LAST:event_applyPathfinderButtonMouseReleased

    private void minusOneButtonMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_minusOneButtonMouseReleased
        if (isFieldSet && areAgentsSet) {
            arePathsSet = true;

            stepCounter = -1;
            stepTF.setText(stepCounter + "");

            refreshPaths();
        }
    }//GEN-LAST:event_minusOneButtonMouseReleased

    private void zeroButtonMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_zeroButtonMouseReleased
        if (isFieldSet && areAgentsSet && arePathsSet) {
            stepCounter = 0;
            stepTF.setText(stepCounter + "");

            refreshAgents();
        }
    }//GEN-LAST:event_zeroButtonMouseReleased

    private void plusButtonMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_plusButtonMouseReleased
        if (isFieldSet && areAgentsSet && arePathsSet && stepCounter < (choosePathfinder().FAILURE_CRITERION)) {
            stepCounter++;
            stepTF.setText(stepCounter + "");

            refreshAgents();
        }
    }//GEN-LAST:event_plusButtonMouseReleased

    private void minusButtonMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_minusButtonMouseReleased
        if (isFieldSet && areAgentsSet && arePathsSet && stepCounter > 0) {
            stepCounter--;
            stepTF.setText(stepCounter + "");

            refreshAgents();
        }
    }//GEN-LAST:event_minusButtonMouseReleased

    /**
    * @param args the command line arguments
    */
    public static void main(String args[]) {
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                new GraphicsGUI().setVisible(true);
            }
        });
    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JMenuItem aboutMenuItem;
    private javax.swing.JTextField agentsTF;
    private javax.swing.JButton applyPathfinderButton;
    private javax.swing.JMenuItem contentsMenuItem;
    private javax.swing.JMenuItem copyMenuItem;
    private javax.swing.JMenuItem cutMenuItem;
    private javax.swing.JMenuItem deleteMenuItem;
    private javax.swing.JMenu editMenu;
    private javax.swing.JMenuItem exitMenuItem;
    private javax.swing.JTextField failuresTF;
    private javax.swing.JTextField fieldTF;
    private javax.swing.JMenu fileMenu;
    private javax.swing.JButton generateAgentsButton;
    private javax.swing.JButton generateFieldButton;
    private javax.swing.JMenu helpMenu;
    public static javax.swing.JDialog infoDialog;
    public static javax.swing.JTextField infoStepTF;
    public static javax.swing.JTextField infoSuccessTF;
    public static javax.swing.JTextField infoTimeTF;
    private javax.swing.JLabel jLabel1;
    private javax.swing.JLabel jLabel2;
    private javax.swing.JLabel jLabel3;
    private javax.swing.JLabel jLabel4;
    private javax.swing.JLabel jLabel5;
    private javax.swing.JLabel jLabel6;
    private javax.swing.JLabel jLabel7;
    private javax.swing.JLabel jLabel8;
    private javax.swing.JLabel jLabel9;
    private javax.swing.JMenuBar menuBar;
    private javax.swing.JButton minusButton;
    private javax.swing.JButton minusOneButton;
    private javax.swing.JMenuItem openMenuItem;
    private javax.swing.JPanel panel;
    private javax.swing.JMenuItem pasteMenuItem;
    private javax.swing.JComboBox pathfinderComboBox;
    private javax.swing.JButton plusButton;
    private javax.swing.JMenuItem saveAsMenuItem;
    private javax.swing.JMenuItem saveMenuItem;
    private javax.swing.JScrollPane scrollPane;
    private javax.swing.JTextField stepTF;
    private javax.swing.JButton zeroButton;
    // End of variables declaration//GEN-END:variables

}
