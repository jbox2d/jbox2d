/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met: * Redistributions of source code must retain the
 * above copyright notice, this list of conditions and the following disclaimer. * Redistributions
 * in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 * * Neither the name of the <organization> nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 4:23:48 PM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.ListCellRenderer;
import javax.swing.SwingConstants;
import javax.swing.UIManager;
import javax.swing.border.EtchedBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.jbox2d.testbed.framework.TestbedModel.ListItem;

/**
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
public class TestbedMain extends JFrame {

  private TestPanel panel;
  private int currTestIndex;
  private SidePanel side;
  private TestbedModel model;

  public TestbedMain(TestbedModel argModel) {
    super("JBox2D Testbed");
    model = argModel;
    setLayout(new BorderLayout());
    panel = new TestPanel(model.getSettings());

    panel.addKeyListener(new KeyListener() {

      @Override
      public void keyTyped(KeyEvent e) {
      }

      @Override
      public void keyReleased(KeyEvent e) {
      }

      @Override
      public void keyPressed(KeyEvent e) {
        if (e.getKeyChar() == '[') {
          lastTest();
        } else if (e.getKeyChar() == ']') {
          nextTest();
        }
      }
    });

    add(panel, "Center");
    side = new SidePanel(argModel);
    side.setMain(this);
    add(new JScrollPane(side), "East");
    pack();

    setVisible(true);
    setDefaultCloseOperation(3);
    currTestIndex = 0;
    side.tests.setSelectedIndex(0);
    side.actionPerformed(null);
    // panel.changeTest(argModel.getTestAt(0));
  }

  public void nextTest() {
    int index = currTestIndex + 1;
    index %= model.getTestsSize();

    while (!model.isTestAt(index) && index < model.getTestsSize() - 1) {
      index++;
    }
    if (model.isTestAt(index)) {
      side.tests.setSelectedIndex(index);
    }
  }

  public void saveTest() {
    panel.getCurrTest().save();
  }

  public void loadTest() {
    panel.getCurrTest().load();
  }

  public void lastTest() {
    int index = currTestIndex - 1;
    index = (index < 0) ? index + model.getTestsSize() : index;

    while (!model.isTestAt(index) && index > 0) {
      index--;
    }
    if (model.isTestAt(index)) {
      side.tests.setSelectedIndex(index);
    }
  }

  public void resetTest() {
    panel.resetTest();
  }

  public void testChanged(int argIndex) {
    if (argIndex == -1) {
      return;
    }
    while (!model.isTestAt(argIndex)) {
      if (argIndex + 1 < model.getTestsSize()) {
        argIndex++;
      }else{
        return;
      }
    }
    side.tests.setSelectedIndex(argIndex);
    
    currTestIndex = argIndex;
    TestbedTest test = model.getTestAt(argIndex);
    if (panel.getCurrTest() != test) {
      panel.changeTest(model.getTestAt(argIndex));
      side.saveButton.setEnabled(test.isSaveLoadEnabled());
      side.loadButton.setEnabled(test.isSaveLoadEnabled());
    }
    panel.grabFocus();
  }

  public static void main(String[] args) {
    try {
      UIManager.setLookAndFeel("com.sun.java.swing.plaf.nimbus.NimbusLookAndFeel");
    } catch (Exception e) {
    }

    TestbedModel model = new TestbedModel();
    TestList.populateModel(model);
    new TestbedMain(model);
  }
}

/**
 * quick hackup of a side panel
 * 
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
class SidePanel extends JPanel implements ChangeListener, ActionListener {

  final TestbedModel model;
  TestbedMain main;

  private JSlider hz;
  private JLabel hzText;
  private JSlider pos;
  private JLabel posText;
  private JSlider vel;
  private JLabel velText;

  public JComboBox tests;

  private JButton pauseButton = new JButton("Pause");
  private JButton stepButton = new JButton("Step");
  private JButton resetButton = new JButton("Reset");
  private JButton quitButton = new JButton("Quit");

  public JButton saveButton = new JButton("Save");
  public JButton loadButton = new JButton("Load");

  static String[] checkboxLabels = { "Warm Starting", "Continuous Collision", "Draw Shapes",
      "Draw Joints", "Draw AABBs", "Draw Pairs", "Draw Contact Points", "Draw Contact Normals",
      "Draw Contact Forces", "Draw Friction Forces", "Draw Center of Mass", "Draw Stats",
      "Draw Help", "Draw Dynamic Tree" };

  public SidePanel(TestbedModel argModel) {
    model = argModel;
    initComponents();
    addListeners();
  }

  public void setMain(TestbedMain argMain) {
    main = argMain;
  }

  public void initComponents() {
    setLayout(new BorderLayout());
    setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

    TestbedSettings settings = model.getSettings();

    JPanel top = new JPanel();
    top.setLayout(new GridLayout(0, 1));
    top.setBorder(BorderFactory.createCompoundBorder(new EtchedBorder(EtchedBorder.LOWERED),
        BorderFactory.createEmptyBorder(10, 10, 10, 10)));
    tests = new JComboBox(model.getComboModel());
    tests.setMaximumRowCount(30);
    tests.setMaximumSize(new Dimension(250, 20));
    tests.addActionListener(this);
    tests.setRenderer(new ListCellRenderer() {
      JLabel categoryLabel = null;
      JLabel testLabel = null;

      @Override
      public Component getListCellRendererComponent(JList list, Object value, int index,
          boolean isSelected, boolean cellHasFocus) {
        ListItem item = (ListItem) value;

        if (item.isCategory()) {
          if(categoryLabel == null){
            categoryLabel = new JLabel();
            categoryLabel.setOpaque(true);
            categoryLabel.setBackground(new Color(.5f, .5f, .6f));
            categoryLabel.setForeground(Color.white);
            categoryLabel.setHorizontalAlignment(SwingConstants.CENTER);
            categoryLabel.setBorder(BorderFactory.createEmptyBorder(1, 1, 1, 1));
          }
          categoryLabel.setText(item.category);
          return categoryLabel;
        } else {
          if(testLabel == null){
            testLabel = new JLabel();
            testLabel.setBorder(BorderFactory.createEmptyBorder(0, 5, 1, 0));
          }
         
          testLabel.setText(item.test.getTestName());

          if (isSelected) {
            testLabel.setBackground(list.getSelectionBackground());
            testLabel.setForeground(list.getSelectionForeground());
          } else {
            testLabel.setBackground(list.getBackground());
            testLabel.setForeground(list.getForeground());
          }
          return testLabel;
        }
      }
    });
    JPanel testsp = new JPanel();
    testsp.setLayout(new GridLayout(1, 2));
    testsp.add(new JLabel("Choose a test:"));
    testsp.add(tests);

    top.add(tests);
    hz = new JSlider(1, 400, (int) settings.hz);
    hz.setMaximumSize(new Dimension(200, 20));
    hz.addChangeListener(this);
    hzText = new JLabel("Hz: " + (int) settings.hz);
    top.add(hzText);
    top.add(hz);

    pos = new JSlider(0, 100, (int) settings.positionIterations);
    pos.setMaximumSize(new Dimension(200, 20));
    pos.addChangeListener(this);
    posText = new JLabel("Pos iters: " + (int) settings.positionIterations);
    top.add(posText);
    top.add(pos);

    vel = new JSlider(1, 100, (int) settings.velocityIterations);
    vel.setMaximumSize(new Dimension(200, 20));
    vel.addChangeListener(this);
    velText = new JLabel("Vel iters: " + (int) settings.velocityIterations);
    top.add(velText);
    top.add(vel);

    add(top, "North");

    JPanel middle = new JPanel();
    middle.setLayout(new GridLayout(0, 1));
    middle.setBorder(BorderFactory.createCompoundBorder(new EtchedBorder(EtchedBorder.LOWERED),
        BorderFactory.createEmptyBorder(5, 10, 5, 10)));
    for (int i = 0; i < checkboxLabels.length; i++) {
      String s = checkboxLabels[i];

      JCheckBox box = new JCheckBox(s);
      box.putClientProperty("index", i);
      // "Warm Starting", "Continuous Collision",
      // "Draw Shapes", "Draw Joints", "Draw AABBs",
      // "Draw Pairs", "Draw Contact Points",
      // "Draw Contact Normals", "Draw Contact Forces",
      // "Draw Friction Forces", "Draw Center of Mass",
      // "Draw Stats"
      boolean tf = false;
      switch (i) {
        case 0:
          tf = settings.enableWarmStarting;
          break;
        case 1:
          tf = settings.enableContinuous;
          break;
        case 2:
          tf = settings.drawShapes;
          break;
        case 3:
          tf = settings.drawJoints;
          break;
        case 4:
          tf = settings.drawAABBs;
          break;
        case 5:
          tf = settings.drawPairs;
          break;
        case 6:
          tf = settings.drawContactPoints;
          break;
        case 7:
          tf = settings.drawContactNormals;
          break;
        case 8:
          tf = settings.drawContactForces;
          break;
        case 9:
          tf = settings.drawFrictionForces;
          break;
        case 10:
          tf = settings.drawCOMs;
          break;
        case 11:
          tf = settings.drawStats;
          break;
        case 12:
          tf = settings.drawHelp;
          break;
        case 13:
          tf = settings.drawDynamicTree;
          break;
        default:
          System.out.println("oh no: " + i);
          tf = false;
      }
      box.setSelected(tf);
      box.addChangeListener(this);
      middle.add(box);
    }
    add(middle, "Center");

    pauseButton.setAlignmentX(CENTER_ALIGNMENT);
    stepButton.setAlignmentX(CENTER_ALIGNMENT);
    resetButton.setAlignmentX(CENTER_ALIGNMENT);
    saveButton.setAlignmentX(CENTER_ALIGNMENT);
    loadButton.setAlignmentX(CENTER_ALIGNMENT);
    quitButton.setAlignmentX(CENTER_ALIGNMENT);
    
    Box buttonGroups = Box.createHorizontalBox();
    JPanel buttons1 = new JPanel();
    buttons1.setLayout(new GridLayout(0, 1));
    buttons1.add(pauseButton);
    buttons1.add(stepButton);
    
    JPanel buttons2 = new JPanel();
    buttons2.setLayout(new GridLayout(0, 1));
    buttons2.add(resetButton);
    buttons2.add(saveButton);
    buttons2.add(loadButton);
    
    JPanel buttons3 = new JPanel();
    buttons3.setLayout(new GridLayout(0, 1));
    buttons3.add(quitButton);
    
    buttonGroups.add(buttons1);
    buttonGroups.add(buttons2);
    buttonGroups.add(buttons3);
    
    add(buttonGroups, "South");
  }

  public void addListeners() {
    pauseButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        if (model.getSettings().pause) {
          model.getSettings().pause = false;
          pauseButton.setText("Pause");
        } else {
          model.getSettings().pause = true;
          pauseButton.setText("Resume");
        }
      }
    });

    stepButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        model.getSettings().singleStep = true;
        if (!model.getSettings().pause) {
          model.getSettings().pause = true;
          pauseButton.setText("Resume");
        }
      }
    });

    resetButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        main.resetTest();
      }
    });

    quitButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        System.exit(0);
      }
    });

    saveButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        main.saveTest();
      }
    });

    loadButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        main.loadTest();
      }
    });
  }

  /**
   * @see javax.swing.event.ChangeListener#stateChanged(javax.swing.event.ChangeEvent)
   */
  public void stateChanged(ChangeEvent e) {
    if (e.getSource() instanceof JCheckBox) {
      JCheckBox box = (JCheckBox) e.getSource();

      TestbedSettings settings = model.getSettings();

      int i = (Integer) box.getClientProperty("index");
      boolean tf = box.isSelected();
      switch (i) {
        case 0:
          settings.enableWarmStarting = tf;
          break;
        case 1:
          settings.enableContinuous = tf;
          break;
        case 2:
          settings.drawShapes = tf;
          break;
        case 3:
          settings.drawJoints = tf;
          break;
        case 4:
          settings.drawAABBs = tf;
          break;
        case 5:
          settings.drawPairs = tf;
          break;
        case 6:
          settings.drawContactPoints = tf;
          break;
        case 7:
          settings.drawContactNormals = tf;
          break;
        case 8:
          settings.drawContactForces = tf;
          break;
        case 9:
          settings.drawFrictionForces = tf;
          break;
        case 10:
          settings.drawCOMs = tf;
          break;
        case 11:
          settings.drawStats = tf;
          break;
        case 12:
          settings.drawHelp = tf;
          break;
        case 13:
          settings.drawDynamicTree = tf;
          break;
        default:
          System.out.println("oh no: " + i);
      }
    } else if (e.getSource() instanceof JSlider) {
      JSlider s = (JSlider) e.getSource();

      TestbedSettings settings = model.getSettings();

      if (s == hz) {
        settings.hz = s.getValue();
        hzText.setText("Hz: " + s.getValue());
      } else if (s == pos) {
        settings.positionIterations = s.getValue();
        posText.setText("Pos iters: " + s.getValue());
      } else if (s == vel) {
        settings.velocityIterations = s.getValue();
        velText.setText("Vel iters: " + s.getValue());
      }
    }
  }

  public void actionPerformed(ActionEvent e) {
    if(e != null){
      String action = e.getActionCommand();
      System.out.println(action != null ? action : "null");
    }
    main.testChanged(tests.getSelectedIndex());
  }
}
