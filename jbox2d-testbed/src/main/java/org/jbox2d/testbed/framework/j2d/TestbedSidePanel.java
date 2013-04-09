/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.testbed.framework.j2d;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.ListCellRenderer;
import javax.swing.SwingConstants;
import javax.swing.border.EtchedBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.jbox2d.testbed.framework.TestbedController;
import org.jbox2d.testbed.framework.TestbedModel;
import org.jbox2d.testbed.framework.TestbedModel.ListItem;
import org.jbox2d.testbed.framework.TestbedSetting;
import org.jbox2d.testbed.framework.TestbedSetting.SettingType;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;


/**
 * The testbed side panel. Facilitates test and setting changes.
 * 
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
public class TestbedSidePanel extends JPanel implements ChangeListener, ActionListener {

  private static final String SETTING_TAG = "settings";
  private static final String LABEL_TAG = "label";

  final TestbedModel model;
  final TestbedController controller;

  public JComboBox tests;

  private JButton pauseButton = new JButton("Pause");
  private JButton stepButton = new JButton("Step");
  private JButton resetButton = new JButton("Reset");
  private JButton quitButton = new JButton("Quit");

  public JButton saveButton = new JButton("Save");
  public JButton loadButton = new JButton("Load");

  public TestbedSidePanel(TestbedModel argModel, TestbedController argController) {
    model = argModel;
    controller = argController;
    initComponents();
    addListeners();

    model.addTestChangeListener(new TestbedModel.TestChangedListener() {
      @Override
      public void testChanged(TestbedTest argTest, int argIndex) {
        tests.setSelectedIndex(argIndex);
        saveButton.setEnabled(argTest.isSaveLoadEnabled());
        loadButton.setEnabled(argTest.isSaveLoadEnabled());
      }
    });
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
      public Component getListCellRendererComponent(JList list, Object ovalue,
          int index, boolean isSelected, boolean cellHasFocus) {
        ListItem value = (ListItem) ovalue;
        if (value.isCategory()) {
          if (categoryLabel == null) {
            categoryLabel = new JLabel();
            categoryLabel.setOpaque(true);
            categoryLabel.setBackground(new Color(.5f, .5f, .6f));
            categoryLabel.setForeground(Color.white);
            categoryLabel.setHorizontalAlignment(SwingConstants.CENTER);
            categoryLabel.setBorder(BorderFactory.createEmptyBorder(1, 1, 1, 1));
          }
          categoryLabel.setText(value.category);
          return categoryLabel;
        } else {
          if (testLabel == null) {
            testLabel = new JLabel();
            testLabel.setBorder(BorderFactory.createEmptyBorder(0, 5, 1, 0));
          }

          testLabel.setText(value.test.getTestName());

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

    top.add(new JLabel("Choose a test:"));
    top.add(tests);

    addSettings(top, settings, SettingType.DRAWING);

    add(top, "North");

    JPanel middle = new JPanel();
    middle.setLayout(new GridLayout(0, 1));
    middle.setBorder(BorderFactory.createCompoundBorder(new EtchedBorder(EtchedBorder.LOWERED),
        BorderFactory.createEmptyBorder(5, 10, 5, 10)));

    addSettings(middle, settings, SettingType.ENGINE);

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
    buttons1.add(resetButton);

    JPanel buttons2 = new JPanel();
    buttons2.setLayout(new GridLayout(0, 1));
    buttons2.add(pauseButton);
    buttons2.add(stepButton);

    JPanel buttons3 = new JPanel();
    buttons3.setLayout(new GridLayout(0, 1));
    buttons3.add(saveButton);
    buttons3.add(loadButton);
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
        controller.resetTest();
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
        controller.saveTest();
      }
    });

    loadButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        controller.loadTest();
      }
    });
  }

  private void addSettings(JPanel argPanel, TestbedSettings argSettings, SettingType argIgnore) {
    for (TestbedSetting setting : argSettings.getSettings()) {
      if (setting.settingsType == argIgnore) {
        continue;
      }
      switch (setting.constraintType) {
        case RANGE:
          JLabel text = new JLabel(setting.name + ": " + setting.value);
          JSlider slider = new JSlider(setting.min, setting.max, setting.value);
          slider.setMaximumSize(new Dimension(200, 20));
          slider.addChangeListener(this);
          slider.setName(setting.name);
          slider.putClientProperty(SETTING_TAG, setting);
          slider.putClientProperty(LABEL_TAG, text);
          argPanel.add(text);
          argPanel.add(slider);
          break;
        case BOOLEAN:
          JCheckBox checkbox = new JCheckBox(setting.name);
          checkbox.setSelected(setting.enabled);
          checkbox.addChangeListener(this);
          checkbox.putClientProperty(SETTING_TAG, setting);
          argPanel.add(checkbox);
          break;
      }
    }
  }

  public void stateChanged(ChangeEvent e) {
    JComponent component = (JComponent) e.getSource();
    TestbedSetting setting = (TestbedSetting) component.getClientProperty(SETTING_TAG);

    switch (setting.constraintType) {
      case BOOLEAN:
        JCheckBox box = (JCheckBox) e.getSource();
        setting.enabled = box.isSelected();
        break;
      case RANGE:
        JSlider slider = (JSlider) e.getSource();
        setting.value = slider.getValue();
        JLabel label = (JLabel) slider.getClientProperty(LABEL_TAG);
        label.setText(setting.name + ": " + setting.value);
        break;
    }
  }

  public void actionPerformed(ActionEvent e) {
    controller.playTest(tests.getSelectedIndex());
  }
}
