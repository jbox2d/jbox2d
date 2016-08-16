/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met: * Redistributions of source code must retain the
 * above copyright notice, this list of conditions and the following disclaimer. * Redistributions
 * in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.testbed.framework.javafx;

import java.util.HashMap;
import java.util.Map;

import javax.swing.ComboBoxModel;
import javax.swing.DefaultComboBoxModel;
import javax.swing.event.ListDataEvent;
import javax.swing.event.ListDataListener;

import org.jbox2d.testbed.framework.AbstractTestbedController;
import org.jbox2d.testbed.framework.TestbedModel;
import org.jbox2d.testbed.framework.TestbedModel.ListItem;
import org.jbox2d.testbed.framework.TestbedSetting;
import org.jbox2d.testbed.framework.TestbedSetting.SettingType;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

import javafx.collections.ObservableList;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Control;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.cell.ComboBoxListCell;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.util.StringConverter;


/**
 * The testbed side panel. Facilitates test and setting changes.
 * 
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
public class TestbedSidePanel extends BorderPane {

  private static final String SETTING_TAG = "settings";
  private static final String LABEL_TAG = "label";

  final TestbedModel model;
  final AbstractTestbedController controller;

  public ComboBox<ListItem> tests;

  private Button pauseButton = new Button("Pause");
  private Button stepButton = new Button("Step");
  private Button resetButton = new Button("Reset");
  private Button quitButton = new Button("Quit");

  public Button saveButton = new Button("Save");
  public Button loadButton = new Button("Load");

  public TestbedSidePanel(TestbedModel argModel, AbstractTestbedController argController) {
    model = argModel;
    controller = argController;
    initComponents();
    addListeners();

    model.addTestChangeListener(new TestbedModel.TestChangedListener() {
      @Override
      public void testChanged(TestbedTest argTest, int argIndex) {
        tests.getSelectionModel().select(argIndex);
        saveButton.setDisable(!argTest.isSaveLoadEnabled());
        loadButton.setDisable(!argTest.isSaveLoadEnabled());
      }
    });
  }

  private void updateTests(ComboBoxModel<ListItem> model) {
    ObservableList<ListItem> list = tests.itemsProperty().get();
    list.clear();
    for (int i = 0; i < model.getSize(); i++) {
      list.add((ListItem) model.getElementAt(i));
    }
  }

  public void initComponents() {
    // setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

    TestbedSettings settings = model.getSettings();

    VBox top = new VBox();
    // top.setLayout(new GridLayout(0, 1));
    // top.setBorder(BorderFactory.createCompoundBorder(new EtchedBorder(EtchedBorder.LOWERED),
    // BorderFactory.createEmptyBorder(10, 10, 10, 10)));
    DefaultComboBoxModel testList = model.getComboModel();
    testList.addListDataListener(new ListDataListener() {
      @Override
      public void intervalRemoved(ListDataEvent e) {
        updateTests((ComboBoxModel<ListItem>) e.getSource());
      }

      @Override
      public void intervalAdded(ListDataEvent e) {
        updateTests((ComboBoxModel<ListItem>) e.getSource());
      }

      @Override
      public void contentsChanged(ListDataEvent e) {
        updateTests((ComboBoxModel<ListItem>) e.getSource());
      }
    });

    tests = new ComboBox<ListItem>();
    updateTests((ComboBoxModel<ListItem>) testList);
    tests.setOnAction((actionEvent) -> {
      testSelected();
    });
    tests.setCellFactory(ComboBoxListCell.<ListItem>forListView(new StringConverter<ListItem>() {
      @Override
      public String toString(ListItem listItem) {
        if (listItem == null) {
          return ("");
        } else if (listItem.isCategory()) {
          return (listItem.category);
        } else {
          return (listItem.test.getTestName());
        }
      }

      @Override
      public ListItem fromString(String string) {
        return null;
      }
    }, tests.getItems()));

    top.getChildren().add(new Label("Choose a test:"));
    top.getChildren().add(tests);

    addSettings(top, settings, SettingType.DRAWING);

    setTop(top);

    VBox middle = new VBox();
    // middle.setLayout(new GridLayout(0, 1));
    // middle.setBorder(BorderFactory.createCompoundBorder(new EtchedBorder(EtchedBorder.LOWERED),
    // BorderFactory.createEmptyBorder(5, 10, 5, 10)));

    addSettings(middle, settings, SettingType.ENGINE);

    setCenter(middle);

    pauseButton.setAlignment(Pos.CENTER);
    stepButton.setAlignment(Pos.CENTER);
    resetButton.setAlignment(Pos.CENTER);
    saveButton.setAlignment(Pos.CENTER);
    loadButton.setAlignment(Pos.CENTER);
    quitButton.setAlignment(Pos.CENTER);

    HBox buttonGroups = new HBox();
    VBox buttons1 = new VBox();
    buttons1.getChildren().add(resetButton);

    VBox buttons2 = new VBox();
    buttons2.getChildren().add(pauseButton);
    buttons2.getChildren().add(stepButton);

    VBox buttons3 = new VBox();
    buttons3.getChildren().add(saveButton);
    buttons3.getChildren().add(loadButton);
    buttons3.getChildren().add(quitButton);

    buttonGroups.getChildren().add(buttons1);
    buttonGroups.getChildren().add(buttons2);
    buttonGroups.getChildren().add(buttons3);

    setBottom(buttonGroups);
  }

  protected void testSelected() {
    int testNum = tests.getSelectionModel().getSelectedIndex();
    controller.playTest(testNum);
  }

  public void addListeners() {
    pauseButton.setOnAction((e) -> {
      if (model.getSettings().pause) {
        model.getSettings().pause = false;
        pauseButton.setText("Pause");
      } else {
        model.getSettings().pause = true;
        pauseButton.setText("Resume");
      }
      model.getPanel().grabFocus();
    });

    stepButton.setOnAction((e) -> {
      model.getSettings().singleStep = true;
      if (!model.getSettings().pause) {
        model.getSettings().pause = true;
        pauseButton.setText("Resume");
      }
      model.getPanel().grabFocus();
    });

    resetButton.setOnAction((e) -> {
      controller.reset();
    });

    quitButton.setOnAction((e) -> {
      System.exit(0);
    });

    saveButton.setOnAction((e) -> {
      controller.save();
    });

    loadButton.setOnAction((e) -> {
      controller.load();
    });
  }

  private void addSettings(Pane argPanel, TestbedSettings argSettings, SettingType argIgnore) {
    for (TestbedSetting setting : argSettings.getSettings()) {
      if (setting.settingsType == argIgnore) {
        continue;
      }
      switch (setting.constraintType) {
        case RANGE:
          Label text = new Label(setting.name + ": " + setting.value);
          Slider slider = new Slider(setting.min, setting.max, setting.value);
          // slider.setMaximumSize(new Dimension(200, 20));
          slider.valueProperty().addListener((prop, oldValue, newValue) -> {
            stateChanged(slider);
          });
          putClientProperty(slider, "name", setting.name);
          putClientProperty(slider, SETTING_TAG, setting);
          putClientProperty(slider, LABEL_TAG, text);
          argPanel.getChildren().add(text);
          argPanel.getChildren().add(slider);
          break;
        case BOOLEAN:
          CheckBox checkbox = new CheckBox(setting.name);
          checkbox.setSelected(setting.enabled);
          checkbox.selectedProperty().addListener((prop, oldValue, newValue) -> {
            stateChanged(checkbox);
          });
          putClientProperty(checkbox, SETTING_TAG, setting);
          argPanel.getChildren().add(checkbox);
          break;
      }
    }
  }

  private <T> T getClientProperty(Control control, String tag) {
    Map<String, Object> map = (Map<String, Object>) control.getUserData();
    return (map != null ? (T) map.get(tag) : null);
  }

  private void putClientProperty(Control control, String tag, Object o) {
    Map<String, Object> map = (Map<String, Object>) control.getUserData();
    if (map == null) {
      map = new HashMap<>();
      control.setUserData(map);
    }
    map.put(tag, o);
  }

  public void stateChanged(Control control) {
    TestbedSetting setting = getClientProperty(control, SETTING_TAG);

    switch (setting.constraintType) {
      case BOOLEAN:
        CheckBox box = (CheckBox) control;
        setting.enabled = box.isSelected();
        break;
      case RANGE:
        Slider slider = (Slider) control;
        setting.value = (int) slider.getValue();
        Label label = getClientProperty(slider, LABEL_TAG);
        label.setText(setting.name + ": " + setting.value);
        break;
    }
    model.getPanel().grabFocus();
  }
}
