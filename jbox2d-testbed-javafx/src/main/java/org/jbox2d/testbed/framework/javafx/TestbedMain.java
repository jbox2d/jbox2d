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

import org.jbox2d.testbed.framework.TestList;
import org.jbox2d.testbed.framework.AbstractTestbedController;
import org.jbox2d.testbed.framework.AbstractTestbedController.MouseBehavior;
import org.jbox2d.testbed.framework.AbstractTestbedController.UpdateBehavior;
import org.jbox2d.testbed.framework.TestbedModel;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.ScrollPane;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;

/**
 * The entry point for the testbed application
 * 
 * @author Daniel Murphy
 */
public class TestbedMain extends Application {
  // private static final Logger log = LoggerFactory.getLogger(TestbedMain.class);

  @Override
  public void start(Stage primaryStage) throws Exception {
    TestbedModel model = new TestbedModel();
    final AbstractTestbedController controller = new TestbedControllerJavaFX(model,
        UpdateBehavior.UPDATE_CALLED, MouseBehavior.NORMAL, (Exception e, String message) -> {
          new Alert(Alert.AlertType.ERROR).showAndWait();
        });
    TestPanelJavaFX panel = new TestPanelJavaFX(model, controller);
    model.setPanel(panel);
    model.setDebugDraw(new DebugDrawJavaFX(panel, true));
    TestList.populateModel(model);

    BorderPane testbed = new BorderPane();
    testbed.setCenter(panel);

    testbed.setRight(new ScrollPane(new TestbedSidePanel(model, controller)));

    Scene scene = new Scene(testbed, TestPanelJavaFX.INIT_WIDTH + 175, TestPanelJavaFX.INIT_HEIGHT);
    primaryStage.setScene(scene);
    primaryStage.setTitle("JBox2D Testbed");
    primaryStage.show();
    System.out.println(System.getProperty("java.home"));

    Platform.runLater(new Runnable() {
      @Override
      public void run() {
        controller.playTest(0);
        controller.start();
      }
    });
  }

  public static void main(String[] args) {
    launch(TestbedMain.class, args);
  }
}
