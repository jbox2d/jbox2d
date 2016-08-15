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

import org.jbox2d.testbed.framework.AbstractTestbedController;
import org.jbox2d.testbed.framework.TestbedModel;
import org.jbox2d.testbed.framework.TestbedPanel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.geometry.Bounds;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.input.MouseButton;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;

/**
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
public class TestPanelJavaFX extends Canvas implements TestbedPanel {
  private static final Logger log = LoggerFactory.getLogger(TestPanelJavaFX.class);

  public static final int SCREEN_DRAG_BUTTON = MouseButton.SECONDARY.ordinal();

  public static final int INIT_WIDTH = 600;
  public static final int INIT_HEIGHT = 600;

  private final AbstractTestbedController controller;

  public TestPanelJavaFX(final TestbedModel model, final AbstractTestbedController controller) {
    super(INIT_WIDTH, INIT_HEIGHT);
    this.controller = controller;
    updateSize(INIT_WIDTH, INIT_HEIGHT);

    JavaFXPanelHelper.addHelpAndPanelListeners(this, model, controller, SCREEN_DRAG_BUTTON);
    ChangeListener<Number> sizeListener = (prop, oldValue, newValue) -> {
      updateSize(getWidth(), getHeight());
    };
    widthProperty().addListener(sizeListener);
    heightProperty().addListener(sizeListener);

    getGraphicsContext2D().setFont(new Font("Courier New", 12));
  }

  @Override
  public double maxWidth(double height) {
    return Double.MAX_VALUE;
  }

  @Override
  public double maxHeight(double width) {
    return Double.MAX_VALUE;
  }

  public GraphicsContext getDBGraphics() {
    return getGraphicsContext2D();
  }

  private void updateSize(double width, double height) {
    controller.updateExtents((float) width / 2, (float) height / 2);
  }

  public boolean render() {
    GraphicsContext dbg = getDBGraphics();
    dbg.setFill(Color.BLACK);
    Bounds bounds = getBoundsInLocal();
    dbg.fillRect(bounds.getMinX(), bounds.getMinX(), bounds.getWidth(), bounds.getHeight());
    return true;
  }

  public void paintScreen() {}

  @Override
  public void grabFocus() {
    Platform.runLater(this::requestFocus);
  }
}
