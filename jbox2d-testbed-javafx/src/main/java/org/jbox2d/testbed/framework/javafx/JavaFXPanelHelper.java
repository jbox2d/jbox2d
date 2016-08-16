package org.jbox2d.testbed.framework.javafx;

import java.util.List;

import org.jbox2d.common.Vec2;
import org.jbox2d.testbed.framework.TestbedCamera.ZoomType;
import org.jbox2d.testbed.framework.AbstractTestbedController;
import org.jbox2d.testbed.framework.TestbedModel;
import org.jbox2d.testbed.framework.TestbedTest;

import com.google.common.collect.Lists;

import javafx.scene.Node;
import javafx.scene.input.KeyCode;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;

public class JavaFXPanelHelper {

  static boolean screenDragButtonDown = false;
  static boolean mouseJointButtonDown = false;


  /**
   * Adds common help text and listeners for awt-based testbeds.
   */
  public static void addHelpAndPanelListeners(Node panel, final TestbedModel model,
      final AbstractTestbedController controller, final int screenDragButton) {
    final Vec2 oldDragMouse = new Vec2();
    final Vec2 mouse = new Vec2();
    List<String> help = Lists.newArrayList();
    help.add("Click and drag the left mouse button to move objects.");
    help.add("Click and drag the right mouse button to move the view.");
    help.add("Shift-Click to aim a bullet, or press space.");
    help.add("Scroll to zoom in/out on the mouse position");
    help.add("Press '[' or ']' to change tests, and 'r' to restart.");
    model.setImplSpecificHelp(help);

    panel.setOnZoom((zoomEvent) -> {
      TestbedTest currTest = model.getCurrTest();
      if (currTest == null) {
        return;
      }
      ZoomType zoom = zoomEvent.getZoomFactor() > 1 ? ZoomType.ZOOM_IN : ZoomType.ZOOM_OUT;
      currTest.getCamera().zoomToPoint(mouse, zoom);
    });

    panel.setOnMouseReleased((mouseEvent) -> {
      if (toInt(mouseEvent.getButton()) == screenDragButton) {
        screenDragButtonDown = false;
      } else if (model.getCodedKeys()[toInt(KeyCode.SHIFT)] && !mouseJointButtonDown) {
        controller.queueMouseUp(toVec(mouseEvent), TestbedTest.BOMB_SPAWN_BUTTON);
      } else {
        if (toInt(mouseEvent.getButton()) == TestbedTest.MOUSE_JOINT_BUTTON) {
          mouseJointButtonDown = false;
        }
        controller.queueMouseUp(new Vec2((float) mouseEvent.getX(), (float) mouseEvent.getY()),
            toInt(mouseEvent.getButton()));
      }
    });

    panel.setOnMousePressed((mouseEvent) -> {
      if (toInt(mouseEvent.getButton()) == screenDragButton) {
        screenDragButtonDown = true;
        oldDragMouse.set(toVec(mouseEvent));
        return;
      } else if (model.getCodedKeys()[toInt(KeyCode.SHIFT)]) {
        controller.queueMouseDown(toVec(mouseEvent), TestbedTest.BOMB_SPAWN_BUTTON);
      } else {
        if (toInt(mouseEvent.getButton()) == TestbedTest.MOUSE_JOINT_BUTTON) {
          mouseJointButtonDown = true;
        }
        controller.queueMouseDown(toVec(mouseEvent), toInt(mouseEvent.getButton()));
      }
    });

    panel.setOnMouseMoved((mouseEvent) -> {
      mouse.set(toVec(mouseEvent));
      controller.queueMouseMove(toVec(mouseEvent));
    });

    panel.setOnMouseDragged((mouseEvent) -> {
      mouse.set(toVec(mouseEvent));
      if (screenDragButtonDown) {
        TestbedTest currTest = model.getCurrTest();
        if (currTest == null) {
          return;
        }
        Vec2 diff = oldDragMouse.sub(mouse);
        currTest.getCamera().moveWorld(diff);
        oldDragMouse.set(mouse);
      } else if (mouseJointButtonDown) {
        controller.queueMouseDrag(new Vec2(mouse), TestbedTest.MOUSE_JOINT_BUTTON);
      } else if (model.getCodedKeys()[toInt(KeyCode.SHIFT)]) {
        controller.queueMouseDrag(toVec(mouseEvent), TestbedTest.BOMB_SPAWN_BUTTON);
      } else {
        controller.queueMouseDrag(toVec(mouseEvent), toInt(mouseEvent.getButton()));
      }
    });

    panel.setOnKeyReleased((keyEvent) -> {
      controller.queueKeyReleased(keyEvent.getCharacter().charAt(0), toInt(keyEvent.getCode()));
    });

    panel.setOnKeyPressed((keyEvent) -> {
      String keyName = keyEvent.getText();
      char c = (keyName.length() == 1 ? keyName.charAt(0) : '\0');
      controller.queueKeyPressed(c, toInt(keyEvent.getCode()));
      switch (c) {
        case '[':
          controller.lastTest();
          break;
        case ']':
          controller.nextTest();
          break;
        case 'r':
          controller.reset();
          break;
        case ' ':
          controller.queueLaunchBomb();
          break;
        case 'p':
          controller.queuePause();
          break;
      }
    });
  }

  private static int toInt(KeyCode key) {
    return key.ordinal();
  }

  private static int toInt(MouseButton button) {
    return button.ordinal();
  }

  private static Vec2 toVec(MouseEvent mouseEvent) {
    return new Vec2((float) mouseEvent.getX(), (float) mouseEvent.getY());
  }
}
