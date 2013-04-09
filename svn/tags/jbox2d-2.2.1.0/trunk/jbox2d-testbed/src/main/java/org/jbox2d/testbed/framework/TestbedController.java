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
package org.jbox2d.testbed.framework;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;

import org.jbox2d.common.Vec2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class contains most control logic for the testbed and the update loop. It also watches the
 * model to switch tests and populates the model with some loop statistics.
 * 
 * @author Daniel Murphy
 */
public class TestbedController implements Runnable {
  private static final Logger log = LoggerFactory.getLogger(TestbedController.class);

  public static enum UpdateBehavior {
    UPDATE_CALLED, UPDATE_IGNORED 
  }
  
  public static final int DEFAULT_FPS = 60;

  private TestbedTest currTest = null;
  private TestbedTest nextTest = null;

  private long startTime;
  private long frameCount;
  private int targetFrameRate;
  private float frameRate = 0;
  private boolean animating = false;
  private Thread animator;

  private final TestbedModel model;
  private final TestbedPanel panel;
  
  private UpdateBehavior updateBehavior;

  public TestbedController(TestbedModel argModel, TestbedPanel argPanel, UpdateBehavior behavior) {
    model = argModel;
    setFrameRate(DEFAULT_FPS);
    panel = argPanel;
    animator = new Thread(this, "Testbed");
    updateBehavior = behavior;
    addListeners();
  }
  
  private void addListeners(){
    // time for our controlling
    model.addTestChangeListener(new TestbedModel.TestChangedListener() {
      @Override
      public void testChanged(TestbedTest argTest, int argIndex) {
        nextTest = argTest;
        panel.grabFocus();
      }
    });
    panel.addKeyListener(new KeyListener() {
      @Override
      public void keyTyped(KeyEvent e) {
      }

      @Override
      public void keyReleased(KeyEvent e) {
        char key = e.getKeyChar();
        int code = e.getKeyCode();
        if (key != KeyEvent.CHAR_UNDEFINED) {
          model.getKeys()[key] = false;
        }
        model.getCodedKeys()[code] = false;
        if (model.getCurrTest() != null) {
          model.getCurrTest().queueKeyReleased(key, code);
        }
      }

      @Override
      public void keyPressed(KeyEvent e) {
        char key = e.getKeyChar();
        int code = e.getKeyCode();
        if (key != KeyEvent.CHAR_UNDEFINED) {
          model.getKeys()[key] = true;
        }
        model.getCodedKeys()[code] = true;

        if (key == ' ' && model.getCurrTest() != null) {
          model.getCurrTest().lanchBomb();
        } else if (key == '[') {
          lastTest();
        } else if (key == ']') {
          nextTest();
        } else if (key == 'r') {
          resetTest();
        }
        else if (model.getCurrTest() != null) {
          model.getCurrTest().queueKeyPressed(key, code);
        }
      }
    });

    panel.addMouseListener(new MouseAdapter() {
      @Override
      public void mouseReleased(MouseEvent e) {
        if (model.getCurrTest() != null) {
          Vec2 pos = new Vec2(e.getX(), e.getY());
          model.getDebugDraw().getScreenToWorldToOut(pos, pos);
          model.getCurrTest().queueMouseUp(pos);
        }
      }

      @Override
      public void mousePressed(MouseEvent e) {
        panel.grabFocus();
        if (model.getCurrTest() != null) {
          Vec2 pos = new Vec2(e.getX(), e.getY());
          if (e.getButton() == MouseEvent.BUTTON1) {
            model.getDebugDraw().getScreenToWorldToOut(pos, pos);
            model.getCurrTest().queueMouseDown(pos);
            if (model.getCodedKeys()[KeyEvent.VK_SHIFT]) {
              model.getCurrTest().queueShiftMouseDown(pos);
            }
          }
        }
      }
    });

    panel.addMouseMotionListener(new MouseMotionListener() {
      final Vec2 posDif = new Vec2();
      final Vec2 pos = new Vec2();
      final Vec2 pos2 = new Vec2();

      public void mouseDragged(MouseEvent e) {
        pos.set(e.getX(), e.getY());

        if (e.getButton() == MouseEvent.BUTTON3) {
          posDif.set(model.getMouse());
          model.setMouse(pos);
          posDif.subLocal(pos);
          if(!model.getDebugDraw().getViewportTranform().isYFlip()){
            posDif.y *= -1;
          }
          model.getDebugDraw().getViewportTranform().getScreenVectorToWorld(posDif, posDif);
          model.getDebugDraw().getViewportTranform().getCenter().addLocal(posDif);
          if (model.getCurrTest() != null) {
            model.getCurrTest().setCachedCameraPos(
                model.getDebugDraw().getViewportTranform().getCenter());
          }
        }
        if (model.getCurrTest() != null) {
          model.setMouse(pos);
          model.getDebugDraw().getScreenToWorldToOut(pos, pos);
          model.getCurrTest().queueMouseMove(pos);
        }
      }

      @Override
      public void mouseMoved(MouseEvent e) {
        pos2.set(e.getX(), e.getY());
        model.setMouse(pos2);
        if (model.getCurrTest() != null) {
          model.getDebugDraw().getScreenToWorldToOut(pos2, pos2);
          model.getCurrTest().queueMouseMove(pos2);
        }
      }
    });
  }
  
  protected void loopInit() {
    panel.grabFocus();
    
    if (currTest != null) {
      currTest.init(model);
    }
  }

  protected void update() {
    if (currTest != null && updateBehavior == UpdateBehavior.UPDATE_CALLED) {
      currTest.update();
    }
  }

  public void nextTest() {
    int index = model.getCurrTestIndex() + 1;
    index %= model.getTestsSize();

    while (!model.isTestAt(index) && index < model.getTestsSize() - 1) {
      index++;
    }
    if (model.isTestAt(index)) {
      model.setCurrTestIndex(index);
    }
  }
  
  public void resetTest(){
    model.getCurrTest().reset();
  }

  public void saveTest() {
    model.getCurrTest().save();
  }

  public void loadTest() {
    model.getCurrTest().load();
  }

  public void lastTest() {
    int index = model.getCurrTestIndex() - 1;
    index = (index < 0) ? index + model.getTestsSize() : index;

    while (!model.isTestAt(index) && index > 0) {
      index--;
    }

    if (model.isTestAt(index)) {
      model.setCurrTestIndex(index);
    }
  }
  
  public void playTest(int argIndex){
    if (argIndex == -1) {
      return;
    }
    while (!model.isTestAt(argIndex)) {
      if (argIndex + 1 < model.getTestsSize()) {
        argIndex++;
      } else {
        return;
      }
    }
    model.setCurrTestIndex(argIndex);
  }

  public void setFrameRate(int fps) {
    if (fps <= 0) {
      throw new IllegalArgumentException("Fps cannot be less than or equal to zero");
    }
    targetFrameRate = fps;
    frameRate = fps;
  }

  public int getFrameRate() {
    return targetFrameRate;
  }

  public float getCalculatedFrameRate() {
    return frameRate;
  }

  public long getStartTime() {
    return startTime;
  }

  public long getFrameCount() {
    return frameCount;
  }

  public boolean isAnimating() {
    return animating;
  }

  public synchronized void start() {
    if (animating != true) {
      frameCount = 0;
      animator.start();
    } else {
      log.warn("Animation is already animating.");
    }
  }

  public synchronized void stop() {
    animating = false;
  }

  public void run() {
    long beforeTime, afterTime, updateTime, timeDiff, sleepTime, timeSpent;
    float timeInSecs;
    beforeTime = startTime = updateTime = System.nanoTime();
    sleepTime = 0;

    animating = true;
    loopInit();
    while (animating) {

      if (nextTest != null) {
        nextTest.init(model);
        model.setRunningTest(nextTest);
        if(currTest != null) {
          currTest.exit();    		
        }
        currTest = nextTest;
        nextTest = null;
      }

      timeSpent = beforeTime - updateTime;
      if (timeSpent > 0) {
        timeInSecs = timeSpent * 1.0f / 1000000000.0f;
        updateTime = System.nanoTime();
        frameRate = (frameRate * 0.9f) + (1.0f / timeInSecs) * 0.1f;
        model.setCalculatedFps(frameRate);
      } else {
        updateTime = System.nanoTime();
      }

      if(panel.render()) {
        update();
        panel.paintScreen();        
      }
      frameCount++;

      afterTime = System.nanoTime();

      timeDiff = afterTime - beforeTime;
      sleepTime = (1000000000 / targetFrameRate - timeDiff) / 1000000;
      if (sleepTime > 0) {
        try {
          Thread.sleep(sleepTime);
        } catch (InterruptedException ex) {
        }
      }

      beforeTime = System.nanoTime();
    } // end of run loop
  }
}
