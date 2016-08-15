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
package org.jbox2d.testbed.framework;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class contains most control logic for the testbed and the update loop. It also watches the
 * model to switch tests and populates the model with some loop statistics.
 * 
 * @author Daniel Murphy
 */
public class TestbedController extends AbstractTestbedController implements Runnable {
  private static final Logger log = LoggerFactory.getLogger(TestbedController.class);

  private Thread animator;

  public TestbedController(TestbedModel argModel, UpdateBehavior behavior,
      MouseBehavior mouseBehavior, TestbedErrorHandler errorHandler) {
    super(argModel, behavior, mouseBehavior, errorHandler);
    animator = new Thread(this, "Testbed");
  }

  @Override
  public void startAnimator() {
    super.startAnimator();
    if (!animator.isAlive()) {
      animator.start();
    }
  }

  @Override
  public void stopAnimator() {
    super.stopAnimator();
    animator.stop();
  }

  @Override
  public void run() {
    beforeTime = startTime = updateTime = System.nanoTime();
    sleepTime = 0;
    startAnimator();
    loopInit();
    while (isAnimating()) {
      stepAndRender();
    }
  }

  @Override
  protected void stepAndRender() {
    super.stepAndRender();
    if (sleepTime > 0) {
      try {
        Thread.sleep(sleepTime);
      } catch (InterruptedException ex) {
      }
    }
  }
}
