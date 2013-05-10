/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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

import org.jbox2d.common.IViewportTransform;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.common.Vec2;

import com.google.common.base.Preconditions;

public class TestbedCamera {

  public static enum ZoomType {
    ZOOM_IN, ZOOM_OUT
  }

  private final Vec2 initPosition = new Vec2();
  private float initScale;

  private final IViewportTransform transform;

  private final Mat22 upScale;
  private final Mat22 downScale;

  public TestbedCamera(Vec2 initPosition, float initScale, float zoomScaleDiff) {
    Preconditions.checkArgument(zoomScaleDiff > 0, "Zoom scale %d must be > 0", zoomScaleDiff);
    this.transform = new OBBViewportTransform();
    transform.setCamera(initPosition.x, initPosition.y, initScale);
    this.initPosition.set(initPosition);
    this.initScale = initScale;
    upScale = Mat22.createScaleTransform(1 + zoomScaleDiff);
    downScale = Mat22.createScaleTransform(1 - zoomScaleDiff);
  }

  /**
   * Resets the camera to the initial position
   */
  public void reset() {
    setCamera(initPosition, initScale);
  }

  /**
   * Sets the camera center position
   */
  public void setCamera(Vec2 worldCenter) {
    transform.setCenter(worldCenter);
  }

  /**
   * Sets the camera center position and scale
   */
  public void setCamera(Vec2 worldCenter, float scale) {
    transform.setCamera(worldCenter.x, worldCenter.y, scale);
  }

  private final Vec2 oldCenter = new Vec2();
  private final Vec2 newCenter = new Vec2();

  /**
   * Zooms the camera to a point on the screen. The zoom amount is given on camera initialization.
   */
  public void zoomToPoint(Vec2 screenPosition, ZoomType zoomType) {
    Mat22 zoom;
    switch (zoomType) {
      case ZOOM_IN:
        zoom = upScale;
        break;
      case ZOOM_OUT:
        zoom = downScale;
        break;
      default:
        Preconditions.checkArgument(false, "Zoom type invalid");
        return;
    }

    transform.getScreenToWorld(screenPosition, oldCenter);
    transform.mulByTransform(zoom);
    transform.getScreenToWorld(screenPosition, newCenter);

    Vec2 transformedMove = oldCenter.subLocal(newCenter);
    // set, just in case bad impl by someone
    if (!transform.isYFlip()) {
      transformedMove.y = -transformedMove.y;
    }
    transform.setCenter(transform.getCenter().addLocal(transformedMove));
  }

  private final Vec2 worldDiff = new Vec2();

  /**
   * Moves the camera by the given distance in screen coordinates.
   */
  public void moveWorld(Vec2 screenDiff) {
    transform.getScreenVectorToWorld(screenDiff, worldDiff);
    if (!transform.isYFlip()) {
      worldDiff.y = -worldDiff.y;
    }
    transform.setCenter(transform.getCenter().addLocal(worldDiff));
  }

  public IViewportTransform getTransform() {
    return transform;
  }
}
