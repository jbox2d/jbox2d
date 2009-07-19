/**
 * 
 * 1:13:11 AM, Jul 17, 2009
 */
package org.jbox2d.common;

import org.jbox2d.collision.OBB;

/**
 * This is the viewport transform used from drawing.
 * Use yFlip if you are drawing from the top-left corner.
 * @author daniel
 */
public class ViewportTransform {
	
	private final OBB box = new OBB();
	public boolean yFlip = false;
	private final Mat22 yFlipMat = new Mat22(1,0,0,-1);
	private final Mat22 yFlipMatInv = yFlipMat.invert();
	
	public ViewportTransform(){
		box.R.setIdentity();
	}
	
	public void set(ViewportTransform vpt){
		box.center.set(vpt.box.center);
		box.extents.set( vpt.box.extents);
		box.R.set( vpt.box.R);
		yFlip = vpt.yFlip;
	}
	
	/**
	 * This is the half-width and half-height.
	 * This should be the actual half-width and 
	 * half-height, not anything transformed or scaled.
	 * Not a copy.
	 * @return
	 */
	public Vec2 getExtents(){
		return box.extents;
	}
	
	/**
	 * This sets the half-width and half-height.
	 * This should be the actual half-width and 
	 * half-height, not anything transformed or scaled.
	 * @param argExtents
	 */
	public void setExtents(Vec2 argExtents){
		box.extents.set(argExtents);
	}
	
	/**
	 * This sets the half-width and half-height of the
	 * viewport. This should be the actual half-width and 
	 * half-height, not anything transformed or scaled.
	 * @param argHalfWidth
	 * @param argHalfHeight
	 */
	public void setExtents(float argHalfWidth, float argHalfHeight){
		box.extents.set(argHalfWidth, argHalfHeight);
	}
	
	/**
	 * center of the viewport.  Not a copy.
	 * @return
	 */
	public Vec2 getCenter(){
		return box.center;
	}
	
	/**
	 * set the center of the viewport.
	 * @param argPos
	 */
	public void setCenter(Vec2 argPos){
		box.center.set(argPos);
	}
	
	/**
	 * sets the center of the viewport.
	 * @param x
	 * @param y
	 */
	public void setCenter(float x, float y){
		box.center.set(x,y);
	}
	
	/**
	 * gets the transform of the viewport, transforms around the center.
	 * Not a copy.
	 * @return
	 */
	public Mat22 getTransform(){
		return box.R;
	}
	
	/**
	 * sets the transform of the viewport.  Transforms about the center.
	 * @param transform
	 */
	public void setTransform(Mat22 transform){
		box.R.set(transform);
	}
	
	/**
	 * Transforms by the given matrix.
	 * @see Mat22#createRotationalTransform(float, Mat22)
	 * @see Mat22#createScaleTransform(float, Mat22)
	 * @param transform
	 */
	public void mulByTransform(Mat22 transform){
		box.R.mulLocal(transform);
	}
	
	/**
	 * takes the world coordinate (argWorld) puts the corresponding
	 * screen coordinate in argScreen.
	 * @param argWorld
	 * @param argScreen
	 */
	public void getWorldToScreenToOut(Vec2 argWorld, Vec2 argScreen){
		argScreen.set(argWorld);
		argScreen.subLocal(box.center);
		box.R.mulToOut(argScreen, argScreen);
		if(yFlip){
			yFlipMat.mulToOut( argScreen, argScreen);
		}
		argScreen.addLocal(box.extents);
	}
	
	/**
	 * Takes the world coordinates and puts the corresponding screen
	 * coordinates in argScreen.
	 * @param worldX
	 * @param worldY
	 * @param argScreen
	 */
	public void getWorldToScreenToOut(float worldX, float worldY, Vec2 argScreen){
		argScreen.set(worldX,worldY);
		argScreen.subLocal(box.center);
		box.R.mulToOut(argScreen, argScreen);
		if(yFlip){
			yFlipMat.mulToOut( argScreen, argScreen);
		}
		argScreen.addLocal(box.extents);
	}
	
	/**
	 * takes the world coordinate (argWorld) and returns
	 * the screen coordinates.
	 * @param argWorld
	 */
	public Vec2 getWorldToScreen(Vec2 argWorld){
		Vec2 screen = new Vec2();
		getWorldToScreenToOut( argWorld, screen);
		return screen;
	}
	
	/**
	 * Takes the world coordinates and returns the screen
	 * coordinates.
	 * @param worldX
	 * @param worldY
	 */
	public Vec2 getWorldToScreen(float worldX, float worldY){
		Vec2 argScreen = new Vec2(worldX, worldY);
		getWorldToScreenToOut( argScreen, argScreen);
		return argScreen;
	}
	
	
	/**
	 * takes the screen coordinates (argScreen) and puts the
	 * corresponding world coordinates in argWorld.
	 * @param argScreen
	 * @param argWorld
	 */
	public void getScreenToWorldToOut(Vec2 argScreen, Vec2 argWorld){
		argWorld.set(argScreen);
		argWorld.subLocal(box.extents);
		Mat22 inv = box.R.invert();
		inv.mulToOut(argWorld, argWorld);
		if(yFlip){
			yFlipMatInv.mulToOut( argWorld, argWorld);
		}
		argWorld.addLocal(box.center);
	}
	
	/**
	 * takes the screen coordinates and puts the corresponding 
	 * world coordinates in argWorld.
	 * @param screenX
	 * @param screenY
	 * @param argWorld
	 */
	public void getScreenToWorldToOut(float screenX, float screenY, Vec2 argWorld){
		argWorld.set(screenX,screenY);
		argWorld.subLocal(box.extents);
		Mat22 inv = box.R.invert();
		inv.mulToOut(argWorld, argWorld);
		if(yFlip){
			yFlipMatInv.mulToOut( argWorld, argWorld);
		}
		argWorld.addLocal(box.center);
	}
	
	/**
	 * takes the screen coordinates (argScreen) and returns
	 * the world coordinates
	 * @param argScreen
	 */
	public Vec2 getScreenToWorld(Vec2 argScreen){
		Vec2 world = new Vec2();
		getScreenToWorldToOut( argScreen, world);
		return world;
	}
	
	/**
	 * takes the screen coordinates and returns the
	 * world coordinates.
	 * @param screenX
	 * @param screenY
	 */
	public Vec2 getScreenToWorld(float screenX, float screenY){
		Vec2 screen = new Vec2(screenX, screenY);
		getScreenToWorldToOut( screen, screen);
		return screen;
	}
}
