/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.dynamics;


import org.jbox2d.common.Color3f;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.IViewportTransform;
import org.jbox2d.common.XForm;

/**
 * Implement this abstract class to allow JBox2d to 
 * automatically draw your physics for debugging purposes.
 * Not intended to replace your own custom rendering
 * routines!
 */
public abstract class DebugDraw {
	/// Implement and register this class with a b2World to provide debug drawing of physics
	/// entities in your game.

	public static final int e_shapeBit				= 0x0001; ///< draw shapes
	public static final int e_jointBit				= 0x0002; ///< draw joint connections
	public static final int e_coreShapeBit			= 0x0004; ///< draw core (TOI) shapes
	public static final int e_aabbBit				= 0x0008; ///< draw axis aligned bounding boxes
	public static final int e_obbBit				= 0x0010; ///< draw oriented bounding boxes
	public static final int e_pairBit				= 0x0020; ///< draw broad-phase pairs
	public static final int e_centerOfMassBit		= 0x0040; ///< draw center of mass frame
	public static final int e_controllerBit			= 0x0080; ///< draw controllers

	protected int m_drawFlags;
	protected final IViewportTransform viewportTransform;

	public DebugDraw(IViewportTransform viewport) {
		m_drawFlags = 0;
		viewportTransform = viewport;
	}

	public void setFlags(int flags) {
		m_drawFlags = flags;
	}

	public int getFlags() {
		return m_drawFlags;
	}

	public void appendFlags(int flags) {
		m_drawFlags |= flags;
	}

	public void clearFlags(int flags) {
		m_drawFlags &= ~flags;
	}

	/// Draw a closed polygon provided in CCW order.
	public abstract void drawPolygon(Vec2[] vertices, int vertexCount, Color3f color);

	/// Draw a solid closed polygon provided in CCW order.
	public abstract void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color);

	/// Draw a circle.
	public abstract void drawCircle(Vec2 center, float radius, Color3f color);
	
	/// Draw a solid circle.
	public abstract void drawSolidCircle(Vec2 center, float radius, Vec2 axis, Color3f color);
	
	/// Draw a point.
	public abstract void drawPoint(Vec2 position, float f, Color3f color3f);
	
	/// Draw a line segment.
	public abstract void drawSegment(Vec2 p1, Vec2 p2, Color3f color);

	/// Draw a transform. Choose your own length scale.
	/// @param xf a transform.
	public abstract void drawXForm(XForm xf);

	public abstract void drawString(float x, float y, String s, Color3f color);
	
	public IViewportTransform getViewportTranform(){
		return viewportTransform;
	}

	/**
	 * @param x
	 * @param y
	 * @param scale
	 * @see IViewportTransform#setCamera(float, float, float)
	 */
	public void setCamera(float x, float y, float scale){
		viewportTransform.setCamera(x,y,scale);
	}
	
	
	/**
	 * @param argScreen
	 * @param argWorld
	 * @see org.jbox2d.common.IViewportTransform#getScreenToWorld(org.jbox2d.common.Vec2, org.jbox2d.common.Vec2)
	 */
	public void getScreenToWorldToOut(Vec2 argScreen, Vec2 argWorld) {
		viewportTransform.getScreenToWorld(argScreen, argWorld);
	}

	/**
	 * @param argWorld
	 * @param argScreen
	 * @see org.jbox2d.common.IViewportTransform#getWorldToScreen(org.jbox2d.common.Vec2, org.jbox2d.common.Vec2)
	 */
	public void getWorldToScreenToOut(Vec2 argWorld, Vec2 argScreen) {
		viewportTransform.getWorldToScreen(argWorld, argScreen);
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
		viewportTransform.getWorldToScreen(argScreen, argScreen);
	}
	
	/**
	 * takes the world coordinate (argWorld) and returns
	 * the screen coordinates.
	 * @param argWorld
	 */
	public Vec2 getWorldToScreen(Vec2 argWorld){
		Vec2 screen = new Vec2();
		viewportTransform.getWorldToScreen( argWorld, screen);
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
		viewportTransform.getWorldToScreen( argScreen, argScreen);
		return argScreen;
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
		viewportTransform.getScreenToWorld(argWorld, argWorld);
	}
	
	/**
	 * takes the screen coordinates (argScreen) and returns
	 * the world coordinates
	 * @param argScreen
	 */
	public Vec2 getScreenToWorld(Vec2 argScreen){
		Vec2 world = new Vec2();
		viewportTransform.getScreenToWorld(argScreen, world);
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
		viewportTransform.getScreenToWorld( screen, screen);
		return screen;
	}
}
