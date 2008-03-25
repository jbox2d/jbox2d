package org.jbox2d.dynamics;

import javax.vecmath.Color3f;

import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

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

	protected int m_drawFlags;
		
		public DebugDraw() {
			m_drawFlags = 0;
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

		//All the following should be overridden if the concrete drawing
		//class does any sort of camera movement
		
		/**
		 * Stub method to overload for camera movement/zoom.
		 * @param x - x coordinate of camera
		 * @param y - y coordinate of camera
		 * @param scale - zoom factor
		 */
		public void setCamera(float x, float y, float scale) {
			
		}

		/**
		 * @param screenV Screen position
		 * @return World position
		 */
		public Vec2 screenToWorld(Vec2 screenV) {
			return screenToWorld(screenV.x, screenV.y);
		}
		
		/**
		 * @param screenx Screen x position
		 * @param screeny Screey y position
		 * @return World position
		 */
		public Vec2 screenToWorld(float screenx, float screeny) {
			return new Vec2(screenx, screeny);
		}

		/**
		 * @param worldV World position
		 * @return Screen position
		 */
		public Vec2 worldToScreen(Vec2 worldV) {
			return worldToScreen(worldV.x, worldV.y);
		}
		
		/**
		 * @param worldx World x position
		 * @param worldy World y position
		 * @return Screen position
		 */
		public Vec2 worldToScreen(float worldx, float worldy) {
			return new Vec2(worldx, worldy);
		}

}
