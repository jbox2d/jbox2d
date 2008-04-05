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
package org.jbox2d.integrations.slick;

import org.jbox2d.common.Color3f;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.DebugDraw;
import org.newdawn.slick.Color;
import org.newdawn.slick.GameContainer;
import org.newdawn.slick.Graphics;
import org.newdawn.slick.geom.Circle;
import org.newdawn.slick.geom.Polygon;
import org.newdawn.slick.opengl.TextureImpl;

/**
 * Not fully implemented - just enough here to get the Pyramid
 * demo to draw.
 *
 */
public class SlickDebugDraw extends DebugDraw {
	Graphics g;
	GameContainer container;

	// World 0,0 maps to transX, transY on screen
    public float transX = 270.0f;
    public float transY = 250.0f;
    public float scaleFactor = 10.0f;
    public float yFlip = -1.0f; //flip y coordinate
    
    public float map(float mapMe, float fromLow, float fromHigh, float toLow, float toHigh) {
    	float interp = (mapMe - fromLow) / (fromHigh - fromLow);
    	return (interp*toHigh + (1.0f-interp)*toLow);
    }
    
    public Vec2 worldToScreen(Vec2 world) {
		float x = map(world.x, 0f, 1f, transX, transX+scaleFactor);
		float y = map(world.y, 0f, 1f, transY, transY+scaleFactor);
		if (yFlip == -1.0f) y = map(y,0f,container.getHeight(), container.getHeight(),0f);
		return new Vec2(x, y);
	}
	public Vec2 worldToScreen(float x, float y) {
		return worldToScreen(new Vec2(x,y));
	}
	
	public Vec2 screenToWorld(Vec2 screen) {
		float x = map(screen.x, transX, transX+scaleFactor, 0f, 1f);
		float y = screen.y;
		if (yFlip == -1.0f) y = map(y,container.getHeight(),0f,0f,container.getHeight());
		y = map(y, transY, transY+scaleFactor, 0f, 1f);
		return new Vec2(x,y);
	}
	public Vec2 screenToWorld(float x, float y) {
		return screenToWorld(new Vec2(x,y));
	}
	
	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawCircle(org.jbox2d.common.Vec2, float, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawCircle(Vec2 center, float radius, Color3f color) {
		// TODO Auto-generated method stub
		center = worldToScreen(center);
		Circle circle = new Circle(center.x,center.y,radius);
		Color slickColor = new Color(color.x/255.0f,color.y/255.0f,color.z/255.0f);
		g.setColor(slickColor);
		g.draw(circle);
	}

	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawPoint(org.jbox2d.common.Vec2, float, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawPoint(Vec2 position, float f, Color3f color3f) {
		// TODO Auto-generated method stub

	}

	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawPolygon(org.jbox2d.common.Vec2[], int, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
		Polygon polygon = new Polygon();
		for (int i=0; i<vertexCount; ++i) {
			Vec2 screenPt = worldToScreen(vertices[i]);
			polygon.addPoint(screenPt.x, screenPt.y);
		}
		Color slickColor = new Color(color.x/255.0f,color.y/255.0f,color.z/255.0f);
		g.setColor(slickColor);
		g.draw(polygon);
	}

	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawSolidPolygon(org.jbox2d.common.Vec2[], int, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
		Polygon polygon = new Polygon();
		for (int i=0; i<vertexCount; ++i) {
			Vec2 screenPt = worldToScreen(vertices[i]);
			polygon.addPoint(screenPt.x, screenPt.y);
		}
		Color slickColor = new Color(color.x/255.0f,color.y/255.0f,color.z/255.0f);
		g.setColor(slickColor);
		g.draw(polygon);
		slickColor.a = 0.5f;
		g.fill(polygon);
	}
	
	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawSegment(org.jbox2d.common.Vec2, org.jbox2d.common.Vec2, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawSegment(Vec2 p1, Vec2 p2, Color3f color) {
		Color slickColor = new Color(color.x/255.0f,color.y/255.0f,color.z/255.0f);
		g.setColor(slickColor);
		TextureImpl.bindNone();
		g.setLineWidth(1);
		g.setAntiAlias(false);
		Vec2 screen1 = worldToScreen(p1);
		Vec2 screen2 = worldToScreen(p2);
		g.drawLine(screen1.x,screen1.y,screen2.x,screen2.y);
	}

	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawSolidCircle(org.jbox2d.common.Vec2, float, org.jbox2d.common.Vec2, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawSolidCircle(Vec2 center, float radius, Vec2 axis,
			Color3f color) {
		// TODO Auto-generated method stub

	}


	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawString(float, float, java.lang.String, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawString(float x, float y, String s, Color3f color) {
		// TODO Auto-generated method stub

	}

	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawXForm(org.jbox2d.common.XForm)
	 */
	@Override
	public void drawXForm(XForm xf) {
		// TODO Auto-generated method stub

	}

}
