/**
 * Created at 3:09:27 AM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.arrays.IntArray;
import org.jbox2d.testbed.pooling.ColorPool;

// pooling local, not thread-safe
/**
 * Implementation of {@link DebugDraw} that uses Java2D! Hooray!</br>
 * @author Daniel Murphy
 */
public class DebugDrawJ2D extends DebugDraw{

	private final TestPanel panel;
	private final ColorPool cpool = new ColorPool();
	public Graphics2D graphics = null;
	/**
	 * @param viewport
	 */
	public DebugDrawJ2D(TestPanel argTestPanel) {
		super(new OBBViewportTransform());
		viewportTransform.setYFlip(true);
		panel = argTestPanel;
	}

	private final Vec2 sCenter = new Vec2();
	/**
	 * @see org.jbox2d.callbacks.DebugDraw#drawCircle(org.jbox2d.common.Vec2, float, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawCircle(Vec2 center, float radius, Color3f color) {
		getWorldToScreenToOut(center, sCenter);
		Graphics2D g = getGraphics();		
		Color c = cpool.getColor(color.x, color.y, color.z);
		g.setColor(c);
		
		g.drawOval((int)sCenter.x, (int)sCenter.y, (int)radius, (int)radius);
	}

	private final Vec2 sp1 = new Vec2();
	private final Vec2 sp2 = new Vec2();
	/**
	 * @see org.jbox2d.callbacks.DebugDraw#drawSegment(org.jbox2d.common.Vec2, org.jbox2d.common.Vec2, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawSegment(Vec2 p1, Vec2 p2, Color3f color) {
		getWorldToScreenToOut(p1, sp1);
		getWorldToScreenToOut(p2, sp2);
		
		Graphics2D g = getGraphics();		
		Color c = cpool.getColor(color.x, color.y, color.z);
		g.setColor(c);
		
		g.drawLine((int)sp1.x, (int)sp1.y, (int)sp2.x, (int)sp2.y);
	}

	private final Vec2 saxis = new Vec2();
	private final Vec2 sup = new Vec2(0,1);
	private final Vec2 sright = new Vec2(1,0);
	/**
	 * @see org.jbox2d.callbacks.DebugDraw#drawSolidCircle(org.jbox2d.common.Vec2, float, org.jbox2d.common.Vec2, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawSolidCircle(Vec2 center, float radius, Vec2 axis, Color3f color) {
		// outside
		drawCircle(center, radius, color);
		
		getWorldToScreenToOut(center, sCenter);
		Graphics2D g = getGraphics();		
		
		// inside
		Color c;
		if(axis != null){
			c = cpool.getColor(color.x, color.y, color.z);
			g.setColor(c);
			getWorldToScreenToOut(center.x + radius * axis.x, center.y + radius * axis.y, saxis);
			g.drawLine((int)sCenter.x, (int)sCenter.y, (int)saxis.x, (int)saxis.y);
		}
		
		sup.set(radius,0);
		getViewportTranform().vectorTransform(sup, sup);
		
		sright.set(0,radius);
		getViewportTranform().vectorTransform(sright, sright);
		
		sCenter.subLocal(sup).subLocal(sright);
		
		c = cpool.getColor(color.x, color.y, color.z, .8f);
		g.setColor(c);
		g.fillOval((int)sCenter.x, (int)sCenter.y, (int)sright.length()*2, (int)sup.length()*2);
		
	}

	
	// TODO change IntegerArray to a specific class for int[] arrays
	private final Vec2 temp = new Vec2();
	private final static IntArray xIntsPool = new IntArray();
	private final static IntArray yIntsPool = new IntArray();
	/**
	 * @see org.jbox2d.callbacks.DebugDraw#drawSolidPolygon(org.jbox2d.common.Vec2[], int, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
		// outside
		drawPolygon(vertices, vertexCount, color);
		
		// inside
		Graphics2D g = getGraphics();		
		int[] xInts = xIntsPool.get(vertexCount);
		int[] yInts = yIntsPool.get(vertexCount);
		
		for(int i=0; i<vertexCount; i++){
			getWorldToScreenToOut(vertices[i], temp);
			xInts[i] = (int)temp.x;
			yInts[i] = (int)temp.y;
		}
		
		Color c = cpool.getColor(color.x, color.y, color.z, .8f);
		g.setColor(c);
		g.fillPolygon(xInts, yInts, vertexCount);
	}

	/**
	 * @see org.jbox2d.callbacks.DebugDraw#drawString(float, float, java.lang.String, org.jbox2d.common.Color3f)
	 */
	@Override
	public void drawString(float x, float y, String s, Color3f color) {
		Graphics2D g = getGraphics();		
		
		Color c = cpool.getColor(color.x, color.y, color.z);
		g.setColor(c);
		g.drawString(s, x, y);
	}
	
	private Graphics2D getGraphics(){
		if(graphics == null){
			graphics = panel.dbg;
		}
		return graphics;
	}

	private final Vec2 temp2 = new Vec2();
	/**
	 * @see org.jbox2d.callbacks.DebugDraw#drawTransform(org.jbox2d.common.Transform)
	 */
	@Override
	public void drawTransform(Transform xf) {
		Graphics2D g = getGraphics();		
		getWorldToScreenToOut(xf.position, temp);
		temp2.setZero();
		float k_axisScale = 0.4f;
		
		Color c = cpool.getColor(1, 0, 0);
		g.setColor(c);
		
		temp2.x = xf.position.x + k_axisScale * xf.R.col1.x;
		temp2.y = xf.position.y + k_axisScale * xf.R.col1.y;
		getWorldToScreenToOut(temp2, temp2);
		g.drawLine((int)temp.x, (int)temp.y, (int)temp2.x, (int)temp2.y);
		
		c = cpool.getColor(0, 1, 0);
		g.setColor(c);
		temp2.x = xf.position.x + k_axisScale * xf.R.col2.x;
		temp2.y = xf.position.y + k_axisScale * xf.R.col2.y;
		getWorldToScreenToOut(temp2, temp2);
		g.drawLine((int)temp.x, (int)temp.y, (int)temp2.x, (int)temp2.y);
	}

}
