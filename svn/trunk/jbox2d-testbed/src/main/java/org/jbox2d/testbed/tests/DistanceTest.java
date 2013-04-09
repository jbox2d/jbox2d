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
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.Distance.SimplexCache;
import org.jbox2d.collision.DistanceInput;
import org.jbox2d.collision.DistanceOutput;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

public class DistanceTest extends TestbedTest {
	
	Vec2 m_positionB;
	float m_angleB;
	
	Transform m_transformA;
	Transform m_transformB;
	PolygonShape m_polygonA;
	PolygonShape m_polygonB;
	
	@Override
	public String getTestName() {
		return "Distance";
	}
	
	@Override
	public void initTest(boolean argDeserialized) {
		
		input.transformA = new Transform();
		input.transformB = new Transform();
		{
			m_transformA = new Transform();
			m_transformA.setIdentity();
			m_transformA.p.set(0.0f, -0.2f);
			m_polygonA = new PolygonShape();
			m_polygonA.setAsBox(10.0f, 0.2f);
		}
		
		{
			m_positionB = new Vec2();
			m_positionB.set(12.017401f, 0.13678508f);
			m_angleB = -0.0109265f;
			
			m_transformB = new Transform();
			m_transformB.set(m_positionB, m_angleB);
			
			m_polygonB = new PolygonShape();
			m_polygonB.setAsBox(2.0f, 0.1f);
		}
		for (int i = 0; i < v.length; i++) {
			v[i] = new Vec2();
		}
	}
	
	DistanceInput input = new DistanceInput();
	SimplexCache cache = new SimplexCache();
	DistanceOutput output = new DistanceOutput();
	Color3f color = new Color3f(0.9f, 0.9f, 0.9f);
	Vec2[] v = new Vec2[Settings.maxPolygonVertices];
	Color3f c1 = new Color3f(1.0f, 0.0f, 0.0f);
	Color3f c2 = new Color3f(1.0f, 1.0f, 0.0f);
	
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		
		input.proxyA.set(m_polygonA,0);
		input.proxyB.set(m_polygonB,0);
		input.transformA.set(m_transformA);
		input.transformB.set(m_transformB);
		input.useRadii = true;
		cache.count = 0;
		getWorld().getPool().getDistance().distance(output, cache, input);
		
		addTextLine("distance = " + output.distance);
		addTextLine("iterations = " + output.iterations);
		
		{
			for (int i = 0; i < m_polygonA.m_count; ++i) {
				Transform.mulToOutUnsafe(m_transformA, m_polygonA.m_vertices[i], v[i]);
			}
			getDebugDraw().drawPolygon(v, m_polygonA.m_count, color);
			
			for (int i = 0; i < m_polygonB.m_count; ++i) {
				Transform.mulToOutUnsafe(m_transformB, m_polygonB.m_vertices[i], v[i]);
			}
			getDebugDraw().drawPolygon(v, m_polygonB.m_count, color);
		}
		
		Vec2 x1 = output.pointA;
		Vec2 x2 = output.pointB;
		
		getDebugDraw().drawPoint(x1, 4.0f, c1);
		
		getDebugDraw().drawPoint(x2, 4.0f, c2);
	}
	
	@Override
	public void keyPressed(char argKeyChar, int argKeyCode) {
		
		switch (argKeyChar) {
			case 'a' :
				m_positionB.x -= 0.1f;
				break;
			
			case 'd' :
				m_positionB.x += 0.1f;
				break;
			
			case 's' :
				m_positionB.y -= 0.1f;
				break;
			
			case 'w' :
				m_positionB.y += 0.1f;
				break;
			
			case 'q' :
				m_angleB += 0.1f * MathUtils.PI;
				break;
			
			case 'e' :
				m_angleB -= 0.1f * MathUtils.PI;
				break;
		}
		
		m_transformB.set(m_positionB, m_angleB);
	}
}
