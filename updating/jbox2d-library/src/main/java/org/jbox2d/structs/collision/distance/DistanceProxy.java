/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.structs.collision.distance;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;

/**
 * A distance proxy is used by the GJK algorithm.
 * It encapsulates any shape.
 *
 * @author daniel
 */
public class DistanceProxy {
	public final Vec2[] m_vertices;
	public int m_count;
	public float m_radius;
	
	public DistanceProxy(){
		m_vertices = new Vec2[Settings.maxPolygonVertices];
		for(int i=0; i<m_vertices.length; i++){
			m_vertices[i] = new Vec2();
		}
		m_count = 0;
		m_radius = 0f;
	}
	
	/**
	 * Initialize the proxy using the given shape. The shape
	 * must remain in scope while the proxy is in use.
	 */
	public final void set(final Shape shape){
		switch(shape.getType()){
			case CIRCLE:
				final CircleShape circle = (CircleShape) shape;
				m_vertices[0].set(circle.m_p);
				m_count = 1;
				m_radius = circle.m_radius;
				
				break;
			case POLYGON:
				final PolygonShape poly = (PolygonShape) shape;
				m_count = poly.m_vertexCount;
				m_radius = poly.m_radius;
				for(int i=0; i<m_count; i++){
					m_vertices[i].set(poly.m_vertices[i]);
				}
				break;
			default:
				assert(false);
		}
	}
	
	/**
	 * Get the supporting vertex index in the given direction.
	 * @param d
	 * @return
	 */
	public final int getSupport(final Vec2 d){
		int bestIndex = 0;
		float bestValue = Vec2.dot(m_vertices[0], d);
		for( int i=1; i<m_count; i++){
			float value = Vec2.dot(m_vertices[i], d);
			if(value > bestValue){
				bestIndex = i;
				bestValue = value;
			}
		}
		
		return bestIndex;
	}
	
	/**
	 * Get the supporting vertex in the given direction.
	 * @param d
	 * @return
	 */
	public final Vec2 getSupportVertex(final Vec2 d){
		int bestIndex = 0;
		float bestValue = Vec2.dot(m_vertices[0], d);
		for( int i=1; i<m_count; i++){
			float value = Vec2.dot(m_vertices[i], d);
			if(value > bestValue){
				bestIndex = i;
				bestValue = value;
			}
		}
		
		return m_vertices[bestIndex];
	}
	
	/**
	 * Get the vertex count.
	 * @return
	 */
	public final int getVertexCount(){
		return m_count;
	}
	
	/**
	 * Get a vertex by index. Used by b2Distance.
	 * @param index
	 * @return
	 */
	public final Vec2 getVertex(int index){
		assert(0 <= index && index < m_count);
		return m_vertices[index];
	}
}
