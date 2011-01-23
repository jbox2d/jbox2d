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
package org.jbox2d.collision.broadphase;

import java.util.Stack;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.callbacks.TreeCallback;
import org.jbox2d.callbacks.TreeRayCastCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.TLAABB;
import org.jbox2d.pooling.TLRayCastInput;
import org.jbox2d.pooling.TLVec2;
import org.jbox2d.pooling.arrays.Vec2Array;
import org.jbox2d.pooling.stacks.DynamicTreeNodeStack;
import org.jbox2d.pooling.stacks.Vec2Stack;
import org.jbox2d.structs.collision.RayCastInput;

/**
 * A dynamic tree arranges data in a binary tree to accelerate
 * queries such as volume queries and ray casts. Leafs are proxies
 * with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
 * so that the proxy AABB is bigger than the client object. This allows the client
 * object to move by small amounts without triggering a tree update.
 *
 * @author daniel
 */
public class DynamicTree {
	public static final int MAX_STACK_SIZE = 128;
	
	private static final DynamicTreeNodeStack stack = new DynamicTreeNodeStack();

	private DynamicTreeNode m_root;
	
	private int m_nodeCount;
	
	private DynamicTreeNode lastLeaf;
	
	private int m_insertionCount;
	
	private int m_path;
	
	public DynamicTree(){
		m_root = null;
		m_nodeCount = 0;
		m_insertionCount = 0;
		m_path = 0;
		lastLeaf = null;
	}
	
	/**
	 * Create a proxy. Provide a tight fitting AABB and a userData pointer.
	 * @param argAABB
	 * @param userData
	 * @return
	 */
	public final DynamicTreeNode createProxy( final AABB argAABB, Object argUserData){
		DynamicTreeNode proxy = allocateNode();
		
		// Fatten the aabb
		proxy.aabb.lowerBound.x = argAABB.lowerBound.x - Settings.aabbExtension;
		proxy.aabb.lowerBound.y = argAABB.lowerBound.y - Settings.aabbExtension;
		proxy.aabb.upperBound.x = argAABB.upperBound.x + Settings.aabbExtension;
		proxy.aabb.upperBound.y = argAABB.upperBound.y + Settings.aabbExtension;
		proxy.userData = argUserData;
		
		insertLeaf(proxy);
		
		int iterationCount = m_nodeCount >> 4;
		int tryCount = 0;
		int height = computeHeight();
		while(height > 64 && tryCount < 10){
			rebalance(iterationCount);
			height = computeHeight();
			++tryCount;
		}
		
		return proxy;
	}
	
	/**
	 * Destroy a proxy
	 * @param argProxy
	 */
	public final void destroyProxy(DynamicTreeNode argProxy){
		assert(argProxy != null);
		assert(argProxy.isLeaf());
		
		removeLeaf(argProxy);
		freeNode(argProxy);
	}
	
	// djm pooling
	private final Vec2 d = new Vec2();
	/**
	 * Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
	 * then the proxy is removed from the tree and re-inserted. Otherwise
	 * the function returns immediately.
	 * @return true if the proxy was re-inserted.
	 */
	public final boolean moveProxy( DynamicTreeNode argProxy, final AABB argAABB, Vec2 displacement){
		assert( argProxy != null);
		assert( argProxy.isLeaf());
		
		if( argProxy.aabb.contains(argAABB)){
			return false;
		}
		
		removeLeaf(argProxy);
		
		// Extend AABB
		argAABB.lowerBound.x -= Settings.aabbExtension;
		argAABB.lowerBound.y -= Settings.aabbExtension;
		argAABB.upperBound.x += Settings.aabbExtension;
		argAABB.upperBound.y += Settings.aabbExtension;

		// Predict AABB displacement.
		d.set(displacement).mulLocal(Settings.aabbMultiplier);
		if (d.x < 0.0f){
			argAABB.lowerBound.x += d.x;
		}
		else{
			argAABB.upperBound.x += d.x;
		}

		if (d.y < 0.0f){
			argAABB.lowerBound.y += d.y;
		}
		else{
			argAABB.upperBound.y += d.y;
		}

		argProxy.aabb.set(argAABB);
		
		insertLeaf(argProxy);
		return true;
	}
	
	/**
	 * Rebalances the tree for the given iterations.  Goes through
	 * the tree by child1-child2 order (then back to root).  If given enough
	 * iterations it will hit all the nodes.  It starts off at the last leaf
	 * that it reinserted.
	 * @param argIterations
	 */
	public final void rebalance(int argIterations){
		if(m_root == null){
			return;
		}
		DynamicTreeNode currNode, nextNode;
		for(int i=0; i< argIterations; i++){
			currNode = m_root;
			
			int bit = 0;
			while(currNode.isLeaf() == false){
				int goLeft = (m_path >> bit) & 1;
				if(goLeft == 0){
					currNode = currNode.child1;
				}else{
					currNode = currNode.child2;
				}
				bit = (bit + 1) & 31;
			}
			++m_path;
			
			removeLeaf(currNode);
			insertLeaf(currNode);
		}
	}
	
	/**
	 * Query an AABB for overlapping proxies. The callback class
	 * is called for each proxy that overlaps the supplied AABB.
	 * @param argCallback
	 * @param argAABB
	 */
	public final void query(TreeCallback argCallback, AABB argAABB){
		query(argCallback, argAABB, m_root, 1);
	}
	
	// recursive query
	// returns if to proceed
	private final boolean query(TreeCallback argCallback, AABB argAABB, DynamicTreeNode argNode, int count){
		if(argNode == null){
			return true;
		}
		
		if (AABB.testOverlap(argAABB, argNode.aabb)){
			
			if(argNode.isLeaf()){
				boolean proceed = argCallback.treeCallback(argNode);
				if( !proceed ){
					return false;
				}
			}else{
				if(count< MAX_STACK_SIZE){
					boolean proceed = query(argCallback, argAABB, argNode.child1, ++count);
					if(!proceed){
						return false;
					}
				}
				if(count< MAX_STACK_SIZE){
					boolean proceed = query(argCallback, argAABB, argNode.child2, ++count);
					if(!proceed){
						return false;
					}
				}
			}
		}
		return true;
	}
	
	// stacks because it's recursive
	private final Stack<Vec2> vec2stack = new Stack<Vec2>();
	private final AABB aabb = new AABB();
	private final RayCastInput subInput = new RayCastInput();
	// TODO replace with non-threadlocal vec2 stack
	private final Vec2 getVec2(){
		if(vec2stack.isEmpty()){
			vec2stack.push(new Vec2());
			vec2stack.push(new Vec2());
			vec2stack.push(new Vec2());
			vec2stack.push(new Vec2());
			vec2stack.push(new Vec2());
			return new Vec2();
		}
		return vec2stack.pop();
	}
	
	private final void recycleVec2(Vec2... items){
		for(Vec2 v :items){
			vec2stack.push(v);
		}
	}
	
	/**
	 * Ray-cast against the proxies in the tree. This relies on the callback
	 * to perform a exact ray-cast in the case were the proxy contains a shape.
	 * The callback also performs the any collision filtering. This has performance
	 * roughly equal to k * log(n), where k is the number of collisions and n is the
	 * number of proxies in the tree.
	 * @param argInput the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	 * @param argCallback a callback class that is called for each proxy that is hit by the ray.
	 */
	public void raycast(TreeRayCastCallback argCallback,  RayCastInput argInput){
		
		final Vec2 r = getVec2();
		final Vec2 v = getVec2();
		final Vec2 absV = getVec2();
		
		Vec2 p1 = argInput.p1;
		Vec2 p2 = argInput.p2;
		r.set(p2).subLocal(p1);
		assert(r.lengthSquared() > 0f);
		r.normalize();
		
		// v is perpendicular to the segment.
		Vec2.crossToOut(1f, r, v);
		absV.set(v).absLocal();
		
		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		
		float[] maxFraction = new float[1];
		maxFraction[0] = argInput.maxFraction;
		
		// Build a bounding box for the segment.
		final AABB segAABB = aabb;
		//b2Vec2 t = p1 + maxFraction * (p2 - p1);
		final Vec2 temp = getVec2();
		temp.set(p2).subLocal(p1).mulLocal(maxFraction[0]).addLocal(p1);
		Vec2.minToOut(p1, temp, segAABB.lowerBound);
		Vec2.maxToOut(p1, temp, segAABB.upperBound);
		
		raycast(m_root, argInput, 0, segAABB, v, p1, p2, absV, maxFraction, argCallback);
		recycleVec2(r, v, absV, temp);
	}
		
	public boolean raycast(DynamicTreeNode argNode,
			RayCastInput argInput,
			int argCount,
			AABB argSegAABB,
			Vec2 argV,
			Vec2 argP1,
			Vec2 argP2,
			Vec2 argAbs_v,
			float[] argMaxFraction,
			TreeRayCastCallback argCallback){
		// start part from c++ code that's in the while loop
		if(argNode == null){
			return false;
		}
		
		if ( AABB.testOverlap(argNode.aabb, argSegAABB) == false ){
			return false;
		}
		
		final Vec2 temp = getVec2();
		final Vec2 c = getVec2();
		final Vec2 h = getVec2();
		argNode.aabb.getCenterToOut(c);
		argNode.aabb.getExtentsToOut(h);
		
		temp.set(argP1).subLocal(c);
		float separation = MathUtils.abs( Vec2.dot(argV, temp)) - Vec2.dot(argAbs_v, h);
		
		if(separation > 0f){
			recycleVec2(temp, c, h);
			return false;
		}
		
		if( argNode.isLeaf()){
			subInput.p1.set(argInput.p1);
			subInput.p2.set(argInput.p2);
			subInput.maxFraction = argMaxFraction[0];
			
			float value = argCallback.raycastCallback( subInput, argNode);
			
			if(value == 0f){
				recycleVec2(temp, c, h);
				// The client has terminated the ray cast.
				return true;
			}
			
			if(value > 0f){
				// Update segment bounding box
				argMaxFraction[0] = value;
				temp.set(argP2).subLocal(argP1).mulLocal( value).addLocal( argP1);
				Vec2.minToOut( argP1, temp, argSegAABB.lowerBound);
				Vec2.maxToOut( argP1, temp, argSegAABB.upperBound);
			}
		}else{
			if(argCount < MAX_STACK_SIZE){
				if(raycast(argNode.child1,argInput,++argCount,argSegAABB,
							argV, argP1, argP2, argAbs_v, argMaxFraction, argCallback)){
					recycleVec2(temp, c, h);
					return true; // to stop whole raycast
				}
			}
			if(argCount < MAX_STACK_SIZE){
				if(raycast(argNode.child2,argInput,++argCount,argSegAABB,
						argV, argP1, argP2, argAbs_v, argMaxFraction, argCallback)){
					recycleVec2(temp, c, h);
					return true; // to stop whole raycast
				}
			}		
		}
		recycleVec2(temp, c, h);
		return false;
	}
	
	/**
	 * Compute the height of the tree.
	 */
	public final int computeHeight(){
		return computeHeight(m_root);
	}
	
	private final int computeHeight(DynamicTreeNode argNode){
		if(argNode == null){
			return 0;
		}
		
		assert(argNode != null);
		int height1 = computeHeight(argNode.child1);
		int height2 = computeHeight(argNode.child2);
		return 1 + MathUtils.max(height1, height2);
	}
	
	private int nodeNum = 0;
	
	private final DynamicTreeNode allocateNode(){
		DynamicTreeNode node = stack.get();
		node.parent = null;
		node.child1 = null;
		node.child2 = null;
		node.userData = null;
		// not quite 100% guarantee of no duplicates, but close enough for now
		node.key = (int)(Math.random()*1000000000);
		m_nodeCount++;
		return node;
	}
	
	/**
	 * returns a node to the pool
	 * @param argNode
	 */
	private final void freeNode(DynamicTreeNode argNode){
		assert(argNode != null);
		assert( 0 < m_nodeCount);
		stack.recycle(argNode);
		m_nodeCount--;
	}
	
	private final Vec2 center = new Vec2();
	//private static final TLVec2 tltemp = new TLVec2();
	private final Vec2 delta1 = new Vec2();
	private final Vec2 delta2 = new Vec2();


	private final void insertLeaf(DynamicTreeNode argNode){
		m_insertionCount++;
		
		if(m_root == null){
			m_root = argNode;
			argNode.parent = null;
			return;
		}
		
		// find the best sibling
		//final Vec2 temp = tltemp.get();
		
		argNode.aabb.getCenterToOut(center);
		DynamicTreeNode sibling = m_root;
		
		DynamicTreeNode child1,child2;
		if(sibling.isLeaf() == false){
			do{
				child1 = sibling.child1;
				child2 = sibling.child2;
				
				child1.aabb.getCenterToOut(delta1);
				child2.aabb.getCenterToOut(delta2);
				delta1.subLocal(center).absLocal();
				delta2.subLocal(center).absLocal();
				
				float norm1 = delta1.x + delta1.y;
				float norm2 = delta2.x + delta2.y;
				
				if(norm1 < norm2){
					sibling = child1;
				}else {
					sibling = child2;
				}
				
			} while( sibling.isLeaf() == false);
		}
		
		// Create a parent for the siblings
		DynamicTreeNode node1 = sibling.parent;
		DynamicTreeNode node2 = allocateNode();
		node2.parent = node1;
		node2.userData = null;
		node2.aabb.combine(argNode.aabb, sibling.aabb);
		
		// was that the head node?
		if(node1 != null){
			if(sibling.parent.child1 == sibling){
				node1.child1 = node2;
			}else{
				node1.child2 = node2;
			}
			
			node2.child1 = sibling;
			node2.child2 = argNode;
			sibling.parent = node2;
			argNode.parent = node2;
			
			// build the aabb's up in case we expanded them out
			do {
				if( node1.aabb.contains(node2.aabb)){
					break;
				}
				
				node1.aabb.combine(node1.child1.aabb, node1.child2.aabb);
				node2 = node1;
				node1 = node1.parent;
			}while(node1 != null);
		}else{
			node2.child1 = sibling;
			node2.child2 = argNode;
			sibling.parent = node2;
			argNode.parent = node2;
			m_root = node2;
		}
	}
	
	private final AABB oldAABB = new AABB();
	
	private final void removeLeaf(DynamicTreeNode argNode){
		if(argNode == m_root){
			m_root = null;
			
			if(lastLeaf == argNode){
				lastLeaf = null;
			}
			return;
		}
		
		DynamicTreeNode node2 = argNode.parent;
		DynamicTreeNode node1 = node2.parent;
		DynamicTreeNode sibling;
		if( node2.child1 == argNode){
			sibling = node2.child2;
		}else{
			sibling = node2.child1;
		}
		
		if(node1 != null){
			// Destroy node2 and connect node1 to sibling.
			if(node1.child1 == node2){
				node1.child1 = sibling;
			}else{
				node1.child2 = sibling;
			}
			
			sibling.parent = node1;
			freeNode(node2);
			
			// Adjust ancestor bounds.  if the old one was larger, we just keep it
			while(node1 != null){
				oldAABB.set(node1.aabb);
				node1.aabb.combine(node1.child1.aabb, node1.child2.aabb);
				
				if(oldAABB.contains(node1.aabb)){
					break;
				}
				
				node1 = node1.parent;
			}
		}
		else{
			m_root = sibling;
			sibling.parent = null;
			freeNode(node2);
		}
		
		// in case we just removed our last leaf
		// (this is for rebalancing)
		if(lastLeaf == argNode){
			lastLeaf = m_root;
		}
	}

	public void drawTree(DebugDraw argDraw) {
		if(m_root == null){
			return;
		}
		int height = computeHeight();
		drawTree(argDraw, m_root, 0, height);
	}
	
	private final Color3f color = new Color3f();
	private static final Vec2Array vecs = new Vec2Array();
	private final Vec2 textVec = new Vec2();
	
	public void drawTree(DebugDraw argDraw, DynamicTreeNode argNode, int spot, int height){
		Vec2[] ray = vecs.get(4);
		argNode.aabb.getVertices(ray);
		
		color.set(1, (height-spot)*1f/height, (height-spot)*1f/height);
		argDraw.drawPolygon(ray, 4, color);
		
		argDraw.getViewportTranform().getWorldToScreen(argNode.aabb.upperBound, textVec);
		argDraw.drawString(textVec.x, textVec.y, (spot+1)+"/"+height, color);
		
		if(argNode.child1 != null){
			drawTree(argDraw, argNode.child1, spot+1, height);
		}
		if(argNode.child2 != null){
			drawTree(argDraw, argNode.child2, spot+1, height);
		}
	}
}
