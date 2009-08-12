package org.jbox2d.collision;

import org.jbox2d.pooling.stacks.DynamicTreeNodeStack;
import org.jbox2d.structs.collision.broadphase.QueryCallback;
import org.jbox2d.structs.collision.tree.DynamicTreeNode;
import org.jbox2d.structs.collision.tree.TreeQueryCallback;


/**
 * A dynamic tree arranges data in a binary tree to accelerate
 * queries such as volume queries and ray casts. Leafs are proxies
 * with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
 * so that the proxy AABB is bigger than the client object. This allows the client
 * object to move by small amounts without triggering a tree update.
 *
 * Nodes are pooled and relocatable, so we use node indices rather than pointers.
 *
 * @author daniel
 */
public class DynamicTree {
	
	private DynamicTreeNode m_root;
	
	private int m_nodeCount;
	private int m_nodeCapacity;
	
	public DynamicTree(){
		m_root = null;
		
		m_nodeCapacity = 32;
		m_nodeCount = 0;
	}
	
	private static final DynamicTreeNodeStack stack = new DynamicTreeNodeStack();
	
	public final void query(TreeQueryCallback callback, AABB aabb){
		assert( AABB.testOverlap( aabb, m_root.aabb));
		
	}
	
	public final DynamicTreeNode allocateNode(){
		DynamicTreeNode node = stack.get();
		node.parent = null;
		node.child1 = null;
		node.child2 = null;
		node.userData = null;
	}
		
}
