package org.jbox2d.collision;

import java.util.ArrayList;

import org.jbox2d.collision.structs.RayCastInput;
import org.jbox2d.collision.structs.RayCastOutput;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;

public class DynamicTree {
	public static final short NULL_NODE = Short.MAX_VALUE;
	
	public static final int k_stackSize = 32;
	
	private short m_root;
	private DynamicTreeNode[] m_nodes;
	
	private int m_nodeCount;
	
	private short m_freeList;
	
	/// This is used incrementally traverse the tree for re-balancing.
	private int m_path;
	
	
	/**
	 * Constructing the tree initializes the node pool.
	 */
	public DynamicTree(){
		m_root = NULL_NODE;
		m_nodeCount = Math.max( Settings.nodePoolSize, 1);
		m_nodes = new DynamicTreeNode[m_nodeCount];
		
		for(int i=0; i<m_nodeCount - 1; i++){
			m_nodes[i] = new DynamicTreeNode();
			m_nodes[i].parent = (short)(i+1);
		}
		m_nodes[m_nodeCount-1] = new DynamicTreeNode();
		m_nodes[m_nodeCount-1].parent = NULL_NODE;
		m_freeList = 0;
		
		m_path = 0;
	}
	
	/**
	 * Destroy the tree, freeing the node pool.
	 */
	public void destructor(){
		
	}
	
	// djm pooled
	private static final Vec2 center = new Vec2();
	private static final Vec2 extents = new Vec2();
	/**
	 * Create a proxy in the tree as a leaf node. We return the index
	 * of the node instead of a pointer so that we can grow
	 * the node pool.
	 */
	public short createProxy(AABB aabb, Object userData){
		short node = allocateNode();
		
		// Fatten the aabb.
		aabb.getCenterToOut(center);
		aabb.getExtentsToOut(extents);
		extents.mulLocal(Settings.fatAABBFactor);
		m_nodes[node].aabb.lowerBound.set(center).subLocal(extents);
		m_nodes[node].aabb.upperBound.set(center).addLocal(extents);
		m_nodes[node].userData = userData;
		
		insertLeaf(node);
		
		return node;
	}
	
	/**
	 * Destroy a proxy. This asserts if the id is invalid.
	 */
	public void destroyProxy(short proxyId){
		assert(proxyId < m_nodeCount);
		assert(m_nodes[proxyId].isLeaf());

		removeLeaf(proxyId);
		freeNode(proxyId);
	}
	
	// djm pooling, from above
	/**
	 * Move a proxy. If the proxy has moved outside of its fattened AABB,
	 * then the proxy is removed from the tree and re-inserted. Otherwise
	 * the function returns immediately.
	 * @param proxyId
	 * @param aabb
	 */
	public void moveProxy(short proxyId,  AABB aabb){
		assert(proxyId < m_nodeCount);

		assert(m_nodes[proxyId].isLeaf());

		if (m_nodes[proxyId].aabb.contains(aabb)){
			return;
		}

		removeLeaf(proxyId);

		aabb.getCenterToOut(center);
		aabb.getExtentsToOut(extents);
		extents.mulLocal(Settings.fatAABBFactor);
		m_nodes[proxyId].aabb.lowerBound.set(center).subLocal(extents);
		m_nodes[proxyId].aabb.upperBound.set(center).addLocal(extents);

		insertLeaf(proxyId);
	}

	/**
	 * Perform some iterations to re-balance the tree.
	 * @param iterations
	 */
	public void rebalance(int iterations){
		if (m_root == NULL_NODE){
			return;
		}

		for (int i = 0; i < iterations; ++i)
		{
			short node = m_root;

			int bit = 0;
			while (m_nodes[node].isLeaf() == false){
				short children = m_nodes[node].child1;
				// REALLY not sure if this is right
				node = (short) (children + ((m_path >> bit) & 1));
				// this mods it so it only has 8 bits (from 0 to 255)
				bit = (bit + 1) & (8 * 32 - 1);
			}
			++m_path;

			removeLeaf(node);
			insertLeaf(node);
		}
	}

	/**
	 * Get proxy user data.
	 * @param proxyId
	 * @return the proxy user data or null if the id is invalid.
	 */
	public Object getProxy(short proxyId){
		if (proxyId < m_nodeCount){
			return m_nodes[proxyId].userData;
		}
		else{
			return null;
		}
	}

	// djm pooling
	private static final int[] stack = new int[k_stackSize];
	/**
	 * Query an AABB for overlapping proxies. The callback class
	 * is called for each proxy that overlaps the supplied AABB.
	 */
	public void query(QueryCallback callback,  AABB aabb){
		if( m_root  == NULL_NODE ){
			return;
		}
		
		int count = 0;
		stack[count++] = m_root;
		
		while(count > 0){
			DynamicTreeNode node = m_nodes[stack[--count]];
			
			if( AABB.testOverlap( node.aabb, aabb)){
				if(node.isLeaf()){
					callback.queryCallback( aabb, node.userData);
				}
				else{
					assert(count + 1 < k_stackSize);
					stack[count++] = node.child1;
					stack[count++] = node.child2;
				}
			}
		}
	}

	// djm pooling, and from above
	private static final Vec2 r = new Vec2();
	private static final Vec2 v = new Vec2();
	private static final Vec2 abs_v = new Vec2();
	private static final Vec2 t = new Vec2();
	private static final AABB segmentAABB = new AABB();
	private static final Vec2 c = new Vec2();
	private static final Vec2 h = new Vec2();
	private static final Vec2 temp = new Vec2();
	private static final RayCastInput subInput = new RayCastInput();
	private static final RayCastOutput output = new RayCastOutput();

	/**
	 * Ray-cast against the proxies in the tree. This relies on the callback
	 * to perform a exact ray-cast in the case were the proxy contains a shape.
	 * The callback also performs the any collision filtering. This has performance
	 * roughly equal to k * log(n), where k is the number of collisions and n is the
	 * number of proxies in the tree.
	 * @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	 * @param callback a callback class that is called for each proxy that is hit by the ray.
	 */
	public void rayCast(RayCastCallback callback,  RayCastInput input){
		if( m_root == NULL_NODE){
			return;
		}
		
		Vec2 p1 = input.p1;
		Vec2 p2 = input.p2;
		r.set( p2).subLocal( p1);
		r.normalize();
		
		// v is perpendicular to the segment.
		Vec2.crossToOut( 1.0f, r, v);
		Vec2.absToOut( v, abs_v);
		
		float maxFraction = input.maxFraction;
		// Build a bounding box for the segment.
		{
			t.set(p2).subLocal(p1).mulLocal( maxFraction);
			t.addLocal( p1);
			//Vec2 t = p1 + maxFraction * (p2 - p1);
			Vec2.minToOut( p1, t, segmentAABB.lowerBound);
			Vec2.maxToOut( p1, t, segmentAABB.upperBound);
			//segmentAABB.lowerBound = b2Min(p1, t);
			//segmentAABB.upperBound = b2Max(p1, t);
		}
		
		int count = 0;
		stack[count++] = m_root;
		
		while(count > 0){
			DynamicTreeNode node = m_nodes[stack[--count]];
			
			if( AABB.testOverlap( node.aabb, segmentAABB) == false){
				continue;
			}
			
			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			node.aabb.getCenterToOut(c);
			node.aabb.getExtentsToOut(h);
			
			temp.set(p1).subLocal( c);
			float separation = Math.abs(Vec2.dot(v, temp)) - Vec2.dot(abs_v, h);
			if (separation > 0.0f){
				continue;
			}
			
			if(node.isLeaf()){
				subInput.p1.set(input.p1);
				subInput.p2.set(input.p2);
				subInput.maxFraction = maxFraction;
				
				callback.rayCastCallback( output, subInput, node.userData);
				
				if(output.hit){
					if(output.fraction == 0f){
						return;
					}
					
					maxFraction = output.fraction;
					
					// Update segment bounding box.
					{
						t.set(p2).subLocal(p1).mulLocal( maxFraction);
						t.addLocal( p1);
						//Vec2 t = p1 + maxFraction * (p2 - p1);
						Vec2.minToOut( p1, t, segmentAABB.lowerBound);
						Vec2.maxToOut( p1, t, segmentAABB.upperBound);
					}
				}
			}
			else{
				assert(count + 1 < k_stackSize);
				stack[count++] = node.child1;
				stack[count++] = node.child2;
			}
		}
	}
	
	// Allocate a node from the pool. Grow the pool if necessary.
	private short allocateNode(){
		// Peel a node off the free list.
		if( m_freeList != NULL_NODE){
			short node = m_freeList;
			m_freeList = m_nodes[node].parent;
			m_nodes[node].parent = NULL_NODE;
			m_nodes[node].child1 = NULL_NODE;
			m_nodes[node].child2 = NULL_NODE;
			return node;
		}
		
		// The free list is empty. Rebuild a bigger pool.
		System.out.println("increasing node pool size");
		int newPoolCount = Math.min(2 * m_nodeCount, Short.MAX_VALUE - 1);
		assert(newPoolCount > m_nodeCount);
		DynamicTreeNode[] newPool = new DynamicTreeNode[newPoolCount];
		System.arraycopy(m_nodes, 0, newPool, 0, m_nodes.length);
		
		for(int i=m_nodeCount-1; i<newPoolCount-1; i++){
			newPool[i] = new DynamicTreeNode();
			newPool[i].parent = (short)(i-1);
		}
		newPool[newPoolCount-1] = new DynamicTreeNode();
		newPool[newPoolCount-1].parent = NULL_NODE;

		m_freeList = (short)m_nodeCount;
		
		m_nodes = newPool;
		m_nodeCount = newPoolCount;
		
		// Finally peel a node off the new free list
		short node = m_freeList;
		m_freeList = m_nodes[node].parent;
		return node;
	}
	
	// Return a node to the pool.
	private void freeNode(short node){
		assert(node < Short.MAX_VALUE);
		m_nodes[node].parent = m_freeList;
		m_freeList = node;
	}
	
	// djm pooling, from above
	private static final Vec2 delta1 = new Vec2();
	private static final Vec2 delta2 = new Vec2();
	
	private void insertLeaf(short leaf){
		if (m_root == NULL_NODE){
			m_root = leaf;
			m_nodes[m_root].parent = NULL_NODE;
			return;
		}

		// Find the best sibling for this node.
		m_nodes[leaf].aabb.getCenterToOut(center);
		short sibling = m_root;
		if (m_nodes[sibling].isLeaf() == false){
			
			do {
				short child1 = m_nodes[sibling].child1;
				short child2 = m_nodes[sibling].child2;

				//b2Vec2 delta1 = b2Abs(m_nodes[child1].aabb.GetCenter() - center);
				//b2Vec2 delta2 = b2Abs(m_nodes[child2].aabb.GetCenter() - center);
				m_nodes[child1].aabb.getCenterToOut(delta1);
				m_nodes[child2].aabb.getCenterToOut(delta2);
				delta1.subLocal(center).absLocal();
				delta2.subLocal(center).absLocal();
				
				
				float norm1 = delta1.x + delta1.y;
				float norm2 = delta2.x + delta2.y;

				if (norm1 < norm2){
					sibling = child1;
				}
				else{
					sibling = child2;
				}

			}
			while(m_nodes[sibling].isLeaf() == false);
		}

		// Create a parent for the siblings.
		short node1 = m_nodes[sibling].parent;
		short node2 = allocateNode(); // this is the new parent node
		m_nodes[node2].parent = node1;
		m_nodes[node2].userData = null;
		m_nodes[node2].aabb.combine(m_nodes[leaf].aabb, m_nodes[sibling].aabb);

		if (node1 != NULL_NODE){
			// we set our sibling's old parent node's reference to sibling
			// to our new parent node
			if (m_nodes[m_nodes[sibling].parent].child1 == sibling){
				m_nodes[node1].child1 = node2;
			}
			else{
				m_nodes[node1].child2 = node2;
			}

			m_nodes[node2].child1 = sibling;
			m_nodes[node2].child2 = leaf;
			m_nodes[sibling].parent = node2;
			m_nodes[leaf].parent = node2;

			// expand the parent nodes aabb to our added aabb, all the way up
			do {
				if (m_nodes[node1].aabb.contains(m_nodes[node2].aabb)){
					break;
				}

				m_nodes[node1].aabb.combine(m_nodes[m_nodes[node1].child1].aabb, m_nodes[m_nodes[node1].child2].aabb);
				node2 = node1;
				node1 = m_nodes[node1].parent;
			}
			while(node1 != NULL_NODE);
		}
		else{
			// i guess this is the case of the second node added
			m_nodes[node2].child1 = sibling;
			m_nodes[node2].child2 = leaf;
			m_nodes[sibling].parent = node2;
			m_nodes[leaf].parent = node2;
			m_root = node2;
		}
	}
	
	private void removeLeaf(short leaf){
		if (leaf == m_root){
			m_root = NULL_NODE;
			return;
		}

		short node2 = m_nodes[leaf].parent;
		short node1 = m_nodes[node2].parent;
		short sibling;
		if (m_nodes[node2].child1 == leaf){
			sibling = m_nodes[node2].child2;
		}
		else{
			sibling = m_nodes[node2].child1;
		}

		if (node1 != NULL_NODE)
		{
			// Destroy node2 and connect node1 to sibling.
			if (m_nodes[node1].child1 == node2){
				m_nodes[node1].child1 = sibling;
			}
			else{
				m_nodes[node1].child2 = sibling;
			}
			m_nodes[sibling].parent = node1;
			
			freeNode(node2);

			// Adjust ancestor bounds.
			while (node1 != NULL_NODE)
			{
				AABB oldAABB = m_nodes[node1].aabb;
				m_nodes[node1].aabb.combine(m_nodes[m_nodes[node1].child1].aabb, m_nodes[m_nodes[node1].child2].aabb);

				if (oldAABB.contains(m_nodes[node1].aabb)){
					break;
				}

				node1 = m_nodes[node1].parent;
			}
		}
		else{
			m_root = sibling;
			m_nodes[sibling].parent = NULL_NODE;
			freeNode(node2);
		}
	}
}

class DynamicTreeNode {
	public final AABB aabb = new AABB();
	public Object userData;
	public short parent;
	public short child1;
	public short child2;
	
	public void set(DynamicTreeNode node){
		this.aabb.set( node.aabb);
		userData = node.userData;
		parent = node.parent;
		child1 = node.child1;
		child2 = node.child2;
	}
	
	boolean isLeaf(){
		return child1 == DynamicTree.NULL_NODE;
	}
}
