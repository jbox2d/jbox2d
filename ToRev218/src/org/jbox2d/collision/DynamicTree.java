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
	private final DynamicTreeNode[] m_nodes;
	
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
	}
	
	/**
	 * Destroy the tree, freeing the node pool.
	 */
	public void destructor(){
		
	}
	
	/**
	 * Create a proxy. Provide a tight fitting AABB and a userData pointer.
	 */
	public short createProxy(AABB aabb, Object userData){
		
	}
	
	/**
	 * Destroy a proxy. This asserts if the id is invalid.
	 */
	public void destroyProxy(short proxyId){
		
	}
	
	/**
	 * Move a proxy. If the proxy has moved outside of its fattened AABB,
	 * then the proxy is removed from the tree and re-inserted. Otherwise
	 * the function returns immediately.
	 * @param proxyId
	 * @param aabb
	 */
	public void moveProxy(short proxyId,  AABB aabb){
		
	}

	/**
	 * Perform some iterations to re-balance the tree.
	 * @param iterations
	 */
	public void rebalance(int iterations){
		
	}

	/**
	 * Get proxy user data.
	 * @param proxyId
	 * @return the proxy user data or NULL if the id is invalid.
	 */
	public void getProxy(short proxyId){
		
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
			DynamicTreeNode node = m_nodes.get(stack[--count]);
			
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
			DynamicTreeNode node = m_nodes.get( stack[--count]);
			
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
	
	private short allocateNode(){
		
	}
	
	private void freeNode(short node){
		
	}
	
	private void insertLeaf(short node){
		
	}
	
	private void removeLeaf(short node){
		
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
