package org.jbox2d.collision.broadphase;

import java.util.Arrays;

import org.jbox2d.collision.AABB;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.stacks.PairStack;
import org.jbox2d.structs.collision.RayCastCallback;
import org.jbox2d.structs.collision.RayCastInput;
import org.jbox2d.structs.collision.broadphase.Pair;
import org.jbox2d.structs.collision.broadphase.PairCallback;
import org.jbox2d.structs.collision.broadphase.QueryCallback;

// updated to rev 100
/**
 * The broad-phase is used for computing pairs and performing volume queries and ray casts.
 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
 * It is up to the client to consume the new pairs and to track subsequent overlap.
 *
 * @author Daniel Murphy
 */
public class BroadPhase implements QueryCallback{
	public static final int NULL_PROXY = -1;
	
	private final DynamicTree m_tree;
	
	private int m_proxyCount;
	
	private DynamicTreeNode[] m_moveBuffer;
	private int	m_moveCapacity;
	private int m_moveCount;
	
	private Pair[] m_pairBuffer;
	private int m_pairCapacity;
	private int m_pairCount;
	
	private DynamicTreeNode m_queryProxy;
	
	public BroadPhase(){
		m_proxyCount = 0;
		
		m_pairCapacity = 16;
		m_pairCount = 0;
		m_pairBuffer = new Pair[m_pairCapacity];
		for(int i=0; i<m_pairCapacity; i++){
			m_pairBuffer[i] = new Pair();
		}
		
		m_moveCapacity = 16;
		m_moveCount = 0;
		m_moveBuffer = new DynamicTreeNode[m_moveCapacity];
		
		m_tree = new DynamicTree();
		m_queryProxy = null;
	}
	/**
	 * Create a proxy with an initial AABB. Pairs are not reported until
	 * updatePairs is called.
	 * @param aabb
	 * @param userData
	 * @return
	 */
	public final DynamicTreeNode createProxy(final AABB aabb, Object userData){
		DynamicTreeNode node = m_tree.createProxy(aabb, userData);
		++m_proxyCount;
		bufferMove(node);
		return node;
	}
	
	/**
	 * Destroy a proxy. It is up to the client to remove any pairs.
	 * @param proxyId
	 */
	public final void destroyProxy(DynamicTreeNode proxy){
		unbufferMove(proxy);
		--m_proxyCount;
		m_tree.destroyProxy(proxy);
	}
	/**
	 * Call MoveProxy as many times as you like, then when you are done
	 * call UpdatePairs to finalized the proxy pairs (for your time step).
	 */
	public final void moveProxy(DynamicTreeNode proxy, final AABB aabb, final Vec2 displacement){
		boolean buffer = m_tree.moveProxy(proxy, aabb, displacement);
		if(buffer){
			bufferMove(proxy);
		}
	}
	
	public boolean testOverlap(DynamicTreeNode proxyA, DynamicTreeNode proxyB){
		return AABB.testOverlap(proxyA.aabb, proxyB.aabb);
	}
	
	/**
	 * Get the number of proxies.
	 * @return
	 */
	public final int getProxyCount(){
		return m_proxyCount;
	}
	
	/**
	 * Update the pairs. This results in pair callbacks. This can only add pairs.
	 * @param callback
	 */
	public final void updatePairs(PairCallback callback){
		// Reset pair buffer
		m_pairCount = 0;

		// Perform tree queries for all moving proxies.
		for (int i = 0; i < m_moveCount; ++i){
			m_queryProxy = m_moveBuffer[i];
			if (m_queryProxy == null){
				continue;
			}

			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			//final AABB fatAABB = m_tree.getFatAABB(m_queryProxy);

			// Query tree, create pairs and add them pair buffer.
			m_tree.query(this, m_queryProxy.aabb);
		}

		// Reset move buffer
		m_moveCount = 0;
		
		// Sort the pair buffer to expose duplicates.
		Arrays.sort(m_pairBuffer);

		// Send the pairs back to the client.
		int i = 0;
		while (i < m_pairCount){
			Pair primaryPair = m_pairBuffer[i];
			Object userDataA = primaryPair.proxyA.userData;
			Object userDataB = primaryPair.proxyB.userData;

			callback.addPair(userDataA, userDataB);
			++i;

			// Skip any duplicate pairs.
			while (i < m_pairCount){
				Pair pair = m_pairBuffer[i];
				if (pair.proxyA != primaryPair.proxyA || pair.proxyB != primaryPair.proxyB){
					break;
				}
				++i;
			}
		}
		
		// Try to keep the tree balanced.
		m_tree.rebalance(4);
	}

	/** 
	 * Query an AABB for overlapping proxies. The callback class
	 * is called for each proxy that overlaps the supplied AABB.
	 * @param callback
	 * @param aabb
	 */
	public final void query(final QueryCallback callback, final AABB aabb){
		m_tree.query(callback, aabb);
	}
	
	/**
	 * Ray-cast against the proxies in the tree. This relies on the callback
	 * to perform a exact ray-cast in the case were the proxy contains a shape.
	 * The callback also performs the any collision filtering. This has performance
	 * roughly equal to k * log(n), where k is the number of collisions and n is the
	 * number of proxies in the tree.
	 * @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	 * @param callback a callback class that is called for each proxy that is hit by the ray.
	 */
	public final void raycast(final RayCastCallback callback, final RayCastInput input){
		m_tree.raycast(callback, input);
	}
	
	/**
	 * Compute the height of the embedded tree.
	 * @return
	 */
	public final int computeHeight(){
		return m_tree.computeHeight();
	}
	
	
	protected final void bufferMove(DynamicTreeNode node){
		if(m_moveCount == m_moveCapacity){
			DynamicTreeNode[] old = m_moveBuffer;
			m_moveCapacity *= 2;
			m_moveBuffer = new DynamicTreeNode[m_moveCapacity];
			for(int i=0; i<old.length; i++){
				m_moveBuffer[i] = old[i];
			}
		}
		
		m_moveBuffer[m_moveCount] = node;
		++m_moveCount;
	}
	
	protected final void unbufferMove(DynamicTreeNode proxy){
		for(int i=0; i < m_moveCount; i++){
			if(m_moveBuffer[i] == proxy){
				m_moveBuffer[i] = null;
				return;
			}
		}
	}
	
	private final PairStack pairStack = new PairStack();
	/**
	 * This is called from DynamicTree::query when we are gathering pairs.
	 */
	public final boolean queryCallback(DynamicTreeNode proxy){
		
		// A proxy cannot form a pair with itself.
		if (proxy == m_queryProxy){
			return true;
		}
		
		// Grow the pair buffer as needed.
		if (m_pairCount == m_pairCapacity){
			Pair[] oldBuffer = m_pairBuffer;
			m_pairCapacity *= 2;
			m_pairBuffer = new Pair[m_pairCapacity];
			for(int i=0; i<oldBuffer.length; i++){
				m_pairBuffer[i] = oldBuffer[i];
			}
			for(int i=oldBuffer.length; i<m_pairCapacity; i++){
				m_pairBuffer[i] = new Pair();
			}
		}
		
		//m_pairBuffer[m_pairCount]proxyIdA = b2Min(proxyId, m_queryProxyId);
		//m_pairBuffer[m_pairCount].proxyIdB = b2Max(proxyId, m_queryProxyId);
		
		if(proxy.key < m_queryProxy.key){
			m_pairBuffer[m_pairCount].proxyA = proxy;
			m_pairBuffer[m_pairCount].proxyB = m_queryProxy;
		}else{
			m_pairBuffer[m_pairCount].proxyA = m_queryProxy;
			m_pairBuffer[m_pairCount].proxyB = proxy;
		}
		
		++m_pairCount;
		return true;
	}
}
