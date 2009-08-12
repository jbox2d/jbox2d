package org.jbox2d.collision;

import java.util.Arrays;

import org.jbox2d.pooling.TLBroadPhaseQueryWrapper;
import org.jbox2d.structs.collision.RayCastInput;
import org.jbox2d.structs.collision.broadphase.BroadPhaseQueryWrapper;
import org.jbox2d.structs.collision.broadphase.Pair;
import org.jbox2d.structs.collision.broadphase.PairCallback;
import org.jbox2d.structs.collision.broadphase.QueryCallback;

/**
 * The broad-phase is used for computing pairs and performing volume queries and ray casts.
 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
 * It is up to the client to consume the new pairs and to track subsequent overlap.
 *
 * @author daniel
 */
public class BroadPhase {
	public static final int NULL_PROXY = -1;
	
	private final DynamicTree m_tree;
	
	private int m_proxyCount;
	
	private int[] m_moveBuffer;
	private int	m_moveCapacity;
	private int m_moveCount;
	
	private Pair[] m_pairBuffer;
	private int m_pairCapacity;
	private int m_pairCount;
	
	private int m_queryProxyId;
	
	public BroadPhase(){
		
	}
	/**
	 * Create a proxy with an initial AABB. Pairs are not reported until
	 * updatePairs is called.
	 * @param aabb
	 * @param userData
	 * @return
	 */
	public final int createProxy(final AABB aabb, Object userData){
		
	}
	
	/**
	 * Destroy a proxy. It is up to the client to remove any pairs.
	 * @param proxyId
	 */
	public final void destroyProxy(int proxyId){
		
	}
	/**
	 * Call MoveProxy as many times as you like, then when you are done
	 * call UpdatePairs to finalized the proxy pairs (for your time step).
	 * @param proxyId
	 * @param aabb
	 */
	public final void moveProxy(int proxyId, final AABB aabb){
		
	}
	
	/**
	 * Get the fat AABB for a proxy
	 * @param proxyId
	 * @return
	 */
	public final AABB getFatAABB(int proxyId){
		return m_tree.getFatAABB(proxyId);
	}
	
	/**
	 * Gets the user data for a proxy.  Returns null if the 
	 * id is invalid.
	 * @param proxyId
	 * @return
	 */
	public final Object getUserData(int proxyId){
		return m_tree.getUserData(proxyId);
	}
	
	/**
	 * Test overlap of fat AABBs
	 * @param proxyIdA
	 * @param proxyIdB
	 * @return
	 */
	public final boolean testOverlap(int proxyIdA, int proxyIdB){
		final AABB a = m_tree.getFatAABB(proxyIdA);
		final AABB b = m_tree.getFatAABB(proxyIdB);
		return AABB.testOverlap( a, b);
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
			m_queryProxyId = m_moveBuffer[i];
			if (m_queryProxyId == NULL_PROXY){
				continue;
			}

			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			final AABB fatAABB = m_tree.getFatAABB(m_queryProxyId);

			// Query tree, create pairs and add them pair buffer.
			m_tree.query(this, fatAABB);
		}

		// Reset move buffer
		m_moveCount = 0;
		
		// Sort the pair buffer to expose duplicates.
		Arrays.sort(m_pairBuffer);

		// Send the pairs back to the client.
		int i = 0;
		while (i < m_pairCount){
			Pair primaryPair = m_pairBuffer[i];
			Object userDataA = m_tree.getUserData(primaryPair.proxyIdA);
			Object userDataB = m_tree.getUserData(primaryPair.proxyIdB);

			callback.addPair(userDataA, userDataB);
			++i;

			// Skip any duplicate pairs.
			while (i < m_pairCount){
				Pair pair = m_pairBuffer[i];
				if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB){
					break;
				}
				++i;
			}
		}
	}
	
	private static final TLBroadPhaseQueryWrapper tlwrapper = new TLBroadPhaseQueryWrapper();
	/** 
	 * Query an AABB for overlapping proxies. The callback class
	 * is called for each proxy that overlaps the supplied AABB.
	 * @param callback
	 * @param aabb
	 */
	public final void query(final QueryCallback callback, final AABB aabb){
		BroadPhaseQueryWrapper wrapper = tlwrapper.get();
		wrapper.tree = m_tree;
		wrapper.callback = callback;
		
		m_tree.query(wrapper, aabb);
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
	public final void raycast(final Object callback, final RayCastInput input){
		
	}
	
	/**
	 * Compute the height of the embedded tree.
	 * @return
	 */
	public final int computeHeight(){
		
	}
	
	
	protected final void bufferMove(int proxyId){
		
	}
	
	protected final void unbufferMove(int proxyId){
		
	}
	
	public final void queryCallback(int proxyId){
		
	}
}
