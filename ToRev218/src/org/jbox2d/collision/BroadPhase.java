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

package org.jbox2d.collision;

//Version: b2BroadPhase.h/.cpp rev 108->139
import org.jbox2d.collision.structs.Bound;
import org.jbox2d.collision.structs.Proxy;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;

class BoundValues {
	public final short[] lowerValues;
	public final short[] upperValues;

	public BoundValues() {
		lowerValues = new short[2];
		upperValues = new short[2];
	}
}

/**
 * This broad phase uses the Sweep and Prune algorithm as described in:
 * Collision Detection in Interactive 3D Environments by Gino van den Bergen
 * Also, some ideas, such as using integral values for fast compares comes from
 * Bullet (http:/www.bulletphysics.com).<br/>
 * <br/>
 * 
 * Notes:
 * - we use bound arrays instead of linked lists for cache coherence.
 * - we use quantized integral values for fast compares.
 * - we use short indices rather than pointers to save memory.
 * - we use a stabbing count for fast overlap queries (less than order N).
 * - we also use a time stamp on each proxy to speed up the registration of
 *   overlap query results.
 * - where possible, we compare bound indices instead of values to reduce
 *   cache misses (TODO_ERIN).
 * - no broadphase is perfect and neither is this one: it is not great for huge
 *   worlds (use a multi-SAP instead), it is not great for large objects.
 */
public class BroadPhase {
	public static final short BROADPHASE_MAX = Short.MAX_VALUE;
	public static final short INVALID = BROADPHASE_MAX;
	public static final short NULL_EDGE = BROADPHASE_MAX;

	public PairManager m_pairManager;

	public final Proxy m_proxyPool[];
	public short m_freeProxy;

	public final Bound m_bounds[][];
	
	public final short m_queryResults[];
	public final float m_querySortKeys[];
	int m_queryResultCount;

	public final AABB m_worldAABB;
	public final Vec2 m_quantizationFactor;
	public int m_proxyCount;
	short m_timeStamp;

	private static final boolean debugPrint = false;

	public static final boolean s_validate = false;

	// Dumps m_bounds array to console for debugging
	@SuppressWarnings("unused")
	private void dump() {
		for ( int i = 0; i < 10; i++) {
			System.out.printf( "bounds[ %d ] = %d, %d \n", i, m_bounds[0][i].value,
			                   m_bounds[1][i].value);
		}
	}

	public BroadPhase( final AABB worldAABB, final PairCallback callback) {
		if ( BroadPhase.debugPrint) {
			System.out.println( "BroadPhase()");
		}
		
		// array initialization
		m_proxyPool = new Proxy[Settings.maxProxies];
		m_bounds = new Bound[2][2 * Settings.maxProxies];
		m_queryResults = new short[Settings.maxProxies];
		m_querySortKeys = new float[Settings.maxProxies];

		for ( int i = 0; i < 2 * Settings.maxProxies; i++) {
			m_bounds[0][i] = new Bound();
			m_bounds[1][i] = new Bound();
		}

		m_pairManager = new PairManager();
		m_pairManager.initialize( this, callback);

		assert worldAABB.isValid();

		m_worldAABB = new AABB( worldAABB);
		m_proxyCount = 0;

		final Vec2 d = worldAABB.upperBound.sub( worldAABB.lowerBound);
		m_quantizationFactor = new Vec2( BROADPHASE_MAX / d.x, BROADPHASE_MAX / d.y);

		for (short i = 0; i < Settings.maxProxies - 1; ++i) {
			m_proxyPool[i] = new Proxy();
			m_proxyPool[i].setNext((short) (i+1));
			m_proxyPool[i].timeStamp = 0;
			m_proxyPool[i].overlapCount = BroadPhase.INVALID;
			m_proxyPool[i].userData = null;
		}

		m_proxyPool[Settings.maxProxies - 1] = new Proxy();
		m_proxyPool[Settings.maxProxies - 1].setNext( PairManager.NULL_PROXY);
		m_proxyPool[Settings.maxProxies - 1].timeStamp = 0;
		m_proxyPool[Settings.maxProxies - 1].overlapCount = BroadPhase.INVALID;
		m_proxyPool[Settings.maxProxies - 1].userData = null;
		m_freeProxy = 0;

		m_timeStamp = 1;
		m_queryResultCount = 0;
	}

	/**
	 * Use this to see if your proxy is in range. If it is not in range,
	 * it should be destroyed. Otherwise you may get O(m^2) pairs, where m
	 * is the number of proxies that are out of range.
	 * @param aabb
	 * @return
	 */
	public boolean inRange( final AABB aabb) {
		final float ax = aabb.lowerBound.x - m_worldAABB.upperBound.x;
		final float ay = aabb.lowerBound.y - m_worldAABB.upperBound.y;
		final float bx = m_worldAABB.lowerBound.x - aabb.upperBound.x;
		final float by = m_worldAABB.lowerBound.y - aabb.upperBound.y;
		final float dx = MathUtils.max( ax, bx);
		final float dy = MathUtils.max( ay, by);
		return (MathUtils.max( dx, dy) < 0.0f);
	}

	// djm pooling
	private final short[] lowerValues = new short[2];
	private final short[] upperValues = new short[2];
	private final int[] indexes = new int[2];
	private final int[] ignored = new int[2];
	// Create and destroy proxies. These call Flush first.
	/** internal */
	public int createProxy( final AABB aabb, // int groupIndex, int categoryBits, int
	                        // maskBits,
	                        final Object userData) {
		if ( BroadPhase.debugPrint) {
			System.out.println( "CreateProxy()");
		}

		assert (m_proxyCount < Settings.maxProxies);
		assert (m_freeProxy != PairManager.NULL_PROXY);

		final short proxyId = m_freeProxy;
		final Proxy proxy = m_proxyPool[proxyId];
		m_freeProxy = proxy.getNext();

		proxy.overlapCount = 0;
		proxy.userData = userData;
		// proxy.groupIndex = groupIndex;
		// proxy.categoryBits = categoryBits;
		// proxy.maskBits = maskBits;

		// assert m_proxyCount < Settings.maxProxies;

		final int boundCount = 2 * m_proxyCount;

		computeBounds( lowerValues, upperValues, aabb);

		for ( int axis = 0; axis < 2; ++axis) {
			final Bound[] bounds = m_bounds[axis];
			query( indexes, lowerValues[axis], upperValues[axis], bounds, boundCount, axis);
			final int lowerIndex = indexes[0];
			int upperIndex = indexes[1];

			// System.out.println(edgeCount + ", "+lowerValues[axis] + ",
			// "+upperValues[axis]);
			// memmove(bounds[upperIndex + 2], bounds[upperIndex],
			// (edgeCount - upperIndex) * sizeof(b2Bound));

			System.arraycopy( m_bounds[axis], upperIndex, m_bounds[axis], upperIndex + 2,
			                  boundCount - upperIndex);
			for ( int i = 0; i < boundCount - upperIndex; i++) {
				m_bounds[axis][upperIndex + 2 + i] = new Bound( m_bounds[axis][upperIndex + 2 + i]);
			}

			// memmove(bounds[lowerIndex + 1], bounds[lowerIndex],
			// (upperIndex - lowerIndex) * sizeof(b2Bound));
			// System.out.println(lowerIndex+" "+upperIndex);
			System.arraycopy( m_bounds[axis], lowerIndex, m_bounds[axis], lowerIndex + 1,
			                  upperIndex - lowerIndex);
			for ( int i = 0; i < upperIndex - lowerIndex; i++) {
				m_bounds[axis][lowerIndex + 1 + i] = new Bound( m_bounds[axis][lowerIndex + 1 + i]);
			}

			// The upper index has increased because of the lower bound
			// insertion.
			++upperIndex;

			// Copy in the new bounds.

			// if (bounds[lowerIndex] == null)
			assert (bounds[lowerIndex] != null) : "Null pointer (lower)";
			// if (bounds[upperIndex] == null)
			assert (bounds[upperIndex] != null) : "Null pointer (upper)";

			bounds[lowerIndex].value = lowerValues[axis];
			bounds[lowerIndex].proxyId = proxyId;
			bounds[upperIndex].value = upperValues[axis];
			bounds[upperIndex].proxyId = proxyId;

			bounds[lowerIndex].stabbingCount = lowerIndex == 0	? 0
			                                                  	: bounds[lowerIndex - 1].stabbingCount;
			bounds[upperIndex].stabbingCount = bounds[upperIndex - 1].stabbingCount;

			// System.out.printf("lv: %d , lid: %d, uv: %d, uid: %d
			// \n",lowerValues[axis],proxyId,upperValues[axis],proxyId);

			// Adjust the stabbing count between the new bounds.
			for ( int index = lowerIndex; index < upperIndex; ++index) {
				++bounds[index].stabbingCount;
			}

			// Adjust the all the affected bound indices.
			for ( int index = lowerIndex; index < boundCount + 2; ++index) {
				final Proxy proxyn = m_proxyPool[bounds[index].proxyId];
				if ( bounds[index].isLower()) {
					proxyn.lowerBounds[axis] = (short)index;
				}
				else {
					proxyn.upperBounds[axis] = (short)index;
				}
			}
		}

		++m_proxyCount;

		assert m_queryResultCount < Settings.maxProxies;
		// Create pairs if the AABB is in range.
		for ( int i = 0; i < m_queryResultCount; ++i) {
			assert (m_queryResults[i] < Settings.maxProxies);
			assert (m_proxyPool[m_queryResults[i]].isValid());

			m_pairManager.addBufferedPair( proxyId, m_queryResults[i]);
		}

		m_pairManager.commit();

		if ( BroadPhase.s_validate) {
			validate();
		}

		// Prepare for next query.
		m_queryResultCount = 0;
		incrementTimeStamp();

		return proxyId;
	}

	public void destroyProxy( final int proxyId) {
		assert (0 < m_proxyCount && m_proxyCount <= Settings.maxProxies);
		final Proxy proxy = m_proxyPool[proxyId];
		assert (proxy.isValid());

		final int boundCount = 2 * m_proxyCount;

		for ( int axis = 0; axis < 2; ++axis) {
			final Bound[] bounds = m_bounds[axis];

			final int lowerIndex = proxy.lowerBounds[axis];
			final int upperIndex = proxy.upperBounds[axis];
			final short lowerValue = bounds[lowerIndex].value;
			final short upperValue = bounds[upperIndex].value;

			// memmove(bounds + lowerIndex, bounds + lowerIndex + 1,
			// (upperIndex - lowerIndex - 1) * sizeof(b2Bound));
			// memmove(bounds[lowerIndex + 1], bounds[lowerIndex],
			// (upperIndex - lowerIndex) * sizeof(b2Bound));
			System.arraycopy( m_bounds[axis], lowerIndex + 1, m_bounds[axis], lowerIndex,
			                  upperIndex - lowerIndex - 1);
			for ( int i = 0; i < upperIndex - lowerIndex - 1; i++) {
				m_bounds[axis][lowerIndex + i] = new Bound(m_bounds[axis][lowerIndex + i]);
			}
			// memmove(bounds + upperIndex-1, bounds + upperIndex + 1,
			// (edgeCount - upperIndex - 1) * sizeof(b2Bound));
			System.arraycopy( m_bounds[axis], upperIndex + 1, m_bounds[axis], upperIndex - 1,
			                  boundCount - upperIndex - 1);
			for ( int i = 0; i < boundCount - upperIndex - 1; i++) {
				m_bounds[axis][upperIndex - 1 + i] = new Bound( m_bounds[axis][upperIndex - 1 + i]);
			}

			// Fix bound indices.
			for ( int index = lowerIndex; index < boundCount - 2; ++index) {
				final Proxy proxyn = m_proxyPool[bounds[index].proxyId];
				if ( bounds[index].isLower()) {
					proxyn.lowerBounds[axis] = (short)index;
				}
				else {
					proxyn.upperBounds[axis] = (short)index;
				}
			}

			// Fix stabbing count.
			for ( int index = lowerIndex; index < upperIndex - 1; ++index) {
				--bounds[index].stabbingCount;
			}

			// Query for pairs to be removed. lowerIndex and upperIndex are not
			// needed.
			query( ignored, lowerValue, upperValue, bounds, boundCount - 2, axis);
		}

		assert (m_queryResultCount < Settings.maxProxies);

		for ( int i = 0; i < m_queryResultCount; ++i) {
			assert (m_proxyPool[m_queryResults[i]].isValid());
			m_pairManager.removeBufferedPair( proxyId, m_queryResults[i]);
		}

		m_pairManager.commit();

		// Prepare for next query.
		m_queryResultCount = 0;
		incrementTimeStamp();

		// Return the proxy to the pool.
		proxy.userData = null;
		proxy.overlapCount = BroadPhase.INVALID;
		proxy.lowerBounds[0] = BroadPhase.INVALID;
		proxy.lowerBounds[1] = BroadPhase.INVALID;
		proxy.upperBounds[0] = BroadPhase.INVALID;
		proxy.upperBounds[1] = BroadPhase.INVALID;

		// Return the proxy to the pool.
		proxy.setNext( m_freeProxy);
		m_freeProxy = (short)proxyId;
		--m_proxyCount;

		if ( BroadPhase.s_validate) {
			validate();
		}
	}

	private final BoundValues newValues = new BoundValues();
	private final BoundValues oldValues = new BoundValues();

	// Call MoveProxy as many times as you like, then when you are done
	// call Flush to finalized the proxy pairs (for your time step).
	/** internal */
	public void moveProxy( final int proxyId, final AABB aabb) {
		if ( BroadPhase.debugPrint) {
			System.out.println( "MoveProxy()");
		}

		if ( proxyId == PairManager.NULL_PROXY || Settings.maxProxies <= proxyId) {
			assert(false);
			return;
		}

		assert (aabb.isValid()) : "invalid AABB";

		final int boundCount = 2 * m_proxyCount;

		final Proxy proxy = m_proxyPool[proxyId];

		// Get new bound values
		computeBounds( newValues.lowerValues, newValues.upperValues, aabb);

		// Get old bound values
		for ( int axis = 0; axis < 2; ++axis) {
			oldValues.lowerValues[axis] = m_bounds[axis][proxy.lowerBounds[axis]].value;
			oldValues.upperValues[axis] = m_bounds[axis][proxy.upperBounds[axis]].value;
		}

		for ( int axis = 0; axis < 2; ++axis) {
			final Bound[] bounds = m_bounds[axis];

			final int lowerIndex = proxy.lowerBounds[axis];
			final int upperIndex = proxy.upperBounds[axis];

			final short lowerValue = newValues.lowerValues[axis];
			final short upperValue = newValues.upperValues[axis];

			final int deltaLower = lowerValue - bounds[lowerIndex].value;
			final int deltaUpper = upperValue - bounds[upperIndex].value;

			bounds[lowerIndex].value = lowerValue;
			bounds[upperIndex].value = upperValue;

			//
			// Expanding adds overlaps
			//

			// Should we move the lower bound down?
			if ( deltaLower < 0) {
				int index = lowerIndex;
				while ( index > 0 && lowerValue < bounds[index - 1].value) {
					final Bound bound = bounds[index];
					final Bound prevBound = bounds[index - 1];

					final int prevProxyId = prevBound.proxyId;
					final Proxy prevProxy = m_proxyPool[prevBound.proxyId];

					++prevBound.stabbingCount;

					if ( prevBound.isUpper() == true) {
						if ( testOverlap( newValues, prevProxy)) {
							m_pairManager.addBufferedPair( proxyId, prevProxyId);
						}

						++prevProxy.upperBounds[axis];
						++bound.stabbingCount;
					}
					else {
						++prevProxy.lowerBounds[axis];
						--bound.stabbingCount;
					}

					--proxy.lowerBounds[axis];

					bound.swap( prevBound);

					--index;
				}
			}

			// Should we move the upper bound up?
			if ( deltaUpper > 0) {
				int index = upperIndex;
				while ( index < boundCount - 1 && bounds[index + 1].value <= upperValue) {
					final Bound bound = bounds[index];
					final Bound nextBound = bounds[index + 1];
					final int nextProxyId = nextBound.proxyId;
					final Proxy nextProxy = m_proxyPool[nextProxyId];

					++nextBound.stabbingCount;

					if ( nextBound.isLower() == true) {
						if ( testOverlap( newValues, nextProxy)) {
							m_pairManager.addBufferedPair( proxyId, nextProxyId);
						}

						--nextProxy.lowerBounds[axis];
						++bound.stabbingCount;
					}
					else {
						--nextProxy.upperBounds[axis];
						--bound.stabbingCount;
					}

					++proxy.upperBounds[axis];

					bound.swap( nextBound);

					++index;
				}
			}

			//
			// Shrinking removes overlaps
			//

			// Should we move the lower bound up?
			if ( deltaLower > 0) {
				int index = lowerIndex;
				while ( index < boundCount - 1 && bounds[index + 1].value <= lowerValue) {
					final Bound bound = bounds[index];
					final Bound nextBound = bounds[index + 1];

					final int nextProxyId = nextBound.proxyId;
					final Proxy nextProxy = m_proxyPool[nextProxyId];

					--nextBound.stabbingCount;

					if ( nextBound.isUpper()) {
						if ( testOverlap( oldValues, nextProxy)) {
							m_pairManager.removeBufferedPair( proxyId, nextProxyId);
						}

						--nextProxy.upperBounds[axis];
						--bound.stabbingCount;
					}
					else {
						--nextProxy.lowerBounds[axis];
						++bound.stabbingCount;
					}

					++proxy.lowerBounds[axis];

					bound.swap( nextBound);

					++index;
				}
			}

			// Should we move the upper bound down?
			if ( deltaUpper < 0) {
				int index = upperIndex;
				while ( index > 0 && upperValue < bounds[index - 1].value) {
					final Bound bound = bounds[index];
					final Bound prevBound = bounds[index - 1];

					final int prevProxyId = prevBound.proxyId;
					final Proxy prevProxy = m_proxyPool[prevProxyId];

					--prevBound.stabbingCount;

					if ( prevBound.isLower() == true) {
						if ( testOverlap( oldValues, prevProxy)) {
							m_pairManager.removeBufferedPair( proxyId, prevProxyId);
						}

						++prevProxy.lowerBounds[axis];
						--bound.stabbingCount;
					}
					else {
						++prevProxy.upperBounds[axis];
						++bound.stabbingCount;
					}

					--proxy.upperBounds[axis];

					bound.swap( prevBound);

					--index;
				}
			}
		}

		if ( BroadPhase.s_validate) {
			validate();
		}
	}

	public void commit() {
		m_pairManager.commit();
	}

	/**
	 * Get a single proxy. Returns null if the id is invalid.
	 * @param proxyId
	 * @return
	 */
	public Proxy getProxy( final int proxyId) {
		if ( proxyId == PairManager.NULL_PROXY || (m_proxyPool[proxyId].isValid() == false)) {
			return null;
		}
		else {
			return m_proxyPool[proxyId];
		}
	}
	
	// djm pooled, from above
	/**
	 * Query an AABB for overlapping proxies, returns the user data and the
	 * count, up to the supplied maximum count.
	 */
	public int query( final AABB aabb, Object[] userData, final int maxCount) {
		if ( BroadPhase.debugPrint) {
			System.out.println( "Query(2 args)");
		}

		computeBounds( lowerValues, upperValues, aabb);

		query( indexes, lowerValues[0], upperValues[0], m_bounds[0], 2 * m_proxyCount, 0);
		query( indexes, lowerValues[1], upperValues[1], m_bounds[1], 2 * m_proxyCount, 1);

		assert m_queryResultCount < Settings.maxProxies;


		int count = 0;
		for ( int i = 0; i < m_queryResultCount && count < maxCount; ++i, ++count) {
			assert m_queryResults[i] < Settings.maxProxies;
			final Proxy proxy = m_proxyPool[m_queryResults[i]];
			proxy.isValid();
			userData[i] = proxy.userData;
		}


		// Prepare for next query.
		m_queryResultCount = 0;
		incrementTimeStamp();

		return count;
	}

	// djm pooled
	private final short[] startValues = new short[2];
	private final short[] startValues2 = new short[2];
	/**
	 * Query a segment for overlapping proxies, returns the user data and
	 * the count, up to the supplied maximum count.
	 * If sortKey is provided, then it is a function mapping from proxy userDatas to distances along the segment (between 0 & 1)
	 * Then the returned proxies are sorted on that, before being truncated to maxCount
	 * The sortKey of a proxy is assumed to be larger than the closest point inside the proxy along the segment, this allows for early exits
	 * Proxies with a negative sortKey are discarded
	 * @param segment
	 * @param userData
	 * @param maxCount
	 * @param sortKey
	 * @return
	 */
	public final int querySegment(Segment segment, Object[] userData, int maxCount, ISortKey sortKey){
		float maxLambda = 1;

		float dx = (segment.p2.x-segment.p1.x)*m_quantizationFactor.x;
		float dy = (segment.p2.y-segment.p1.y)*m_quantizationFactor.y;

		int sx = dx< -Settings.EPSILON ? -1 : (dx>Settings.EPSILON ? 1 : 0);
		int sy = dy< -Settings.EPSILON ? -1 : (dy>Settings.EPSILON ? 1 : 0);

		assert(sx!=0||sy!=0);

		float p1x = (segment.p1.x-m_worldAABB.lowerBound.x)*m_quantizationFactor.x;
		float p1y = (segment.p1.y-m_worldAABB.lowerBound.y)*m_quantizationFactor.y;

		int xIndex;
		int yIndex;

		short proxyId;
		Proxy proxy;
		
		// TODO_ERIN implement fast float to short conversion.
		startValues[0] = (short) ((short)(p1x) & (BROADPHASE_MAX - 1));
		startValues2[0] = (short) ((short)(p1x) | 1);

		startValues[1] = (short) ((short)(p1y) & (BROADPHASE_MAX - 1));
		startValues2[1] = (short) ((short)(p1y) | 1);

		//First deal with all the proxies that contain segment.p1
		
		query(indexes,startValues[0],startValues2[0],m_bounds[0],2*m_proxyCount,0);
		int lowerIndex = indexes[0];
		int upperIndex = indexes[1];
		
		if(sx>=0){
			xIndex = upperIndex-1;
		}
		else{
			xIndex = lowerIndex;
		}
		
		query(indexes,startValues[1],startValues2[1],m_bounds[1],2*m_proxyCount,1);
		lowerIndex = indexes[0];
		upperIndex = indexes[1];
		
		if(sy>=0){
			yIndex = upperIndex-1;
		}
		else{
			yIndex = lowerIndex;
		}

		//If we are using sortKey, then sort what we have so far, filtering negative keys
		if(sortKey != null) {
			//Fill keys
			for(int i=0;i<m_queryResultCount;i++)
			{
				m_querySortKeys[i] = sortKey.generateKey(m_proxyPool[m_queryResults[i]].userData);
			}
			//Bubble sort keys
			//Sorting negative values to the top, so we can easily remove them
			// TODO djm: don't use bubble sort
			int i = 0;
			while(i<m_queryResultCount-1)
			{
				float a = m_querySortKeys[i];
				float b = m_querySortKeys[i+1];
				if((a<0)?(b>=0):(a>b&&b>=0))
				{
					m_querySortKeys[i+1] = a;
					m_querySortKeys[i]   = b;
					short tempValue = m_queryResults[i+1];
					m_queryResults[i+1] = m_queryResults[i];
					m_queryResults[i] = tempValue;
					i--;
					if(i==-1) i=1;
				}
				else
				{
					i++;
				}
			}
			//Skim off negative values
			while(m_queryResultCount>0 && m_querySortKeys[m_queryResultCount-1]<0){
				m_queryResultCount--;				
			}
		}

		//Now work through the rest of the segment
		while(true){
			float xProgress = 0;
			float yProgress = 0;
			//Move on to the next bound
			xIndex += (sx >= 0) ? 1 : -1;
			
			if(xIndex<0||xIndex>=m_proxyCount*2){
				break;
			}
			if(sx!=0){
				xProgress = (m_bounds[0][xIndex].value-p1x)/dx;
			}
			//Move on to the next bound
			yIndex += (sy>=0) ? 1 : -1;
			if(yIndex<0 || yIndex>=m_proxyCount*2){
				break;
			}
			if(sy!=0){
				yProgress = (m_bounds[1][yIndex].value-p1y)/dy;
			}
			
			while(true){
				if(sy == 0 || (sx != 0 && xProgress < yProgress)){
					if(xProgress>maxLambda){
						break;
					}

					//Check that we are entering a proxy, not leaving
					if( (sx>0) ? m_bounds[0][xIndex].isLower() : m_bounds[0][xIndex].isUpper()){
						//Check the other axis of the proxy
						proxyId = m_bounds[0][xIndex].proxyId;
						proxy = m_proxyPool[proxyId];
						if(sy>=0){
							if(proxy.lowerBounds[1] <= yIndex-1 && proxy.upperBounds[1] >= yIndex){
								//Add the proxy
								if(sortKey != null){
									addProxyResult(proxyId,proxy,maxCount,sortKey);
								}
								else{
									m_queryResults[m_queryResultCount] = proxyId;
									++m_queryResultCount;
								}
							}
						}
						else{
							if(proxy.lowerBounds[1] <= yIndex && proxy.upperBounds[1] >= yIndex+1){
								//Add the proxy
								if(sortKey != null){
									addProxyResult(proxyId,proxy,maxCount,sortKey);
								}
								else{
									m_queryResults[m_queryResultCount] = proxyId;
									++m_queryResultCount;
								}
							}
						}
					}

					//Early out
					if(sortKey != null && m_queryResultCount==maxCount && m_queryResultCount>0 && xProgress>m_querySortKeys[m_queryResultCount-1]){
						break;						
					}

					//Move on to the next bound
					if(sx>0){
						xIndex++;
						if(xIndex==m_proxyCount*2){
							break;
						}
					}
					else{
						xIndex--;
						if(xIndex<0){
							break;
						}
					}
					xProgress = (m_bounds[0][xIndex].value - p1x) / dx;
				}
				else{
					if(yProgress>maxLambda){
						break;
					}

					//Check that we are entering a proxy, not leaving
					if(sy>0?m_bounds[1][yIndex].isLower():m_bounds[1][yIndex].isUpper()){
						//Check the other axis of the proxy
						proxyId = m_bounds[1][yIndex].proxyId;
						proxy = m_proxyPool[proxyId];
						if(sx>=0){
							if( proxy.lowerBounds[0] <= xIndex-1 && proxy.upperBounds[0] >= xIndex){
								//Add the proxy
								if(sortKey != null){
									addProxyResult(proxyId,proxy,maxCount,sortKey);
								}
								else{
									m_queryResults[m_queryResultCount] = proxyId;
									++m_queryResultCount;
								}
							}
						}
						else{
							if(proxy.lowerBounds[0] <= xIndex && proxy.upperBounds[0] >= xIndex+1)
							{
								//Add the proxy
								if(sortKey != null){
									addProxyResult(proxyId,proxy,maxCount,sortKey);
								}
								else {
									m_queryResults[m_queryResultCount] = proxyId;
									++m_queryResultCount;
								}
							}
						}
					}

					//Early out
					if(sortKey != null && m_queryResultCount == maxCount && m_queryResultCount > 0 && yProgress>m_querySortKeys[m_queryResultCount-1]){
						break;
					}

					//Move on to the next bound
					if(sy>0) {
						yIndex++;
						if(yIndex==m_proxyCount*2){
							break;
						}
					}
					else {
						yIndex--;
						if(yIndex<0){
							break;
						}
					}
					yProgress = (m_bounds[1][yIndex].value - p1y) / dy;
				}
			}

			break;
		}

		int count = 0;
		for(int i=0;i < m_queryResultCount && count<maxCount; ++i, ++count){
			assert(m_queryResults[i] < Settings.maxProxies);
			proxy = m_proxyPool[m_queryResults[i]];
			assert(proxy.isValid());
			userData[i] = proxy.userData;
		}

		// Prepare for next query.
		m_queryResultCount = 0;
		incrementTimeStamp();
		
		return count;
	}
	
	public void validate() {
		if ( BroadPhase.debugPrint) {
			System.out.println( "Validate()");
		}

		for ( int axis = 0; axis < 2; ++axis) {
			final Bound[] bounds = m_bounds[axis];

			final int boundCount = 2 * m_proxyCount;
			int stabbingCount = 0;

			for ( int i = 0; i < boundCount; ++i) {
				final Bound bound = bounds[i];
				assert (i == 0 || bounds[i - 1].value <= bound.value);
				assert (bound.proxyId != PairManager.NULL_PROXY);
				assert (m_proxyPool[bound.proxyId].isValid());

				if ( bound.isLower() == true) {
					assert (m_proxyPool[bound.proxyId].lowerBounds[axis] == i) : (m_proxyPool[bound.proxyId].lowerBounds[axis]
					                                                                                                     + " not " + i);
					++stabbingCount;
				}
				else {
					assert (m_proxyPool[bound.proxyId].upperBounds[axis] == i);
					--stabbingCount;
				}

				assert (bound.stabbingCount == stabbingCount);
			}
		}

	}

	public final void validatePairs(){
		
	}
	
	private void computeBounds( final short[] lowerValues, final short[] upperValues, final AABB aabb) {
		if ( BroadPhase.debugPrint) {
			System.out.println( "ComputeBounds()");
		}
		assert (aabb.upperBound.x >= aabb.lowerBound.x);
		assert (aabb.upperBound.y >= aabb.lowerBound.y);

		float minVertexX = MathUtils.clamp(aabb.lowerBound.x, m_worldAABB.lowerBound.x, m_worldAABB.upperBound.x);
		float minVertexY = MathUtils.clamp(aabb.lowerBound.y, m_worldAABB.lowerBound.y, m_worldAABB.upperBound.y);
		
		float maxVertexX = MathUtils.clamp(aabb.upperBound.x, m_worldAABB.lowerBound.x, m_worldAABB.upperBound.x);
		float maxVertexY = MathUtils.clamp(aabb.upperBound.y, m_worldAABB.lowerBound.y, m_worldAABB.upperBound.y);
		
		lowerValues[0] = (short) ((short)(m_quantizationFactor.x * (minVertexX - m_worldAABB.lowerBound.x)) & (BROADPHASE_MAX - 1));
		upperValues[0] = (short) ((short) (m_quantizationFactor.x * (maxVertexX - m_worldAABB.lowerBound.x)) | 1);

		lowerValues[1] = (short) ((short)(m_quantizationFactor.y * (minVertexY - m_worldAABB.lowerBound.y)) & (BROADPHASE_MAX - 1));
		upperValues[1] = (short) ((short) (m_quantizationFactor.y * (maxVertexY - m_worldAABB.lowerBound.y)) | 1);
		// djm: TODO try this if above works
		/*final float bx = aabb.lowerBound.x < m_worldAABB.upperBound.x	? aabb.lowerBound.x
		                                                             	: m_worldAABB.upperBound.x;
		final float by = aabb.lowerBound.y < m_worldAABB.upperBound.y	? aabb.lowerBound.y
		                                                             	: m_worldAABB.upperBound.y;
		final float minVertexX = m_worldAABB.lowerBound.x > bx ? m_worldAABB.lowerBound.x : bx;
		final float minVertexY = m_worldAABB.lowerBound.y > by ? m_worldAABB.lowerBound.y : by;
		final float b1x = aabb.upperBound.x < m_worldAABB.upperBound.x ? aabb.upperBound.x
		                                                               : m_worldAABB.upperBound.x;
		final float b1y = aabb.upperBound.y < m_worldAABB.upperBound.y ? aabb.upperBound.y
		                                                               : m_worldAABB.upperBound.y;
		final float maxVertexX = m_worldAABB.lowerBound.x > b1x ? m_worldAABB.lowerBound.x : b1x;
		final float maxVertexY = m_worldAABB.lowerBound.y > b1y ? m_worldAABB.lowerBound.y : b1y;

		// System.out.printf("minV = %f %f, maxV = %f %f
		// \n",aabb.minVertex.x,aabb.minVertex.y,aabb.maxVertex.x,aabb.maxVertex.y);

		// Bump lower bounds downs and upper bounds up. This ensures correct
		// sorting of
		// lower/upper bounds that would have equal values.
		// TODO_ERIN implement fast float to int16 conversion.
		lowerValues[0] = (int) (m_quantizationFactor.x * (minVertexX - m_worldAABB.lowerBound.x))
		& (Integer.MAX_VALUE - 1);
		upperValues[0] = (int) (m_quantizationFactor.x * (maxVertexX - m_worldAABB.lowerBound.x)) | 1;

		lowerValues[1] = (int) (m_quantizationFactor.y * (minVertexY - m_worldAABB.lowerBound.y))
		& (Integer.MAX_VALUE - 1);
		upperValues[1] = (int) (m_quantizationFactor.y * (maxVertexY - m_worldAABB.lowerBound.y)) | 1;*/
	}

	// This one is only used for validation.
	protected boolean testOverlap( final Proxy p1, final Proxy p2) {
		for ( int axis = 0; axis < 2; ++axis) {
			final Bound[] bounds = m_bounds[axis];

			assert (p1.lowerBounds[axis] < 2 * m_proxyCount);
			assert (p1.upperBounds[axis] < 2 * m_proxyCount);
			assert (p2.lowerBounds[axis] < 2 * m_proxyCount);
			assert (p2.upperBounds[axis] < 2 * m_proxyCount);

			if ( bounds[p1.lowerBounds[axis]].value > bounds[p2.upperBounds[axis]].value) {
				return false;
			}

			if ( bounds[p1.upperBounds[axis]].value < bounds[p2.lowerBounds[axis]].value) {
				return false;
			}
		}

		return true;
	}

	private boolean testOverlap( final BoundValues b, final Proxy p) {
		for ( int axis = 0; axis < 2; ++axis) {
			final Bound[] bounds = m_bounds[axis];

			assert (p.lowerBounds[axis] < 2 * m_proxyCount);
			assert (p.upperBounds[axis] < 2 * m_proxyCount);

			if ( b.lowerValues[axis] > bounds[p.upperBounds[axis]].value) {
				return false;
			}

			if ( b.upperValues[axis] < bounds[p.lowerBounds[axis]].value) {
				return false;
			}
		}

		return true;
	}
	
	/**
	 * @param results
	 *            out variable
	 */
	private void query( final int[] results, final short lowerValue, final short upperValue, final Bound[] bounds,
	                    final int boundCount, final int axis) {
		if ( BroadPhase.debugPrint) {
			System.out.println( "Query(6 args)");
		}

		final int lowerQuery = BroadPhase.binarySearch( bounds, boundCount, lowerValue);
		final int upperQuery = BroadPhase.binarySearch( bounds, boundCount, upperValue);

		// Easy case: lowerQuery <= lowerIndex(i) < upperQuery
		// Solution: search query range for min bounds.
		for ( int i = lowerQuery; i < upperQuery; ++i) {
			if ( bounds[i].isLower()) {
				incrementOverlapCount( bounds[i].proxyId);
			}
		}
		// Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
		// Solution: use the stabbing count to search down the bound array.
		if ( lowerQuery > 0) {
			int i = lowerQuery - 1;
			int s = bounds[i].stabbingCount;
			// Find the s overlaps.
			while ( s != 0) {
				assert (i >= 0) : ("i = " + i + "; s = " + s);
				if ( bounds[i].isLower()) {
					final Proxy proxy = m_proxyPool[bounds[i].proxyId];
					if ( lowerQuery <= proxy.upperBounds[axis]) {
						incrementOverlapCount( bounds[i].proxyId);
						--s;
					}
				}
				--i;
			}
		}

		results[0] = lowerQuery;
		results[1] = upperQuery;
	}

	private void incrementOverlapCount( final int proxyId) {
		if ( BroadPhase.debugPrint) {
			System.out.println( "IncrementOverlapCount()");
		}

		final Proxy proxy = m_proxyPool[proxyId];
		if ( proxy.timeStamp < m_timeStamp) {
			proxy.timeStamp = m_timeStamp;
			proxy.overlapCount = 1;
		}
		else {
			proxy.overlapCount = 2;
			assert(m_queryResultCount < Settings.maxProxies);
			m_queryResults[m_queryResultCount] = (short)proxyId;
			++m_queryResultCount;
		}
	}

	private void incrementTimeStamp() {
		if ( BroadPhase.debugPrint) {
			System.out.println( "IncrementTimeStamp()");
		}

		if ( m_timeStamp == BROADPHASE_MAX) {
			for ( short i = 0; i < Settings.maxProxies; ++i) {
				m_proxyPool[i].timeStamp = 0;
			}
			m_timeStamp = 1;
		}
		else {
			++m_timeStamp;
		}
	}

	private final void addProxyResult(short proxyId, Proxy proxy, int maxCount, ISortKey sortKey){		
		float key = sortKey.generateKey(proxy.userData);
		//Filter proxies on positive keys
		if(key<0)
			return;
		//Merge the new key into the sorted list.
		//float* p = std::lower_bound(m_querySortKeys,m_querySortKeys+m_queryResultCount,key);
		int index = 0;
		while(m_querySortKeys[index] < key && index < m_queryResultCount)
			index++;
		
		if(maxCount == m_queryResultCount && index == m_queryResultCount)
			return;
		
		if(maxCount == m_queryResultCount)
			m_queryResultCount--;
		//std::copy_backward
		for(int j = m_queryResultCount+1; j > index ;--j){
			m_querySortKeys[j] = m_querySortKeys[j-1];
			m_queryResults[j]  = m_queryResults[j-1];
		}
		m_querySortKeys[index] = key;
		m_queryResults[index] = proxyId;
		m_queryResultCount++;
	}

	public final static int binarySearch(Bound[] bounds, int count, short value)
	{
		int low = 0;
		int high = count - 1;
		while (low <= high){
			int mid = (low + high) >> 1;
			if (bounds[mid].value > value){
				high = mid - 1;
			}
			else if (bounds[mid].value < value){
				low = mid + 1;
			}
			else{
				return (short)mid;
			}
		}
		
		return low;
	}
}
