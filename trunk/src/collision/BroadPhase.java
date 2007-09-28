package collision;

import common.MathUtils;
import common.Settings;
import common.Vec2;

// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
// overlap query results.
// - where possible, we compare bound indices instead of values to reduce
// cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for
// huge worlds (use a multi-SAP instead), it is not great for large objects.

public class BroadPhase {
	public static final int INVALID = Integer.MAX_VALUE;
	public static final int NULL_EDGE = Integer.MAX_VALUE;

	PairManager m_pairManager;

	Proxy m_proxyPool[];
	int m_freeProxy;

	BufferedPair m_pairBuffer[];
	int m_pairBufferCount;

	Bound m_bounds[][];

	// b2PairCallback* m_pairCallback;
	PairCallback m_pairCallback;

	int m_queryResults[];
	int m_queryResultCount;

	AABB m_worldAABB;
	Vec2 m_quantizationFactor;
	int m_proxyCount;
	int m_timeStamp;

	public BroadPhase(AABB worldAABB, PairCallback callback) {
		// array initialization
		m_proxyPool = new Proxy[Settings.maxProxies];
		m_pairBuffer = new BufferedPair[Settings.maxPairs];
		m_bounds = new Bound[2][2 * Settings.maxProxies];
		m_queryResults = new int[Settings.maxProxies];

		assert worldAABB.isValid();
		m_worldAABB = worldAABB;
		m_pairCallback = callback;
		m_proxyCount = 0;

		Vec2 d = worldAABB.maxVertex.sub(worldAABB.minVertex);
		m_quantizationFactor.x = Integer.MAX_VALUE / d.x;
		m_quantizationFactor.y = Integer.MAX_VALUE / d.y;

		for (int i = 0; i < Settings.maxProxies - 1; ++i) {
			m_proxyPool[i].SetNext(i + 1);
			m_proxyPool[i].timeStamp = 0;
			m_proxyPool[i].overlapCount = INVALID;
			m_proxyPool[i].userData = null;
		}
		m_proxyPool[Settings.maxProxies - 1].SetNext(PairManager.NULL_PROXY);
		m_proxyPool[Settings.maxProxies - 1].timeStamp = 0;
		m_proxyPool[Settings.maxProxies - 1].overlapCount = INVALID;
		m_proxyPool[Settings.maxProxies - 1].userData = null;
		m_freeProxy = 0;

		m_pairBufferCount = 0;

		m_timeStamp = 1;
		m_queryResultCount = 0;

	}

	// Create and destroy proxies. These call Flush first.
	int CreateProxy(AABB aabb, Object userData) {
		if (m_freeProxy == PairManager.NULL_PROXY) {
			return PairManager.NULL_PROXY;
		}

		// Flush the pair buffer
		Flush();

		int proxyId = m_freeProxy;
		Proxy proxy = m_proxyPool[proxyId];
		m_freeProxy = proxy.GetNext();

		proxy.overlapCount = 0;
		proxy.userData = userData;

		assert m_proxyCount < Settings.maxProxies;

		int edgeCount = 2 * m_proxyCount;

		int lowerValues[] = new int[2];
		int upperValues[] = new int[2];
		ComputeBounds(lowerValues, upperValues, aabb);

		for (int axis = 0; axis < 2; ++axis) {
			Bound[] bounds = m_bounds[axis];
			int lowerIndex, upperIndex;
			int[] indexes = Query(lowerValues[axis], upperValues[axis], bounds,
					edgeCount, axis);
			lowerIndex = indexes[0];
			upperIndex = indexes[1];

			memmove(bounds[upperIndex + 2], bounds[upperIndex],
					(edgeCount - upperIndex) * sizeof(b2Bound));
			memmove(bounds[lowerIndex + 1], bounds[lowerIndex],
					(upperIndex - lowerIndex) * sizeof(b2Bound));

			// The upper index has increased because of the lower bound
			// insertion.
			++upperIndex;

			// Copy in the new bounds.
			bounds[lowerIndex].value = lowerValues[axis];
			bounds[lowerIndex].proxyId = proxyId;
			bounds[upperIndex].value = upperValues[axis];
			bounds[upperIndex].proxyId = proxyId;

			bounds[lowerIndex].stabbingCount = lowerIndex == 0 ? 0
					: bounds[lowerIndex - 1].stabbingCount;
			bounds[upperIndex].stabbingCount = bounds[upperIndex - 1].stabbingCount;

			// Adjust the stabbing count between the new bounds.
			for (int index = lowerIndex; index < upperIndex; ++index) {
				++bounds[index].stabbingCount;
			}

			// Adjust the all the affected bound indices.
			for (int index = lowerIndex; index < edgeCount + 2; ++index) {
				Proxy proxyn = m_proxyPool[bounds[index].proxyId];
				if (bounds[index].IsLower()) {
					proxyn.lowerBounds[axis] = index;
				} else {
					proxyn.upperBounds[axis] = index;
				}
			}
		}

		++m_proxyCount;

		assert m_queryResultCount < Settings.maxProxies;

		for (int i = 0; i < m_queryResultCount; ++i) {
			Pair pair = m_pairManager.Add(proxyId, m_queryResults[i]);
			if (pair == null) {
				continue;
			}

			// The Add command may return an old pair, which should not
			// happen here.
			assert pair.userData == null;
			pair.userData = m_pairCallback.PairAdded(proxy.userData,
					m_proxyPool[m_queryResults[i]].userData);
		}

		// #if defined(_DEBUG) && B2BP_VALIDATE == 1
		// Validate();
		// #endif

		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();

		return proxyId;
	}

//	void DestroyProxy(short proxyId) {
//	}

	// Call MoveProxy as many times as you like, then when you are done
	// call Flush to finalized the proxy pairs (for your time step).
	// void MoveProxy(short proxyId, AABB aabb) {
	// }

	// public void Flush() {
	// }

	// Query an AABB for overlapping proxies, returns the user data and
	// the count, up to the supplied maximum count.
	// int Query(AABB aabb, void** userData, int maxCount){}

	// void Validate() {
	// }

	// void ValidatePairs() {
	// }

	private void ComputeBounds(int[] lowerValues, int[] upperValues, AABB aabb) {
		Vec2 minVertex = MathUtils.clamp(aabb.minVertex, m_worldAABB.minVertex,
				m_worldAABB.maxVertex);
		Vec2 maxVertex = MathUtils.clamp(aabb.maxVertex, m_worldAABB.minVertex,
				m_worldAABB.maxVertex);

		// Bump lower bounds downs and upper bounds up. This ensures correct
		// sorting of
		// lower/upper bounds that would have equal values.
		// TODO_ERIN implement fast float to uint16 conversion.
		lowerValues[0] = (int) (m_quantizationFactor.x * (minVertex.x - m_worldAABB.minVertex.x))
				& (Integer.MAX_VALUE - 1);
		upperValues[0] = (int) (m_quantizationFactor.x * (maxVertex.x - m_worldAABB.minVertex.x)) | 1;

		lowerValues[1] = (int) (m_quantizationFactor.y * (minVertex.y - m_worldAABB.minVertex.y))
				& (Integer.MAX_VALUE - 1);
		upperValues[1] = (int) (m_quantizationFactor.y * (maxVertex.y - m_worldAABB.minVertex.y)) | 1;
	}

	// private void AddPair(int proxyId1, int proxyId2);
	// private void RemovePair(int proxyId1, int proxyId2);

	private boolean TestOverlap(Proxy p1, Proxy p2) {
		for (int axis = 0; axis < 2; ++axis) {
			Bound[] bounds = m_bounds[axis];

			if (bounds[p1.lowerBounds[axis]].value > bounds[p2.upperBounds[axis]].value)
				return false;

			if (bounds[p1.upperBounds[axis]].value < bounds[p2.lowerBounds[axis]].value)
				return false;
		}

		return true;
	}

	/**
	 * return [lowerQuery, upperQuery]
	 */
	private int[] Query(int lowerValue, int upperValue, Bound[] bounds,
			int edgeCount, int axis) {
		int lowerQuery = BinarySearch(bounds, edgeCount, lowerValue);
		int upperQuery = BinarySearch(bounds, edgeCount, upperValue);

		// Easy case: lowerQuery <= lowerIndex(i) < upperQuery
		// Solution: search query range for min bounds.
		for (int i = lowerQuery; i < upperQuery; ++i) {
			if (bounds[i].IsLower()) {
				IncrementOverlapCount(bounds[i].proxyId);
			}
		}

		// Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
		// Solution: use the stabbing count to search down the bound array.
		if (lowerQuery > 0) {
			int i = lowerQuery - 1;
			int s = bounds[i].stabbingCount;

			// Find the s overlaps.
			while (s != 0) {
				assert i >= 0;

				if (bounds[i].IsLower()) {
					Proxy proxy = m_proxyPool[bounds[i].proxyId];
					if (lowerQuery <= proxy.upperBounds[axis]) {
						IncrementOverlapCount(bounds[i].proxyId);
						--s;
					}
				}
				--i;
			}
		}

		return new int[] { lowerQuery, upperQuery };
	}

	private void IncrementOverlapCount(int proxyId) {
		Proxy proxy = m_proxyPool[proxyId];
		if (proxy.timeStamp < m_timeStamp) {
			proxy.timeStamp = m_timeStamp;
			proxy.overlapCount = 1;
		} else {
			proxy.overlapCount = 2;
			assert m_queryResultCount < Settings.maxProxies;
			m_queryResults[m_queryResultCount] = proxyId;
			++m_queryResultCount;
		}
	}

	private void IncrementTimeStamp() {
		if (m_timeStamp == Integer.MAX_VALUE) {
			for (int i = 0; i < Settings.maxProxies; ++i) {
				m_proxyPool[i].timeStamp = 0;
			}
			m_timeStamp = 1;
		} else {
			++m_timeStamp;
		}
	}

	static int BinarySearch(Bound[] bounds, int count, int value) {
		int low = 0;
		int high = count - 1;
		while (low <= high) {
			int mid = (low + high) >> 1;
			if (bounds[mid].value > value) {
				high = mid - 1;
			} else if (bounds[mid].value < value) {
				low = mid + 1;
			} else {
				return mid;
			}
		}

		return low;
	}

	// TODO move this as into BufferedPair
	boolean Equals(BufferedPair pair1, BufferedPair pair2) {
		return pair1.proxyId1 == pair2.proxyId1
				&& pair1.proxyId2 == pair2.proxyId2;
	}

	// TODO move this as into BufferedPair (implements Comparable)
	boolean minor(BufferedPair pair1, BufferedPair pair2) {
		if (pair1.proxyId1 < pair2.proxyId1)
			return true;

		if (pair1.proxyId1 == pair2.proxyId1) {
			return pair1.proxyId2 < pair2.proxyId2;
		}

		return false;
	}
}
