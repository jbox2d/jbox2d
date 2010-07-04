/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "b2BroadPhase.h"
#include <string.h>
#include <algorithm>

// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
//   overlap query results.
// - where possible, we compare bound indices instead of values to reduce
//   cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for huge
//   worlds (use a multi-SAP instead), it is not great for large objects.

bool b2BroadPhase::s_validate = false;

static int32 BinarySearch(b2Bound[] bounds, int32 count, uint16 value){
	int32 low = 0;
	int32 high = count - 1;
	while (low <= high){
		int32 mid = (low + high) >> 1;
		if (bounds[mid].value > value){
			high = mid - 1;
		} else if (bounds[mid].value < value){
			low = mid + 1;
		} else{
			return (uint16)mid;
		}
	}
	
	return low;
}

boolean Equals(b2BufferedPair pair1, b2BufferedPair pair2){
	if (pair1.proxyId1 == pair2.proxyId1 && pair1.proxyId2 == pair2.proxyId2){
		return true;
	}

	return false;
}

boolean lessThan (b2BufferedPair pair1, b2BufferedPair pair2){
	if (pair1.proxyId1 < pair2.proxyId1)
		return true;

	if (pair1.proxyId1 == pair2.proxyId1){
		return pair1.proxyId2 < pair2.proxyId2;
	}

	return false;
}

b2BroadPhase::b2BroadPhase(b2AABB worldAABB, b2PairCallback callback){
	b2Assert(worldAABB.IsValid());
	m_worldAABB = new b2AABB(worldAABB);
	m_pairCallback = callback;
	m_proxyCount = 0;

	b2Vec2 d = b2Math.subtract(worldAABB.maxVertex, worldAABB.minVertex);
	m_quantizationFactor = new b2Vec2(USHRT_MAX / d.x, USHRT_MAX / d.y);

	b2Proxy[] m_proxyPool = new b2Proxy[b2_maxProxies];
	for (uint16 i = 0; i < b2_maxProxies - 1; ++i){
		m_proxyPool[i] = new b2Proxy();
		m_proxyPool[i].SetNext(i + 1);
		m_proxyPool[i].timeStamp = 0;
		m_proxyPool[i].overlapCount = b2_invalid;
		m_proxyPool[i].userData = null;
	}
	m_proxyPool[b2_maxProxies-1].SetNext(b2_nullProxy);
	m_proxyPool[b2_maxProxies-1].timeStamp = 0;
	m_proxyPool[b2_maxProxies-1].overlapCount = b2_invalid;
	m_proxyPool[b2_maxProxies-1].userData = NULL;
	m_freeProxy = 0;

	m_pairBufferCount = 0;

	m_timeStamp = 1;
	m_queryResultCount = 0;
}

b2BroadPhase::~b2BroadPhase(){
}

inline bool b2BroadPhase::InRange(b2AABB aabb){
	// Does the world AABB completely contain the provided AABB?
	b2Vec2 d = b2Math.b2Min( b2Math.subtract(aabb.minVertex, m_worldAABB.minVertex), b2Math.subtract(m_worldAABB.maxVertex, aabb.maxVertex));
	return b2Min(d.x, d.y) > 0.0f;
}

bool b2BroadPhase::ShouldCollide(int32 id1, int32 id2){
	b2Assert(id1 < b2_maxProxies);
	b2Assert(id2 < b2_maxProxies);
	b2Proxy p1 = m_proxyPool[id1];
	b2Proxy p2 = m_proxyPool[id2];
	if (p1.groupIndex == p2.groupIndex && p1.groupIndex != 0){
		return p1.groupIndex > 0;
	}

	boolean doCollide = (p1.maskBits & p2.categoryBits) != 0 && (p1.categoryBits & p2.maskBits) != 0;
	return doCollide;
}

bool b2BroadPhase::TestOverlap(b2Proxy p1, b2Proxy p2){
	for (int32 axis = 0; axis < 2; ++axis){
		b2Bound[] bounds = m_bounds[axis];

		if (bounds[p1.lowerBounds[axis]].value > bounds[p2.upperBounds[axis]].value)
			return false;

		if (bounds[p1.upperBounds[axis]].value < bounds[p2.lowerBounds[axis]].value)
			return false;
	}

	return true;
}

void b2BroadPhase::ComputeBounds(uint16[] lowerValues, uint16[] upperValues, b2AABB aabb){
	b2Vec2 minVertex = b2Math.b2Clamp(aabb.minVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);
	b2Vec2 maxVertex = b2Math.b2Clamp(aabb.maxVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);

	// Bump lower bounds downs and upper bounds up. This ensures correct sorting of
	// lower/upper bounds that would have equal values.
	// TODO_ERIN implement fast float to uint16 conversion.
	//Java note: bit fiddling, be careful - make sure this works after typedefs are resolved
	lowerValues[0] = (uint16)(m_quantizationFactor.x * (minVertex.x - m_worldAABB.minVertex.x)) & (USHRT_MAX - 1);
	upperValues[0] = (uint16)(m_quantizationFactor.x * (maxVertex.x - m_worldAABB.minVertex.x)) | 1;

	lowerValues[1] = (uint16)(m_quantizationFactor.y * (minVertex.y - m_worldAABB.minVertex.y)) & (USHRT_MAX - 1);
	upperValues[1] = (uint16)(m_quantizationFactor.y * (maxVertex.y - m_worldAABB.minVertex.y)) | 1;
}

void b2BroadPhase::IncrementTimeStamp(){
	if (m_timeStamp == USHRT_MAX){
		for (uint16 i = 0; i < b2_maxProxies; ++i){
			m_proxyPool[i].timeStamp = 0;
		}
		m_timeStamp = 1;
	} else{
		++m_timeStamp;
	}
}

void b2BroadPhase::IncrementOverlapCount(int32 proxyId){
	b2Proxy proxy = m_proxyPool[proxyId];
	if (proxy.timeStamp < m_timeStamp){
		proxy.timeStamp = m_timeStamp;
		proxy.overlapCount = 1;
	} else{
		proxy.overlapCount = 2;
		b2Assert(m_queryResultCount < b2_maxProxies);
		m_queryResults[m_queryResultCount] = (uint16)proxyId;
		++m_queryResultCount;
	}
}

//Java note: dammit, a set of int pointers for return values.
//Will require some tweaking at the call sites to resolve.
void b2BroadPhase::Query(int32* lowerQueryOut, int32* upperQueryOut,
					   uint16 lowerValue, uint16 upperValue,
					   b2Bound[] bounds, int32 edgeCount, int32 axis){
	
	int32 lowerQuery = BinarySearch(bounds, edgeCount, lowerValue);
	int32 upperQuery = BinarySearch(bounds, edgeCount, upperValue);

	// Easy case: lowerQuery <= lowerIndex(i) < upperQuery
	// Solution: search query range for min bounds.
	for (int32 i = lowerQuery; i < upperQuery; ++i){
		if (bounds[i].IsLower()){
			IncrementOverlapCount(bounds[i].proxyId);
		}
	}

	// Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
	// Solution: use the stabbing count to search down the bound array.
	if (lowerQuery > 0){
		int32 i = lowerQuery - 1;
		int32 s = bounds[i].stabbingCount;

		// Find the s overlaps.
		while (s > 0){
			b2Assert(i >= 0);

			if (bounds[i].IsLower()){
				b2Proxy proxy = m_proxyPool[bounds[i].proxyId];
				if (lowerQuery <= proxy.upperBounds[axis]){
					IncrementOverlapCount(bounds[i].proxyId);
					--s;
				}
			}
			--i;
		}
	}

	*lowerQueryOut = lowerQuery; //Java note: the offending base type pointer indirection...
	*upperQueryOut = upperQuery;
}

uint16 b2BroadPhase::CreateProxy(b2AABB aabb, int16 groupIndex, uint16 categoryBits, uint16 maskBits, Object userData){
	b2Assert(m_freeProxy != b2_nullProxy);
	
	if (m_freeProxy == b2_nullProxy){
		b2Assert(false);
		return b2_nullProxy;
	}

	// Flush the pair buffer
	Flush();

	uint16 proxyId = m_freeProxy;
	b2Proxy proxy = m_proxyPool[proxyId];
	m_freeProxy = proxy.GetNext();

	proxy.overlapCount = 0;
	proxy.groupIndex = groupIndex;
	proxy.categoryBits	= categoryBits;
	proxy.maskBits = maskBits;
	proxy.userData = userData;

	b2Assert(m_proxyCount < b2_maxProxies);

	int32 edgeCount = 2 * m_proxyCount;

	uint16[] lowerValues = new uint16[2];
	uint16[] upperValues = new uint16[2];
	ComputeBounds(lowerValues, upperValues, aabb);

	for (int32 axis = 0; axis < 2; ++axis){
		b2Bound[] bounds = m_bounds[axis];
		
		int32 lowerIndex, upperIndex;
		Query(&lowerIndex, &upperIndex, lowerValues[axis], upperValues[axis], bounds, edgeCount, axis); //Java note: Query() call, int pointer issue

		//Java note: I assume these are just array shifts, so we can probably achieve effect using
		//System.arraycopy(), but need to make sure in detail that I resolve this correctly.
		memmove(bounds + upperIndex + 2, bounds + upperIndex, (edgeCount - upperIndex) * sizeof(b2Bound));
		memmove(bounds + lowerIndex + 1, bounds + lowerIndex, (upperIndex - lowerIndex) * sizeof(b2Bound));

		// The upper index has increased because of the lower bound insertion.
		++upperIndex;

		// Copy in the new bounds.
		bounds[lowerIndex].value = lowerValues[axis];
		bounds[lowerIndex].proxyId = proxyId;
		bounds[upperIndex].value = upperValues[axis];
		bounds[upperIndex].proxyId = proxyId;

		bounds[lowerIndex].stabbingCount = lowerIndex == 0 ? 0 : bounds[lowerIndex-1].stabbingCount;
		bounds[upperIndex].stabbingCount = bounds[upperIndex-1].stabbingCount;

		// Adjust the stabbing count between the new bounds.
		for (int32 index = lowerIndex; index < upperIndex; ++index){
			++bounds[index].stabbingCount;
		}

		// Adjust the all the affected bound indices.
		for (int32 index = lowerIndex; index < edgeCount + 2; ++index){
			b2Proxy proxy = m_proxyPool[bounds[index].proxyId];
			if (bounds[index].IsLower()){
				proxy.lowerBounds[axis] = (uint16)index;
			} else{
				proxy.upperBounds[axis] = (uint16)index;
			}
		}
	}

	++m_proxyCount;

	b2Assert(m_queryResultCount < b2_maxProxies);

	// Create pairs if the AABB is in range.
	boolean inRange = InRange(aabb);
	if (inRange){
		for (int32 i = 0; i < m_queryResultCount; ++i){
			if (ShouldCollide(proxyId, m_queryResults[i]) == false){
				continue;
			}

			b2Pair pair = m_pairManager.Add(proxyId, m_queryResults[i]);
			if (pair == null){
				continue;
			}

			// The Add command may return an old pair, which should not happen here.
			b2Assert(pair.IsReceived() == false);
			pair.userData = m_pairCallback.PairAdded(proxy.userData, m_proxyPool[m_queryResults[i]].userData);
			pair.SetReceived();
		}
	}

	/*
#if defined(_DEBUG)
	if (s_validate)
	{
		Validate();
	}
#endif
	 */

	// Prepare for next query.
	m_queryResultCount = 0;
	IncrementTimeStamp();

	return proxyId;
}


STOPPED HERE

void b2BroadPhase::DestroyProxy(int32 proxyId)
{
	if (proxyId == b2_nullProxy)
	{
		b2Assert(false);
		return;
	}

	// Flush the pair buffer.
	Flush();

	b2Proxy* proxy = m_proxyPool + proxyId;
	int32 edgeCount = 2 * m_proxyCount;

	for (int32 axis = 0; axis < 2; ++axis)
	{
		b2Bound* bounds = m_bounds[axis];

		int32 lowerIndex = proxy->lowerBounds[axis];
		int32 upperIndex = proxy->upperBounds[axis];
		uint16 lowerValue = bounds[lowerIndex].value;
		uint16 upperValue = bounds[upperIndex].value;

		memmove(bounds + lowerIndex, bounds + lowerIndex + 1, (upperIndex - lowerIndex - 1) * sizeof(b2Bound));
		memmove(bounds + upperIndex-1, bounds + upperIndex + 1, (edgeCount - upperIndex - 1) * sizeof(b2Bound));

		// Fix bound indices.
		for (int32 index = lowerIndex; index < edgeCount - 2; ++index)
		{
			b2Proxy* proxy = m_proxyPool + bounds[index].proxyId;
			if (bounds[index].IsLower())
			{
				proxy->lowerBounds[axis] = (uint16)index;
			}
			else
			{
				proxy->upperBounds[axis] = (uint16)index;
			}
		}

		// Fix stabbing count.
		for (int32 index = lowerIndex; index < upperIndex - 1; ++index)
		{
			--bounds[index].stabbingCount;
		}

		// Query for pairs to be removed. lowerIndex and upperIndex are not needed.
		Query(&lowerIndex, &upperIndex, lowerValue, upperValue, bounds, edgeCount - 2, axis);
	}

	b2Assert(m_queryResultCount < b2_maxProxies);

	for (int32 i = 0; i < m_queryResultCount; ++i)
	{
		b2Assert(proxy->IsValid() && m_proxyPool[m_queryResults[i]].IsValid());

		b2Proxy* other = m_proxyPool + m_queryResults[i];
		void* pairUserData = m_pairManager.Remove(proxyId, m_queryResults[i]);
		m_pairCallback->PairRemoved(proxy->userData, other->userData, pairUserData);
	}

	// Prepare for next query.
	m_queryResultCount = 0;
	IncrementTimeStamp();

	// Invalidate the proxy.
	proxy->userData = NULL;
	proxy->overlapCount = b2_invalid;

	// Return the proxy to the pool.
	proxy->SetNext(m_freeProxy);
	m_freeProxy = (uint16)proxyId;
	--m_proxyCount;

#if defined(_DEBUG)
	if (s_validate)
	{
		Validate();
	}
#endif
}

void b2BroadPhase::MoveProxy(int32 proxyId, const b2AABB& aabb)
{
	if (proxyId == b2_nullProxy || b2_maxProxies <= proxyId)
	{
		
		return;
	}

	if (aabb.IsValid() == false)
	{
		b2Assert(false);
		return;
	}

	// If the new AABB is not in range, give up.
	bool inRange = InRange(aabb);
	if (inRange == false)
	{
		return;
	}

	int32 edgeCount = 2 * m_proxyCount;

	b2Proxy* proxy = m_proxyPool + proxyId;
	uint16 lowerValues[2], upperValues[2];
	ComputeBounds(lowerValues, upperValues, aabb);

	for (int32 axis = 0; axis < 2; ++axis)
	{
		b2Bound* bounds = m_bounds[axis];

		int32 lowerIndex = proxy->lowerBounds[axis];
		int32 upperIndex = proxy->upperBounds[axis];

		uint16 lowerValue = lowerValues[axis];
		uint16 upperValue = upperValues[axis];

		int32 deltaLower = lowerValue - bounds[lowerIndex].value;
		int32 deltaUpper = upperValue - bounds[upperIndex].value;

		bounds[lowerIndex].value = lowerValue;
		bounds[upperIndex].value = upperValue;

		//
		// Expanding adds overlaps
		//

		// Should we move the lower bound down?
		if (deltaLower < 0)
		{
			int32 index = lowerIndex;
			while (index > 0 && lowerValue < bounds[index-1].value)
			{
				b2Bound* bound = bounds + index;
				b2Bound* prevEdge = bound - 1;

				int32 prevProxyId = prevEdge->proxyId;
				b2Proxy* prevProxy = m_proxyPool + prevEdge->proxyId;

				++prevEdge->stabbingCount;

				if (prevEdge->IsUpper() == true)
				{
					if (TestOverlap(proxy, prevProxy))
					{
						AddPair(proxyId, prevProxyId);
					}

					++prevProxy->upperBounds[axis];
					++bound->stabbingCount;
				}
				else
				{
					++prevProxy->lowerBounds[axis];
					--bound->stabbingCount;
				}

				--proxy->lowerBounds[axis];
				b2Swap(*bound, *prevEdge);
				--index;
			}
		}

		// Should we move the upper bound up?
		if (deltaUpper > 0)
		{
			int32 index = upperIndex;
			while (index < edgeCount-1 && bounds[index+1].value <= upperValue)
			{
				b2Bound* bound = bounds + index;
				b2Bound* nextEdge = bound + 1;
				int32 nextProxyId = nextEdge->proxyId;
				b2Proxy* nextProxy = m_proxyPool + nextProxyId;

				++nextEdge->stabbingCount;

				if (nextEdge->IsLower() == true)
				{
					if (TestOverlap(proxy, nextProxy))
					{
						AddPair(proxyId, nextProxyId);
					}

					--nextProxy->lowerBounds[axis];
					++bound->stabbingCount;
				}
				else
				{
					--nextProxy->upperBounds[axis];
					--bound->stabbingCount;
				}

				++proxy->upperBounds[axis];
				b2Swap(*bound, *nextEdge);
				++index;
			}
		}

		//
		// Shrinking removes overlaps
		//

		// Should we move the lower bound up?
		if (deltaLower > 0)
		{
			int32 index = lowerIndex;
			while (index < edgeCount-1 && bounds[index+1].value <= lowerValue)
			{
				b2Bound* bound = bounds + index;
				b2Bound* nextEdge = bound + 1;

				int32 nextProxyId = nextEdge->proxyId;
				b2Proxy* nextProxy = m_proxyPool + nextProxyId;

				--nextEdge->stabbingCount;

				if (nextEdge->IsUpper())
				{
					RemovePair(proxyId, nextProxyId);

					--nextProxy->upperBounds[axis];
					--bound->stabbingCount;
				}
				else
				{
					--nextProxy->lowerBounds[axis];
					++bound->stabbingCount;
				}

				++proxy->lowerBounds[axis];
				b2Swap(*bound, *nextEdge);
				++index;
			}
		}

		// Should we move the upper bound down?
		if (deltaUpper < 0)
		{
			int32 index = upperIndex;
			while (index > 0 && upperValue < bounds[index-1].value)
			{
				b2Bound* bound = bounds + index;
				b2Bound* prevEdge = bound - 1;

				int32 prevProxyId = prevEdge->proxyId;
				b2Proxy* prevProxy = m_proxyPool + prevProxyId;

				--prevEdge->stabbingCount;

				if (prevEdge->IsLower() == true)
				{
					RemovePair(proxyId, prevProxyId);

					++prevProxy->lowerBounds[axis];
					--bound->stabbingCount;
				}
				else
				{
					++prevProxy->upperBounds[axis];
					++bound->stabbingCount;
				}

				--proxy->upperBounds[axis];
				b2Swap(*bound, *prevEdge);
				--index;
			}
		}
	}

#if defined(_DEBUG)
	if (s_validate)
	{
		Validate();
	}
#endif
}

// As proxies are created and moved, many pairs are created and destroyed. Even worse, the same
// pair may be added and removed multiple times in a single time step of the physics engine. To reduce
// traffic in the pair manager, we try to avoid destroying pairs in the pair manager until all pairs 
// end of the physics step is called. This is done by buffering all the RemovePair requests. AddPair
// requests are processed immediately because we need the hash table entry for quick lookup.
// 
// All user user callbacks are delay until the buffered pairs are confirmed in Flush.
// This is very important because the user callbacks may be very expensive and client logic
// may be harmed if pairs are added and removed within the same time step.

// Buffer a pair for addition.
// We may add a pair that is not in the pair manager or pair buffer.
// We may add a pair that is already in the pair manager and pair buffer.
// If the added pair is not a new pair, then it must be in the pair buffer (because RemovePair was called).
void b2BroadPhase::AddPair(int32 id1, int32 id2)
{
	b2Assert(m_proxyPool[id1].IsValid() && m_proxyPool[id2].IsValid());

	if (ShouldCollide(id1, id2) == false)
	{
		return;
	}

	b2Pair* pair = m_pairManager.Add(id1, id2);
	
	if (pair == NULL)
	{
		return;
	}

	// If this pair is not in the pair buffer ...
	if (pair->IsBuffered() == false)
	{
		// This must be a new pair.
		b2Assert(pair->IsReceived() == false);

		// If there is room in the pair buffer ...
		if (m_pairBufferCount < b2_maxPairs)
		{
			// Add it to the pair buffer.
			pair->SetBuffered();
			m_pairBuffer[m_pairBufferCount].proxyId1 = pair->proxyId1;
			m_pairBuffer[m_pairBufferCount].proxyId2 = pair->proxyId2;
			++m_pairBufferCount;
		}

		b2Assert(m_pairBufferCount <= m_pairManager.GetCount());
	}

	// Confirm this pair for the subsequent call to Flush.
	pair->ClearRemoved();

#if defined(_DEBUG)
	if (s_validate)
	{
		ValidatePairs();
	}
#endif
}

// Buffer a pair for removal.
void b2BroadPhase::RemovePair(int32 id1, int32 id2)
{
	b2Assert(m_proxyPool[id1].IsValid() && m_proxyPool[id2].IsValid());

	b2Pair* pair = m_pairManager.Find(id1, id2);

	if (pair == NULL)
	{
		return;
	}

	// If this pair is not in the pair buffer ...
	if (pair->IsBuffered() == false)
	{
		// This must be an old pair.
		b2Assert(pair->IsReceived());

		if (m_pairBufferCount < b2_maxPairs)
		{
			pair->SetBuffered();
			m_pairBuffer[m_pairBufferCount].proxyId1 = pair->proxyId1;
			m_pairBuffer[m_pairBufferCount].proxyId2 = pair->proxyId2;
			++m_pairBufferCount;
		}

		b2Assert(m_pairBufferCount <= m_pairManager.GetCount());
	}

	pair->SetRemoved();

#if defined(_DEBUG)
	if (s_validate)
	{
		ValidatePairs();
	}
#endif
}

void b2BroadPhase::Flush()
{
	int32 removeCount = 0;

	for (int32 i = 0; i < m_pairBufferCount; ++i)
	{
		b2Pair* pair = m_pairManager.Find(m_pairBuffer[i].proxyId1, m_pairBuffer[i].proxyId2);
		b2Assert(pair->IsBuffered());

		b2Proxy* proxy1 = m_proxyPool + pair->proxyId1;
		b2Proxy* proxy2 = m_proxyPool + pair->proxyId2;

		b2Assert(proxy1->IsValid());
		b2Assert(proxy2->IsValid());

		bool overlap = TestOverlap(proxy1, proxy2);

		if (pair->IsRemoved())
		{
			b2Assert(overlap == false);

			if (pair->IsReceived())
			{
				m_pairCallback->PairRemoved(proxy1->userData, proxy2->userData, pair->userData);
			}

			// Store the ids so we can actually remove the pair below.
			m_pairBuffer[removeCount].proxyId1 = pair->proxyId1;
			m_pairBuffer[removeCount].proxyId2 = pair->proxyId2;
			++removeCount;
		}
		else
		{
			b2Assert(overlap == true);
			pair->ClearBuffered();

			if (pair->IsReceived() == false)
			{
				pair->userData = m_pairCallback->PairAdded(proxy1->userData, proxy2->userData);
				pair->SetReceived();
			}
		}
	}

	for (int32 i = 0; i < removeCount; ++i)
	{
		m_pairManager.Remove(m_pairBuffer[i].proxyId1, m_pairBuffer[i].proxyId2);
	}

	m_pairBufferCount = 0;
}

int32 b2BroadPhase::Query(const b2AABB& aabb, void** userData, int32 maxCount)
{
	uint16 lowerValues[2];
	uint16 upperValues[2];
	ComputeBounds(lowerValues, upperValues, aabb);

	int32 lowerIndex, upperIndex;

	Query(&lowerIndex, &upperIndex, lowerValues[0], upperValues[0], m_bounds[0], 2*m_proxyCount, 0);
	Query(&lowerIndex, &upperIndex, lowerValues[1], upperValues[1], m_bounds[1], 2*m_proxyCount, 1);

	b2Assert(m_queryResultCount < b2_maxProxies);

	int32 count = 0;
	for (int32 i = 0; i < m_queryResultCount && count < maxCount; ++i, ++count)
	{
		b2Assert(m_queryResults[i] < b2_maxProxies);
		b2Proxy* proxy = m_proxyPool + m_queryResults[i];
		b2Assert(proxy->IsValid());
		userData[i] = proxy->userData;
	}

	// Prepare for next query.
	m_queryResultCount = 0;
	IncrementTimeStamp();

	return count;
}

void b2BroadPhase::Validate()
{
	for (int32 axis = 0; axis < 2; ++axis)
	{
		b2Bound* bounds = m_bounds[axis];

		int32 pointCount = 2 * m_proxyCount;
		uint16 stabbingCount = 0;

		for (int32 i = 0; i < pointCount; ++i)
		{
			b2Bound* bound = bounds + i;
			if (i > 0)
			{
				b2Bound* prevEdge = bound - 1;
				b2Assert(prevEdge->value <= bound->value);
			}

			uint16 proxyId = bound->proxyId;

			b2Assert(proxyId != b2_nullProxy);

			b2Proxy* proxy = m_proxyPool + bound->proxyId;

			b2Assert(proxy->IsValid());

			if (bound->IsLower() == true)
			{
				b2Assert(proxy->lowerBounds[axis] == i);
				++stabbingCount;
			}
			else
			{
				b2Assert(proxy->upperBounds[axis] == i);
				--stabbingCount;
			}

			b2Assert(bound->stabbingCount == stabbingCount);
		}
	}

	b2Pair* pairs = m_pairManager.GetPairs();
	int32 pairCount = m_pairManager.GetCount();
	b2Assert(m_pairBufferCount <= pairCount);

	std::sort(m_pairBuffer, m_pairBuffer + m_pairBufferCount);

	for (int32 i = 0; i < m_pairBufferCount; ++i)
	{
		if (i > 0)
		{
			b2Assert(Equals(m_pairBuffer[i], m_pairBuffer[i-1]) == false);
		}

		b2Pair* pair = m_pairManager.Find(m_pairBuffer[i].proxyId1, m_pairBuffer[i].proxyId2);
		b2Assert(pair->IsBuffered());

		b2Proxy* proxy1 = m_proxyPool + pair->proxyId1;
		b2Proxy* proxy2 = m_proxyPool + pair->proxyId2;

		b2Assert(proxy1->IsValid() == true);
		b2Assert(proxy2->IsValid() == true);

		bool overlap = TestOverlap(proxy1, proxy2);

		if (pair->IsRemoved() == true)
		{
			b2Assert(overlap == false);
		}
		else
		{
			b2Assert(overlap == true);
		}
	}

	for (int32 i = 0; i < pairCount; ++i)
	{
		b2Pair* pair = pairs + i;

		b2Proxy* proxy1 = m_proxyPool + pair->proxyId1;
		b2Proxy* proxy2 = m_proxyPool + pair->proxyId2;

		b2Assert(proxy1->IsValid() == true);
		b2Assert(proxy2->IsValid() == true);

		bool overlap = TestOverlap(proxy1, proxy2);

		if (pair->IsBuffered())
		{
			if (pair->IsRemoved() == true)
			{
				b2Assert(overlap == false);
			}
			else
			{
				b2Assert(overlap == true);
			}
		}
		else
		{
			b2Assert(overlap == true);
		}
	}
}

void b2BroadPhase::ValidatePairs()
{
	int32 pairCount = m_pairManager.GetCount();
	b2Assert(m_pairBufferCount <= pairCount);

	std::sort(m_pairBuffer, m_pairBuffer + m_pairBufferCount);

	for (int32 i = 0; i < m_pairBufferCount; ++i)
	{
		if (i > 0)
		{
			b2Assert(Equals(m_pairBuffer[i], m_pairBuffer[i-1]) == false);
		}

		b2Pair* pair = m_pairManager.Find(m_pairBuffer[i].proxyId1, m_pairBuffer[i].proxyId2);
		b2Assert(pair->IsBuffered());

		b2Proxy* proxy1 = m_proxyPool + pair->proxyId1;
		b2Proxy* proxy2 = m_proxyPool + pair->proxyId2;

		b2Assert(proxy1->IsValid() == true);
		b2Assert(proxy2->IsValid() == true);
	}
}
/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

#include "../Common/b2Settings.h"
#include "b2Collision.h"
#include "b2PairManager.h"
#include <limits.h>

const uint16 b2_invalid = USHRT_MAX;
const uint16 b2_nullEdge = USHRT_MAX;


struct b2Bound
{
	bool IsLower() const { return (value & 1) == 0; }
	bool IsUpper() const { return (value & 1) == 1; }

	uint16 value;
	uint16 proxyId;
	uint16 stabbingCount;
};

struct b2Proxy
{
	uint16 GetNext() const { return lowerBounds[0]; }
	void SetNext(uint16 next) { lowerBounds[0] = next; }
	bool IsValid() const { return overlapCount != b2_invalid; }

	uint16 lowerBounds[2], upperBounds[2];
	uint16 overlapCount;
	uint16 timeStamp;
	void* userData;
	uint16 categoryBits;
	uint16 maskBits;
	int16 groupIndex;
};

struct b2BufferedPair
{
	uint16 proxyId1;
	uint16 proxyId2;
};

struct b2PairCallback
{
	virtual ~b2PairCallback() {}

	// This returns the new pair user data.
	virtual void* PairAdded(void* proxyUserData1, void* proxyUserData2) = 0;

	// This should free the pair's user data.
	virtual void PairRemoved(void* proxyUserData1, void* proxyUserData2, void* pairUserData) = 0;
};

class b2BroadPhase
{
public:
	b2BroadPhase(const b2AABB& worldAABB, b2PairCallback* callback);
	~b2BroadPhase();

	// Create and destroy proxies. These call Flush first.
	uint16 CreateProxy(const b2AABB& aabb, int16 groupIndex, uint16 categoryBits, uint16 maskBits, void* userData);
	void DestroyProxy(int32 proxyId);

	// Call MoveProxy as many times as you like, then when you are done
	// call Flush to finalized the proxy pairs (for your time step).
	void MoveProxy(int32 proxyId, const b2AABB& aabb);
	void Flush();

	// Query an AABB for overlapping proxies, returns the user data and
	// the count, up to the supplied maximum count.
	int32 Query(const b2AABB& aabb, void** userData, int32 maxCount);

	void Validate();
	void ValidatePairs();

private:
	void ComputeBounds(uint16* lowerValues, uint16* upperValues, const b2AABB& aabb);

	void AddPair(int32 proxyId1, int32 proxyId2);
	void RemovePair(int32 proxyId1, int32 proxyId2);

	bool TestOverlap(b2Proxy* p1, b2Proxy* p2);

	void Query(int32* lowerIndex, int32* upperIndex, uint16 lowerValue, uint16 upperValue,
				b2Bound* edges, int32 edgeCount, int32 axis);
	void IncrementOverlapCount(int32 proxyId);
	void IncrementTimeStamp();

	bool InRange(const b2AABB& aabb);

	bool ShouldCollide(int32 id1, int32 id2);

public:
	b2PairManager m_pairManager;

	b2Proxy m_proxyPool[b2_maxProxies];
	uint16 m_freeProxy;

	b2BufferedPair m_pairBuffer[b2_maxPairs];
	int32 m_pairBufferCount;

	b2Bound m_bounds[2][2*b2_maxProxies];

	b2PairCallback* m_pairCallback;
	uint16 m_queryResults[b2_maxProxies];
	int32 m_queryResultCount;

	b2AABB m_worldAABB;
	b2Vec2 m_quantizationFactor;
	int32 m_proxyCount;
	uint16 m_timeStamp;

	static bool s_validate;
};

#endif