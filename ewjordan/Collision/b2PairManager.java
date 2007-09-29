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

//#include "b2PairManager.h"

//#include <string.h>


// The pair manager is used by the broad-phase to quickly add/remove/find pairs
// of overlapping proxies. It is based closely on code provided by Pierre Terdiman.
// http://www.codercorner.com/IncrementalSAP.txt

//#ifndef B2_PAIR_MANAGER_H
//#define B2_PAIR_MANAGER_H

//#include "../Common/b2Settings.h"
//#include "../Common/b2Math.h"

//#include <limits.h>

//Java note: figure out what to do with global functions

// Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
// This assumes proxyId1 and proxyId2 are 16-bit.
//Java note: this was originally written in Java, so it's okay (replaced >> with >>>)
public int32 Hash(int32 proxyId1, int32 proxyId2){
	int32 key = (proxyId2 << 16) | proxyId1;
	key = ~key + (key << 15);
	key = key ^ (key >>> 12);
	key = key + (key << 2);
	key = key ^ (key >>> 4);
	key = key * 2057;
	key = key ^ (key >>> 16);
	return key;
}

public boolean Equals(b2Pair pair, int32 proxyId1, int32 proxyId2){
	return pair.proxyId1 == proxyId1 && pair.proxyId2 == proxyId2;
}




const uint16 b2_nullPair = USHRT_MAX;
const uint16 b2_nullProxy = USHRT_MAX;
const int32 b2_tableCapacity = b2_maxPairs;	// must be a power of two
const int32 b2_tableMask = b2_tableCapacity - 1;

class b2Pair{
	enum{
		e_pairBuffered = 0x0001,
		e_pairRemoved = 0x0002,
		e_pairReceived = 0x0004,
	};
	
	public b2Pair(){
		userData = null;
	}

	public void SetBuffered()		{ status |= e_pairBuffered; }
	public void ClearBuffered()	{ status &= ~e_pairBuffered; }
	public boolean IsBuffered()		{ return (status & e_pairBuffered) == e_pairBuffered; }

	public void SetRemoved()		{ status |= e_pairRemoved; }
	public void ClearRemoved()		{ status &= ~e_pairRemoved; }
	public boolean IsRemoved()		{ return (status & e_pairRemoved) == e_pairRemoved; }

	public void SetReceived()		{ status |= e_pairReceived; }
	public boolean IsReceived()		{ return (status & e_pairReceived) == e_pairReceived; }

	public Object userData;
	public uint16 proxyId1;
	public uint16 proxyId2;
	public uint16 status;
};

class b2PairManager{
	
	public b2PairManager(){
		b2Assert(b2IsPowerOfTwo(b2_tableCapacity) == true);
		b2Assert(b2_tableCapacity >= b2_maxPairs);
		m_hashTable = new uint16[b2_tableCapacity];
		m_next[b2_maxPairs] = new uint16[b2_maxPairs];
		m_pairs[b2_maxPairs] = new b2Pair[b2_maxPairs];
		for (int32 i = 0; i < b2_tableCapacity; ++i){
			m_hashTable[i] = b2_nullPair;
		}
		for (int32 i = 0; i < b2_maxPairs; ++i){
			m_next[i] = b2_nullPair;
			m_pairs[i] = new b2Pair();
		}
		m_pairCount = 0;
	}
	
	public void b2PairManagerDestructor(){
	}

	// Add a pair and return the new pair. If the pair already exists,
	// no new pair is created and the old one is returned.
	b2Pair Add(int32 proxyId1, int32 proxyId2){
		if (proxyId1 > proxyId2) {
			//b2Swap(proxyId1, proxyId2); //can't swap ints in Java like this
			int32 buff = proxyId1;
			proxyId1 = proxyId2;
			proxyId2 = buff;
		}
		
		int32 hash = Hash(proxyId1, proxyId2) & b2_tableMask;
		
		b2Pair pair = Find(proxyId1, proxyId2, hash);
		if (pair != null){
			return pair;
		}
		
		if (m_pairCount == b2_maxPairs){
			b2Assert(false);
			return null;
		}
		
		pair = m_pairs[m_pairCount];
		pair.proxyId1 = (uint16)proxyId1; //Java note: how important is this cast in the C code?
		pair.proxyId2 = (uint16)proxyId2;
		pair.status = 0;
		pair.userData = null;
		
		m_next[m_pairCount] = m_hashTable[hash];
		m_hashTable[hash] = (uint16)m_pairCount;
		
		++m_pairCount;
		
		return pair;
	}

	// Remove a pair, return the pair's userData.
	public Object Remove(int32 proxyId1, int32 proxyId2){
		if (proxyId1 > proxyId2) {
			//b2Swap(proxyId1, proxyId2); //can't swap ints in Java like this
			int32 buff = proxyId1;
			proxyId1 = proxyId2;
			proxyId2 = buff;
		}
		
		int32 hash = Hash(proxyId1, proxyId2) & b2_tableMask;
		
		b2Pair pair = Find(proxyId1, proxyId2, hash);
		if (pair == null){
			return null;
		}
		
		Object userData = pair.userData;
		
		b2Assert(pair.proxyId1 == proxyId1);
		b2Assert(pair.proxyId2 == proxyId2);
		
		int32 pairIndex = int32(pair - m_pairs); FIXME //Java note: this won't work...pointer arithmetic is being used to grab the index
			
			b2Assert(pairIndex < m_pairCount);
		
		// Remove the pair from the hash table.
		int32 index = m_hashTable[hash];
		b2Assert(index != b2_nullPair);
		
		int32 previous = b2_nullPair;
		while (index != pairIndex){
			previous = index;
			index = m_next[index];
		}
		
		if (previous != b2_nullPair){
			b2Assert(m_next[previous] == pairIndex);
			m_next[previous] = m_next[pairIndex];
		} else{
			m_hashTable[hash] = m_next[pairIndex];
		}
		
		// We now move the last pair into spot of the
		// pair being removed. We need to fix the hash
		// table indices to support the move.
		
		int32 lastPairIndex = m_pairCount - 1;
		
		// If the removed pair is the last pair, we are done.
		if (lastPairIndex == pairIndex){
			--m_pairCount;
			return userData;
		}
		
		// Remove the last pair from the hash table.
		b2Pair last = m_pairs[lastPairIndex];
		int32 lastHash = Hash(last.proxyId1, last.proxyId2) & b2_tableMask;
		
		index = m_hashTable[lastHash];
		b2Assert(index != b2_nullPair);
		
		previous = b2_nullPair;
		while (index != lastPairIndex){
			previous = index;
			index = m_next[index];
		}
		
		if (previous != b2_nullPair){
			b2Assert(m_next[previous] == lastPairIndex);
			m_next[previous] = m_next[lastPairIndex];
		} else{
			m_hashTable[lastHash] = m_next[lastPairIndex];
		}
		
		// Copy the last pair into the remove pair's spot.
		m_pairs[pairIndex] = m_pairs[lastPairIndex];
		
		// Insert the last pair into the hash table
		m_next[pairIndex] = m_hashTable[lastHash];
		m_hashTable[lastHash] = (uint16)pairIndex;
		
		--m_pairCount;
		
		return userData;
	}
	

//	b2Pair* Find(int32 proxyId1, int32 proxyId2);
	public b2Pair Find(int32 proxyId1, int32 proxyId2){
		if (proxyId1 > proxyId2) {
			//b2Swap(proxyId1, proxyId2); //can't swap ints in Java like this
			int32 buff = proxyId1;
			proxyId1 = proxyId2;
			proxyId2 = buff;
		}
		
		int32 hash = Hash(proxyId1, proxyId2) & b2_tableMask;
		
		int32 index = m_hashTable[hash];
		while (index != b2_nullPair && Equals(m_pairs[index], proxyId1, proxyId2) == false){
			index = m_next[index];
		}
		
		if (index == b2_nullPair){
			return null;
		}
		
		b2Assert(index < m_pairCount);
		
		return m_pairs[index];  //Java note: could be a problem, original returned (m_pairs + index),
								//which is likely to be used as a pointer...doesn't work in Java.
								//Might want to alter so it returns index if it is used like this.
	}

	public int32 GetCount() { return m_pairCount; }
	public b2Pair[] GetPairs() { return m_pairs; }

//	b2Pair* Find(int32 proxyId1, int32 proxyId2, uint32 hashValue);
	private b2Pair Find(int32 proxyId1, int32 proxyId2, uint32 hash){
		int32 index = m_hashTable[hash];
		
		while( index != b2_nullPair && Equals(m_pairs[index], proxyId1, proxyId2) == false){
			index = m_next[index];
		}
		
		if ( index == b2_nullPair ){
			return null;
		}
		
		b2Assert(index < m_pairCount);
		
		return m_pairs[index]; //Java note: see note in Find(int,int)
	}

	public b2Pair m_pairs[b2_maxPairs];
	public int32 m_pairCount;

	public uint16 m_hashTable[b2_tableCapacity];
	public uint16 m_next[b2_maxPairs];
};

//#endif