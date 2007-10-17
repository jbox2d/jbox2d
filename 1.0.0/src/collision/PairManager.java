package collision;

import common.MathUtils;
import common.Settings;

public class PairManager {

    static final int NULL_PAIR = Integer.MAX_VALUE;

    static final int NULL_PROXY = Integer.MAX_VALUE;

    static final int TABLE_CAPACITY = Settings.maxPairs; // must be a power

    // of two
    static final int TABLE_MASK = TABLE_CAPACITY - 1;

    public Pair m_pairs[];

    public int m_pairCount;

    int m_hashTable[];

    int m_next[];

    PairManager() {
        m_pairs = new Pair[Settings.maxPairs];
        m_hashTable = new int[TABLE_CAPACITY];
        m_next = new int[Settings.maxPairs];

        assert MathUtils.isPowerOfTwo(TABLE_CAPACITY) == true;
        assert TABLE_CAPACITY >= Settings.maxPairs;

        for (int i = 0; i < TABLE_CAPACITY; ++i) {
            m_hashTable[i] = NULL_PAIR;
        }
        for (int i = 0; i < Settings.maxPairs; ++i) {
            m_next[i] = NULL_PAIR;
            m_pairs[i] = new Pair();
        }
        m_pairCount = 0;
    }

    // Add a pair and return the new pair. If the pair already exists,
    // no new pair is created and the old one is returned.
    Pair Add(int proxyId1, int proxyId2) {
        // System.out.printf("PairManager.Add(%d, %d)\n", proxyId1, proxyId2);
        if (proxyId1 > proxyId2) {
            // integer primitive swap
            proxyId1 += proxyId2;
            proxyId2 = proxyId1 - proxyId2;
            proxyId1 -= proxyId2;
        }

        int hash = Hash(proxyId1, proxyId2) & TABLE_MASK;

        Pair pair = Find(proxyId1, proxyId2, hash);
        if (pair != null) {
            return pair;
        }

        if (m_pairCount == Settings.maxPairs) {
            assert false;
            return null;
        }

        pair = m_pairs[m_pairCount];
        pair.proxyId1 = proxyId1;
        pair.proxyId2 = proxyId2;
        pair.status = 0;
        pair.userData = null;

        m_next[m_pairCount] = m_hashTable[hash];
        m_hashTable[hash] = m_pairCount;

        ++m_pairCount;

        return pair;
    }

    // Remove a pair, return the pair's userData.
    Object Remove(int proxyId1, int proxyId2) {
        // System.out.printf("PairManager.Remove(%d, %d)\n", proxyId1,
        // proxyId2);
        if (proxyId1 > proxyId2) {
            // integer primitive swap
            proxyId1 += proxyId2;
            proxyId2 = proxyId1 - proxyId2;
            proxyId1 -= proxyId2;
        }

        int hash = Hash(proxyId1, proxyId2) & TABLE_MASK;

        // Pair pair = Find(proxyId1, proxyId2, hash);
        // if (pair == null) {
        int pairIndex = FindIndex(proxyId1, proxyId2, hash);
        if (pairIndex == -1) { // -1 returned if not found
            return null;
        }

        Pair pair = m_pairs[pairIndex];

        Object userData = pair.userData;

        assert pair.proxyId1 == proxyId1;
        assert pair.proxyId2 == proxyId2;

        // FIXME? [ewj: I think this is safe, leaving the note just in case]
        // Java note: this was a nasty one to fix, because in the C++
        // pair - m_pairs was pointer arithmetic, used to extract the
        // array index. Should be resolved now using FindIndex method above
        // int pairIndex = int32(pair - m_pairs);
        assert pairIndex < m_pairCount;

        // Remove the pair from the hash table.
        int index = m_hashTable[hash];
        assert index != NULL_PAIR;

        int previous = NULL_PAIR;
        while (index != pairIndex) {
            previous = index;
            index = m_next[index];
        }

        if (previous != NULL_PAIR) {
            assert m_next[previous] == pairIndex;
            m_next[previous] = m_next[pairIndex];
        }
        else {
            m_hashTable[hash] = m_next[pairIndex];
        }

        // We now move the last pair into spot of the
        // pair being removed. We need to fix the hash
        // table indices to support the move.
        int lastPairIndex = m_pairCount - 1;

        // If the removed pair is the last pair, we are done.
        if (lastPairIndex == pairIndex) {
            --m_pairCount;
            return userData;
        }

        // Remove the last pair from the hash table.
        Pair last = m_pairs[lastPairIndex];
        int lastHash = Hash(last.proxyId1, last.proxyId2) & TABLE_MASK;

        index = m_hashTable[lastHash];
        assert index != NULL_PAIR;

        previous = NULL_PAIR;
        while (index != lastPairIndex) {
            previous = index;
            index = m_next[index];
        }

        if (previous != NULL_PAIR) {
            assert m_next[previous] == lastPairIndex;
            m_next[previous] = m_next[lastPairIndex];
        }
        else {
            m_hashTable[lastHash] = m_next[lastPairIndex];
        }

        // Copy the last pair into the remove pair's spot.
        // m_pairs[pairIndex] = m_pairs[lastPairIndex];
        m_pairs[pairIndex] = new Pair(m_pairs[lastPairIndex]);

        // Insert the last pair into the hash table
        m_next[pairIndex] = m_hashTable[lastHash];
        m_hashTable[lastHash] = pairIndex;

        --m_pairCount;

        return userData;
    }

    Pair Find(int proxyId1, int proxyId2) {
        // System.out.printf("PairManager.Find(%d, %d)\n", proxyId1, proxyId2);
        if (proxyId1 > proxyId2) {
            // integer primitive swap
            proxyId1 += proxyId2;
            proxyId2 = proxyId1 - proxyId2;
            proxyId1 -= proxyId2;

        }

        int hash = Hash(proxyId1, proxyId2) & TABLE_MASK;

        int index = m_hashTable[hash];
        while (index != NULL_PAIR
                && Equals(m_pairs[index], proxyId1, proxyId2) == false) {
            index = m_next[index];
        }

        if (index == NULL_PAIR) {
            return null;
        }

        assert index < m_pairCount;

        return m_pairs[index];
    }

    int FindIndex(int proxyId1, int proxyId2) {
        // System.out.printf("PairManager.FindIndex(%d, %d)\n", proxyId1,
        // proxyId2);
        if (proxyId1 > proxyId2) {
            // integer primitive swap
            proxyId1 += proxyId2;
            proxyId2 = proxyId1 - proxyId2;
            proxyId1 -= proxyId2;
        }

        int hash = Hash(proxyId1, proxyId2) & TABLE_MASK;

        int index = m_hashTable[hash];
        while (index != NULL_PAIR
                && Equals(m_pairs[index], proxyId1, proxyId2) == false) {
            index = m_next[index];
        }

        if (index == NULL_PAIR) {
            return -1;
        }

        assert index < m_pairCount;

        return index;
    }

    int GetCount() {
        return m_pairCount;
    }

    Pair[] GetPairs() {
        return m_pairs;
    }

    private Pair Find(int proxyId1, int proxyId2, int hash) {
        int index = m_hashTable[hash];

        while (index != NULL_PAIR
                && Equals(m_pairs[index], proxyId1, proxyId2) == false) {
            index = m_next[index];
        }

        if (index == NULL_PAIR) {
            return null;
        }

        assert index < m_pairCount;

        return m_pairs[index];
    }

    private int FindIndex(int proxyId1, int proxyId2, int hash) {
        int index = m_hashTable[hash];

        while (index != NULL_PAIR
                && Equals(m_pairs[index], proxyId1, proxyId2) == false) {
            index = m_next[index];
        }

        if (index == NULL_PAIR) {
            return -1;
        }

        assert index < m_pairCount;

        return index;
    }

    private int Hash(int proxyId1, int proxyId2) {
        int key = (proxyId2 << 16) | proxyId1;
        key = ~key + (key << 15);
        key = key ^ (key >>> 12);
        key = key + (key << 2);
        key = key ^ (key >>> 4);
        key = key * 2057;
        key = key ^ (key >>> 16);
        return key;
    }

    boolean Equals(Pair pair, int proxyId1, int proxyId2) {
        return pair.proxyId1 == proxyId1 && pair.proxyId2 == proxyId2;
    }
}
