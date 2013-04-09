package collision;

import java.util.Arrays;
import java.util.Collections;

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

    public PairManager m_pairManager;

    public Proxy m_proxyPool[];

    int m_freeProxy;

    BufferedPair m_pairBuffer[];

    int m_pairBufferCount;

    public Bound m_bounds[][];

    // b2PairCallback* m_pairCallback;
    PairCallback m_pairCallback;

    int m_queryResults[];

    int m_queryResultCount;

    public AABB m_worldAABB;

    public Vec2 m_quantizationFactor;

    public int m_proxyCount;

    int m_timeStamp;

    private static final boolean debugPrint = false;

    // Dumps m_bounds array to console for debugging
    private void dump() {
        for (int i = 0; i < 10; i++) {
            System.out.printf("bounds[ %d ] = %d, %d \n", i,
                    m_bounds[0][i].value, m_bounds[1][i].value);
        }
    }

    public BroadPhase(AABB worldAABB, PairCallback callback) {
        if (debugPrint)
            System.out.println("BroadPhase()");
        // array initialization
        m_proxyPool = new Proxy[Settings.maxProxies];
        m_pairBuffer = new BufferedPair[Settings.maxPairs];
        m_bounds = new Bound[2][2 * Settings.maxProxies];
        m_queryResults = new int[Settings.maxProxies];

        for (int i = 0; i < 2 * Settings.maxProxies; i++) {
            m_bounds[0][i] = new Bound();
            m_bounds[1][i] = new Bound();
        }

        for (int i = 0; i < Settings.maxProxies; i++) {
            m_pairBuffer[i] = new BufferedPair();
        }

        m_pairManager = new PairManager();

        assert worldAABB.isValid();
        m_worldAABB = new AABB(worldAABB);
        m_pairCallback = callback;
        m_proxyCount = 0;

        Vec2 d = worldAABB.maxVertex.sub(worldAABB.minVertex);
        m_quantizationFactor = new Vec2(Integer.MAX_VALUE / d.x,
                Integer.MAX_VALUE / d.y);

        for (int i = 0; i < Settings.maxProxies - 1; ++i) {
            m_proxyPool[i] = new Proxy();
            m_proxyPool[i].SetNext(i + 1);
            m_proxyPool[i].timeStamp = 0;
            m_proxyPool[i].overlapCount = INVALID;
            m_proxyPool[i].userData = null;
        }
        m_proxyPool[Settings.maxProxies - 1] = new Proxy();
        m_proxyPool[Settings.maxProxies - 1].SetNext(PairManager.NULL_PROXY);
        m_proxyPool[Settings.maxProxies - 1].timeStamp = 0;
        m_proxyPool[Settings.maxProxies - 1].overlapCount = INVALID;
        m_proxyPool[Settings.maxProxies - 1].userData = null;
        m_freeProxy = 0;// PairManager.NULL_PROXY;

        m_pairBufferCount = 0;

        m_timeStamp = 1;
        m_queryResultCount = 0;

    }

    // Create and destroy proxies. These call Flush first.
    int CreateProxy(AABB aabb, Object userData) {
        if (debugPrint)
            System.out.println("CreateProxy()");

        if (m_freeProxy == PairManager.NULL_PROXY) {
            assert false : "m_freeProxy == NULL_PROXY error";
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
            int[] indexes = new int[2];

            Query(indexes, lowerValues[axis], upperValues[axis], bounds,
                    edgeCount, axis);
            int lowerIndex = indexes[0];
            int upperIndex = indexes[1];

            // memmove(bounds[upperIndex + 2], bounds[upperIndex],
            // (edgeCount - upperIndex) * sizeof(b2Bound));

            // Ah, so this is the big bad bug that's been holding this back:
            // System.arraycopy makes a shallow copy of the objects, unlike
            // memmove.
            // This means that we have to do copy objects by hand to get the
            // true
            // deep copy that we desire...this will be a speed hit, no doubt.
            System.arraycopy(m_bounds[axis], upperIndex, m_bounds[axis],
                    upperIndex + 2, edgeCount - upperIndex);
            for (int i = 0; i < edgeCount - upperIndex; i++) {
                m_bounds[axis][upperIndex + 2 + i] = new Bound(
                        m_bounds[axis][upperIndex + 2 + i]);
            }

            // memmove(bounds[lowerIndex + 1], bounds[lowerIndex],
            // (upperIndex - lowerIndex) * sizeof(b2Bound));
            System.arraycopy(m_bounds[axis], lowerIndex, m_bounds[axis],
                    lowerIndex + 1, upperIndex - lowerIndex);
            for (int i = 0; i < upperIndex - lowerIndex; i++) {
                m_bounds[axis][lowerIndex + 1 + i] = new Bound(
                        m_bounds[axis][lowerIndex + 1 + i]);
            }

            // The upper index has increased because of the lower bound
            // insertion.
            ++upperIndex;

            // Copy in the new bounds.

            if (bounds[lowerIndex] == null)
                assert false : "Null pointer (lower)";
            if (bounds[upperIndex] == null)
                assert false : "Null pointer (upper)";

            bounds[lowerIndex].value = lowerValues[axis];
            bounds[lowerIndex].proxyId = proxyId;
            bounds[upperIndex].value = upperValues[axis];
            bounds[upperIndex].proxyId = proxyId;

            bounds[lowerIndex].stabbingCount = lowerIndex == 0 ? 0
                    : bounds[lowerIndex - 1].stabbingCount;
            bounds[upperIndex].stabbingCount = bounds[upperIndex - 1].stabbingCount;
            // System.out.printf("lv: %d , lid: %d, uv: %d, uid: %d
            // \n",lowerValues[axis],proxyId,upperValues[axis],proxyId);

            // Adjust the stabbing count between the new bounds.
            for (int index = lowerIndex; index < upperIndex; ++index) {
                ++bounds[index].stabbingCount;
            }

            // Adjust the all the affected bound indices.
            for (int index = lowerIndex; index < edgeCount + 2; ++index) {
                Proxy proxyn = m_proxyPool[bounds[index].proxyId];
                if (bounds[index].IsLower()) {
                    proxyn.lowerBounds[axis] = index;
                }
                else {
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

    void DestroyProxy(short proxyId) {
    }

    // Call MoveProxy as many times as you like, then when you are done
    // call Flush to finalized the proxy pairs (for your time step).
    void MoveProxy(int proxyId, AABB aabb) {
        if (debugPrint)
            System.out.println("MoveProxy()");
        if (proxyId == PairManager.NULL_PROXY || Settings.maxProxies <= proxyId) {
            return;
        }

        if (aabb.isValid() == false) {
            assert false;
            return;
        }

        int edgeCount = 2 * m_proxyCount;

        Proxy proxy = m_proxyPool[proxyId];
        int lowerValues[] = new int[2];
        int upperValues[] = new int[2];
        ComputeBounds(lowerValues, upperValues, aabb);

        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];

            int lowerIndex = proxy.lowerBounds[axis];
            int upperIndex = proxy.upperBounds[axis];

            int lowerValue = lowerValues[axis];
            int upperValue = upperValues[axis];
            // System.out.println(lowerIndex + " " +upperIndex+"
            // "+bounds.length);
            int deltaLower = lowerValue - bounds[lowerIndex].value;
            int deltaUpper = upperValue - bounds[upperIndex].value;

            bounds[lowerIndex].value = lowerValue;
            bounds[upperIndex].value = upperValue;

            //
            // Expanding adds overlaps
            //

            // Should we move the lower bound down?
            if (deltaLower < 0) {
                int index = lowerIndex;
                while (index > 0 && lowerValue < bounds[index - 1].value) {
                    Bound bound = bounds[index];
                    Bound prevEdge = bounds[index - 1];

                    int prevProxyId = prevEdge.proxyId;
                    Proxy prevProxy = m_proxyPool[prevEdge.proxyId];

                    ++prevEdge.stabbingCount;

                    if (prevEdge.IsUpper() == true) {
                        if (TestOverlap(proxy, prevProxy)) {
                            AddPair(proxyId, prevProxyId);
                        }

                        ++prevProxy.upperBounds[axis];
                        ++bound.stabbingCount;
                    }
                    else {
                        ++prevProxy.lowerBounds[axis];
                        --bound.stabbingCount;
                    }

                    --proxy.lowerBounds[axis];
                    // b2Swap(*bound, *prevEdge);
                    Bound tmp = new Bound(bound);
                    bound.set(prevEdge);
                    prevEdge.set(tmp);
                    --index;
                }
            }

            // Should we move the upper bound up?
            if (deltaUpper > 0) {
                int index = upperIndex;
                while (index < edgeCount - 1
                        && bounds[index + 1].value <= upperValue) {
                    Bound bound = bounds[index];
                    Bound nextEdge = bounds[index + 1];
                    int nextProxyId = nextEdge.proxyId;
                    Proxy nextProxy = m_proxyPool[nextProxyId];

                    ++nextEdge.stabbingCount;

                    if (nextEdge.IsLower() == true) {
                        if (TestOverlap(proxy, nextProxy)) {
                            AddPair(proxyId, nextProxyId);
                        }

                        --nextProxy.lowerBounds[axis];
                        ++bound.stabbingCount;
                    }
                    else {
                        --nextProxy.upperBounds[axis];
                        --bound.stabbingCount;
                    }

                    ++proxy.upperBounds[axis];
                    // b2Swap(*bound, *nextEdge);
                    // wasn't actually swapping! bounds[index] and
                    // bounds[index+1] need to be swapped by VALUE
                    Bound tmp = new Bound(bound);
                    bound.set(nextEdge);
                    nextEdge.set(tmp);
                    ++index;
                }
            }

            //
            // Shrinking removes overlaps
            //

            // Should we move the lower bound up?
            if (deltaLower > 0) {
                int index = lowerIndex;
                while (index < edgeCount - 1
                        && bounds[index + 1].value <= lowerValue) {
                    Bound bound = bounds[index];
                    Bound nextEdge = bounds[index + 1];

                    int nextProxyId = nextEdge.proxyId;
                    Proxy nextProxy = m_proxyPool[nextProxyId];

                    --nextEdge.stabbingCount;

                    if (nextEdge.IsUpper()) {
                        RemovePair(proxyId, nextProxyId);

                        --nextProxy.upperBounds[axis];
                        --bound.stabbingCount;
                    }
                    else {
                        --nextProxy.lowerBounds[axis];
                        ++bound.stabbingCount;
                    }

                    ++proxy.lowerBounds[axis];
                    // b2Swap(*bound, *nextEdge);
                    // Bound tmp = bound;
                    // bound = nextEdge;
                    // nextEdge = tmp;
                    Bound tmp = new Bound(bound);
                    bound.set(nextEdge);
                    nextEdge.set(tmp);
                    ++index;
                }
            }

            // Should we move the upper bound down?
            if (deltaUpper < 0) {
                int index = upperIndex;
                while (index > 0 && upperValue < bounds[index - 1].value) {
                    Bound bound = bounds[index];
                    Bound prevEdge = bounds[index - 1];

                    int prevProxyId = prevEdge.proxyId;
                    Proxy prevProxy = m_proxyPool[prevProxyId];

                    --prevEdge.stabbingCount;

                    if (prevEdge.IsLower() == true) {
                        RemovePair(proxyId, prevProxyId);

                        ++prevProxy.lowerBounds[axis];
                        --bound.stabbingCount;
                    }
                    else {
                        ++prevProxy.upperBounds[axis];
                        ++bound.stabbingCount;
                    }

                    --proxy.upperBounds[axis];
                    // b2Swap(*bound, *prevEdge);
                    // Bound tmp = bound;
                    // bound = prevEdge;
                    // prevEdge = tmp;
                    Bound tmp = new Bound(bound);
                    bound.set(prevEdge);
                    prevEdge.set(tmp);
                    --index;
                }
            }
        }

        // #if defined(_DEBUG) && B2BP_VALIDATE == 1
        // Validate();
        // #endif
    }

    public void Flush() {
        Pair[] pairs = m_pairManager.GetPairs();
        int removeCount = 0;

        for (int i = 0; i < m_pairBufferCount; ++i) {
            Pair pair = m_pairManager.Find(m_pairBuffer[i].proxyId1,
                    m_pairBuffer[i].proxyId2);
            assert (pair.IsBuffered());

            Proxy proxy1 = m_proxyPool[pair.proxyId1];
            Proxy proxy2 = m_proxyPool[pair.proxyId2];

            assert (proxy1.IsValid());
            assert (proxy2.IsValid());

            boolean overlap = TestOverlap(proxy1, proxy2);

            if (pair.IsRemoved()) {
                assert (overlap == false);

                if (pair.userData != null) {
                    m_pairCallback.PairRemoved(pair.userData);
                }

                // Store the ids so we can actually remove the pair below.
                m_pairBuffer[removeCount].proxyId1 = pair.proxyId1;
                m_pairBuffer[removeCount].proxyId2 = pair.proxyId2;
                ++removeCount;
            }
            else {
                assert (overlap == true);
                pair.ClearBuffered();

                if (pair.userData == null) {
                    pair.userData = m_pairCallback.PairAdded(proxy1.userData,
                            proxy2.userData);
                }
                assert (pair.userData != null);
            }
        }

        for (int i = 0; i < removeCount; ++i) {
            m_pairManager.Remove(m_pairBuffer[i].proxyId1,
                    m_pairBuffer[i].proxyId2);
        }

        m_pairBufferCount = 0;
        // if (debugPrint) System.out.println("Flush()");
        // int removeCount = 0;
        //
        // for (int i = 0; i < m_pairBufferCount; ++i){
        // Pair pair = m_pairManager.Find(m_pairBuffer[i].proxyId1,
        // m_pairBuffer[i].proxyId2);
        // assert (pair.IsBuffered());
        // //System.out.println(pair.proxyId1+" "+pair.proxyId2);
        // Proxy proxy1 = m_proxyPool[pair.proxyId1];
        // Proxy proxy2 = m_proxyPool[pair.proxyId2];
        //
        // assert (proxy1.IsValid());
        // assert (proxy2.IsValid());
        //
        // boolean overlap = TestOverlap(proxy1, proxy2);
        //
        // if (pair.IsRemoved()){
        // assert (overlap == false);
        //
        // if (pair.IsReceived()){
        // m_pairCallback.PairRemoved(proxy1.userData, proxy2.userData,
        // pair.userData);
        // }
        //
        // // Store the ids so we can actually remove the pair below.
        // m_pairBuffer[removeCount].proxyId1 = pair.proxyId1;
        // m_pairBuffer[removeCount].proxyId2 = pair.proxyId2;
        // ++removeCount;
        // } else{
        // assert (overlap == true);
        // pair.ClearBuffered();
        //
        // if (pair.IsReceived() == false){
        // pair.userData = m_pairCallback.PairAdded(proxy1.userData,
        // proxy2.userData);
        // pair.SetReceived();
        // }
        // }
        // }
        //
        // for (int i = 0; i < removeCount; ++i){
        // m_pairManager.Remove(m_pairBuffer[i].proxyId1,
        // m_pairBuffer[i].proxyId2);
        // }
        //
        // m_pairBufferCount = 0;
    }

    // Query an AABB for overlapping proxies, returns the user data and
    // the count, up to the supplied maximum count.
    public Object[] Query(AABB aabb, int maxCount) {
        if (debugPrint)
            System.out.println("Query(2 args)");
        int lowerValues[] = new int[2];
        int upperValues[] = new int[2];
        ComputeBounds(lowerValues, upperValues, aabb);

        int indexes[] = new int[2]; // lowerIndex, upperIndex;

        Query(indexes, lowerValues[0], upperValues[0], m_bounds[0],
                2 * m_proxyCount, 0);
        Query(indexes, lowerValues[1], upperValues[1], m_bounds[1],
                2 * m_proxyCount, 1);

        assert m_queryResultCount < Settings.maxProxies;

        Object[] results = new Object[maxCount];

        int count = 0;
        for (int i = 0; i < m_queryResultCount && count < maxCount; ++i, ++count) {
            assert m_queryResults[i] < Settings.maxProxies;
            Proxy proxy = m_proxyPool[m_queryResults[i]];
            proxy.IsValid();
            results[i] = proxy.userData;
        }

        // Prepare for next query.
        m_queryResultCount = 0;
        IncrementTimeStamp();

        return results;
    }

    public void Validate() {
        if (debugPrint)
            System.out.println("Validate()");
        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];

            int pointCount = 2 * m_proxyCount;
            int stabbingCount = 0;

            for (int i = 0; i < pointCount; ++i) {
                Bound bound = bounds[i];
                if (i > 0) {
                    Bound prevEdge = bounds[i - 1];
                    assert prevEdge.value <= bound.value;
                }

                int proxyId = bound.proxyId;

                assert proxyId != PairManager.NULL_PROXY;

                Proxy proxy = m_proxyPool[bound.proxyId];

                assert (proxy.IsValid());

                if (bound.IsLower() == true) {
                    assert (proxy.lowerBounds[axis] == i) : (proxy.lowerBounds[axis]
                            + " not " + i);
                    ++stabbingCount;
                }
                else {
                    assert (proxy.upperBounds[axis] == i);
                    --stabbingCount;
                }

                assert (bound.stabbingCount == stabbingCount);
            }
        }

        Pair[] pairs = m_pairManager.GetPairs();
        int pairCount = m_pairManager.GetCount();
        assert (m_pairBufferCount <= pairCount);

        // this compiles, quite inefficient, but compiles!
        // Collections.sort(Arrays.asList(m_pairBuffer),0,m_pairBufferCount);
        Arrays.sort(m_pairBuffer, 0, m_pairBufferCount);

        for (int i = 0; i < m_pairBufferCount; ++i) {
            if (i > 0) {
                // assert (BufferedPair.Equals(m_pairBuffer[i], m_pairBuffer[i -
                // 1]) == false);
                assert (!m_pairBuffer[i].equals(m_pairBuffer[i - 1]));
            }

            Pair pair = m_pairManager.Find(m_pairBuffer[i].proxyId1,
                    m_pairBuffer[i].proxyId2);
            assert (pair.IsBuffered());

            Proxy proxy1 = m_proxyPool[pair.proxyId1];
            Proxy proxy2 = m_proxyPool[pair.proxyId2];

            assert (proxy1.IsValid() == true);
            assert (proxy2.IsValid() == true);

            boolean overlap = TestOverlap(proxy1, proxy2);

            if (pair.IsRemoved() == true) {
                assert (overlap == false);
            }
            else {
                assert (overlap == true);
            }
        }

        for (int i = 0; i < pairCount; ++i) {
            Pair pair = pairs[i];

            Proxy proxy1 = m_proxyPool[pair.proxyId1];
            Proxy proxy2 = m_proxyPool[pair.proxyId2];

            assert (proxy1.IsValid() == true);
            assert (proxy2.IsValid() == true);

            boolean overlap = TestOverlap(proxy1, proxy2);

            if (pair.IsBuffered()) {
                if (pair.IsRemoved() == true) {
                    assert (overlap == false);
                }
                else {
                    assert (overlap == true);
                }
            }
            else {
                assert (overlap == true);
            }
        }
    }

    void ValidatePairs() {
        if (debugPrint)
            System.out.println("ValidatePairs()");
        // unused? Pair[] pairs = m_pairManager.GetPairs();
        int pairCount = m_pairManager.GetCount();
        assert (m_pairBufferCount <= pairCount);

        // this compiles, quite inefficient, but compiles!
        Collections.sort(Arrays.asList(m_pairBuffer));

        for (int i = 0; i < m_pairBufferCount; ++i) {
            if (i > 0) {
                // assert (BufferedPair.Equals(m_pairBuffer[i],
                // m_pairBuffer[i - 1]) == false);
                assert (!m_pairBuffer[i].equals(m_pairBuffer[i - 1]));
            }

            Pair pair = m_pairManager.Find(m_pairBuffer[i].proxyId1,
                    m_pairBuffer[i].proxyId2);
            assert (pair.IsBuffered());

            Proxy proxy1 = m_proxyPool[pair.proxyId1];
            Proxy proxy2 = m_proxyPool[pair.proxyId2];

            assert (proxy1.IsValid() == true);
            assert (proxy2.IsValid() == true);
        }
    }

    private void ComputeBounds(int[] lowerValues, int[] upperValues, AABB aabb) {
        if (debugPrint)
            System.out.println("ComputeBounds()");
        Vec2 minVertex = MathUtils.clamp(aabb.minVertex, m_worldAABB.minVertex,
                m_worldAABB.maxVertex);
        Vec2 maxVertex = MathUtils.clamp(aabb.maxVertex, m_worldAABB.minVertex,
                m_worldAABB.maxVertex);
        // System.out.printf("minV = %f %f, maxV = %f %f
        // \n",aabb.minVertex.x,aabb.minVertex.y,aabb.maxVertex.x,aabb.maxVertex.y);

        // Bump lower bounds downs and upper bounds up. This ensures correct
        // sorting of
        // lower/upper bounds that would have equal values.
        // TODO_ERIN implement fast float to int conversion.
        lowerValues[0] = (int) (m_quantizationFactor.x * (minVertex.x - m_worldAABB.minVertex.x))
                & (Integer.MAX_VALUE - 1);
        // System.out.println( (m_quantizationFactor.x * (minVertex.x -
        // m_worldAABB.minVertex.x))+"..."+lowerValues[0]);
        upperValues[0] = (int) (m_quantizationFactor.x * (maxVertex.x - m_worldAABB.minVertex.x)) | 1;
        // System.out.println( (m_quantizationFactor.x * (maxVertex.x -
        // m_worldAABB.minVertex.x))+"..."+upperValues[0]);

        lowerValues[1] = (int) (m_quantizationFactor.y * (minVertex.y - m_worldAABB.minVertex.y))
                & (Integer.MAX_VALUE - 1);
        upperValues[1] = (int) (m_quantizationFactor.y * (maxVertex.y - m_worldAABB.minVertex.y)) | 1;
    }

    private void AddPair(int id1, int id2) {
        if (debugPrint)
            System.out.println("AddPair()");
        assert (m_proxyPool[id1].IsValid() && m_proxyPool[id2].IsValid());

        Pair pair = m_pairManager.Add(id1, id2);

        if (pair == null) {
            return;
        }

        // If this pair is not in the pair buffer ...
        if (pair.IsBuffered() == false) {
            // This must be a new pair.
            assert (pair.userData == null);

            // If there is room in the pair buffer ...
            if (m_pairBufferCount < Settings.maxPairs) {
                // Add it to the pair buffer.
                pair.SetBuffered();
                m_pairBuffer[m_pairBufferCount] = new BufferedPair();
                m_pairBuffer[m_pairBufferCount].proxyId1 = pair.proxyId1;
                m_pairBuffer[m_pairBufferCount].proxyId2 = pair.proxyId2;
                ++m_pairBufferCount;
            }

            assert (m_pairBufferCount <= m_pairManager.GetCount());
        }

        // Confirm this pair for the subsequent call to Flush.
        pair.SetAdded();

        // #if defined(_DEBUG) && B2BP_VALIDATE == 1
        // ValidatePairs();
        // #endif

    }

    private void RemovePair(int id1, int id2) {
        if (debugPrint)
            System.out.println("RemovePair()");
        assert (m_proxyPool[id1].IsValid() && m_proxyPool[id2].IsValid());

        Pair pair = m_pairManager.Find(id1, id2);

        if (pair == null) {
            return;
        }

        // If this pair is not in the pair buffer ...
        if (pair.IsBuffered() == false) {
            // This must be an old pair.
            assert (pair.userData != null);

            if (m_pairBufferCount < Settings.maxPairs) {
                pair.SetBuffered();
                m_pairBuffer[m_pairBufferCount].proxyId1 = pair.proxyId1;
                m_pairBuffer[m_pairBufferCount].proxyId2 = pair.proxyId2;
                ++m_pairBufferCount;
            }

            assert (m_pairBufferCount <= m_pairManager.GetCount());
        }

        pair.SetRemoved();

        // #if defined(_DEBUG) && B2BP_VALIDATE == 1
        // ValidatePairs();
        // #endif

    }

    private boolean TestOverlap(Proxy p1, Proxy p2) {
        if (debugPrint)
            System.out.println("TestOverlap()");
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
     * @param results
     *            out variable
     */
    private void Query(int[] results, int lowerValue, int upperValue,
            Bound[] bounds, int edgeCount, int axis) {
        if (debugPrint)
            System.out.println("Query(6 args)");

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
                assert (i >= 0) : ("i = " + i + "; s = " + s);
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

        results[0] = lowerQuery;
        results[1] = upperQuery;
    }

    private void IncrementOverlapCount(int proxyId) {
        if (debugPrint)
            System.out.println("IncrementOverlapCount()");
        Proxy proxy = m_proxyPool[proxyId];
        if (proxy.timeStamp < m_timeStamp) {
            proxy.timeStamp = m_timeStamp;
            proxy.overlapCount = 1;
        }
        else {
            proxy.overlapCount = 2;
            assert m_queryResultCount < Settings.maxProxies;
            m_queryResults[m_queryResultCount] = proxyId;
            ++m_queryResultCount;
        }
    }

    private void IncrementTimeStamp() {
        if (debugPrint)
            System.out.println("IncrementTimeStamp()");
        if (m_timeStamp == Integer.MAX_VALUE) {
            for (int i = 0; i < Settings.maxProxies; ++i) {
                m_proxyPool[i].timeStamp = 0;
            }
            m_timeStamp = 1;
        }
        else {
            ++m_timeStamp;
        }
    }

    static int BinarySearch(Bound[] bounds, int count, int value) {
        if (debugPrint)
            System.out.println("BinarySearch()");
        int low = 0;
        int high = count - 1;
        while (low <= high) {
            int mid = (low + high) >> 1;
            if (bounds[mid].value > value) {
                high = mid - 1;
            }
            else if (bounds[mid].value < value) {
                low = mid + 1;
            }
            else {
                return mid;
            }
        }

        return low;

    }

}
