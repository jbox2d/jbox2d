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

    public PairManager pairManager;

    public Proxy proxyPool[];

    int freeProxy;

    BufferedPair pairBuffer[];

    int pairBufferCount;

    public Bound m_bounds[][];

    PairCallback pairCallback;

    int queryResults[];

    int queryResultCount;

    public AABB m_worldAABB;

    public Vec2 quantizationFactor;

    public int proxyCount;

    int timeStamp;

    private static final boolean debugPrint = false;

    public static final boolean s_validate = false;

    // Dumps m_bounds array to console for debugging
    private void dump() {
        for (int i = 0; i < 10; i++) {
            System.out.printf("bounds[ %d ] = %d, %d \n", i,
                    m_bounds[0][i].value, m_bounds[1][i].value);
        }
    }

    public BroadPhase(AABB worldAABB, PairCallback callback) {
        if (debugPrint) {
            System.out.println("BroadPhase()");
        }

        // array initialization
        proxyPool = new Proxy[Settings.maxProxies];
        pairBuffer = new BufferedPair[Settings.maxPairs];
        m_bounds = new Bound[2][2 * Settings.maxProxies];
        queryResults = new int[Settings.maxProxies];

        for (int i = 0; i < 2 * Settings.maxProxies; i++) {
            m_bounds[0][i] = new Bound();
            m_bounds[1][i] = new Bound();
        }

        for (int i = 0; i < Settings.maxProxies; i++) {
            pairBuffer[i] = new BufferedPair();
        }

        pairManager = new PairManager();

        assert worldAABB.isValid();

        m_worldAABB = new AABB(worldAABB);
        pairCallback = callback;
        proxyCount = 0;

        Vec2 d = worldAABB.maxVertex.sub(worldAABB.minVertex);
        quantizationFactor = new Vec2(Integer.MAX_VALUE / d.x,
                Integer.MAX_VALUE / d.y);

        for (int i = 0; i < Settings.maxProxies - 1; ++i) {
            proxyPool[i] = new Proxy();
            proxyPool[i].setNext(i + 1);
            proxyPool[i].timeStamp = 0;
            proxyPool[i].overlapCount = INVALID;
            proxyPool[i].userData = null;
        }

        proxyPool[Settings.maxProxies - 1] = new Proxy();
        proxyPool[Settings.maxProxies - 1].setNext(PairManager.NULL_PROXY);
        proxyPool[Settings.maxProxies - 1].timeStamp = 0;
        proxyPool[Settings.maxProxies - 1].overlapCount = INVALID;
        proxyPool[Settings.maxProxies - 1].userData = null;
        freeProxy = 0;

        pairBufferCount = 0;

        timeStamp = 1;
        queryResultCount = 0;
    }

    boolean shouldCollide(int id1, int id2) {
        assert (id1 < Settings.maxProxies);
        assert (id2 < Settings.maxProxies);
        Proxy p1 = proxyPool[id1];
        Proxy p2 = proxyPool[id2];

        if (p1.groupIndex == p2.groupIndex && p1.groupIndex != 0) {
            return p1.groupIndex > 0;
        }

        return (p1.maskBits & p2.categoryBits) != 0
                && (p1.categoryBits & p2.maskBits) != 0;
    }

    public Proxy getProxy(int proxyId) {
        if (proxyId == PairManager.NULL_PROXY
                || (proxyPool[proxyId].isValid() == false)) {
            return null;
        }
        else {
            return proxyPool[proxyId];
        }
    }

    // Create and destroy proxies. These call Flush first.
    int CreateProxy(AABB aabb, int groupIndex, int categoryBits, int maskBits,
            Object userData) {
        if (debugPrint) {
            System.out.println("CreateProxy()");
        }

        if (freeProxy == PairManager.NULL_PROXY) {
            assert false : "m_freeProxy == NULL_PROXY error";

            return PairManager.NULL_PROXY;
        }

        // Flush the pair buffer
        flush();

        int proxyId = freeProxy;
        Proxy proxy = proxyPool[proxyId];
        freeProxy = proxy.getNext();

        proxy.overlapCount = 0;
        proxy.groupIndex = groupIndex;
        proxy.categoryBits = categoryBits;
        proxy.maskBits = maskBits;
        proxy.userData = userData;

        assert proxyCount < Settings.maxProxies;

        int edgeCount = 2 * proxyCount;

        int lowerValues[] = new int[2];
        int upperValues[] = new int[2];

        computeBounds(lowerValues, upperValues, aabb);

        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];
            int[] indexes = new int[2];

            query(indexes, lowerValues[axis], upperValues[axis], bounds,
                    edgeCount, axis);
            int lowerIndex = indexes[0];
            int upperIndex = indexes[1];
            // System.out.println(edgeCount + ", "+lowerValues[axis] + ",
            // "+upperValues[axis]);
            // memmove(bounds[upperIndex + 2], bounds[upperIndex],
            // (edgeCount - upperIndex) * sizeof(b2Bound));

            // Ah, so this is the big bad bug that's been holding this back:
            // System.arraycopy makes a shallow copy of the objects, unlike
            // memmove.
            // This means that we have to do copy objects by hand to get the
            // true deep copy that we desire...this will be a speed hit, no
            // doubt.
            System.arraycopy(m_bounds[axis], upperIndex, m_bounds[axis],
                    upperIndex + 2, edgeCount - upperIndex);
            for (int i = 0; i < edgeCount - upperIndex; i++) {
                m_bounds[axis][upperIndex + 2 + i] = new Bound(
                        m_bounds[axis][upperIndex + 2 + i]);
            }

            // memmove(bounds[lowerIndex + 1], bounds[lowerIndex],
            // (upperIndex - lowerIndex) * sizeof(b2Bound));
            // System.out.println(lowerIndex+" "+upperIndex);
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

            // if (bounds[lowerIndex] == null)
            assert (bounds[lowerIndex] != null) : "Null pointer (lower)";
            // if (bounds[upperIndex] == null)
            assert (bounds[upperIndex] != null) : "Null pointer (upper)";

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
                Proxy proxyn = proxyPool[bounds[index].proxyId];
                if (bounds[index].isLower()) {
                    proxyn.lowerBounds[axis] = index;
                }
                else {
                    proxyn.upperBounds[axis] = index;
                }
            }
        }

        ++proxyCount;

        assert queryResultCount < Settings.maxProxies;

        for (int i = 0; i < queryResultCount; ++i) {

            if (shouldCollide(proxyId, queryResults[i]) == false) {
                continue;
            }

            Pair pair = pairManager.add(proxyId, queryResults[i]);
            if (pair == null) {
                continue;
            }

            // The Add command may return an old pair, which should not
            // happen here.
            assert (pair.isReceived() == false);
            pair.userData = pairCallback.pairAdded(proxy.userData,
                    proxyPool[queryResults[i]].userData);
            pair.setReceived();
        }

        // #if defined(_DEBUG) && B2BP_VALIDATE == 1
        // Validate();
        // #endif

        // Prepare for next query.
        queryResultCount = 0;

        incrementTimeStamp();

        return proxyId;
    }

    public void destroyProxy(int proxyId) {
        if (proxyId == PairManager.NULL_PROXY) {
            assert (false);
            return;
        }

        // Flush the pair buffer.
        flush();

        Proxy proxy = proxyPool[proxyId];
        int edgeCount = 2 * proxyCount;

        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];

            int lowerIndex = proxy.lowerBounds[axis];
            int upperIndex = proxy.upperBounds[axis];
            int lowerValue = bounds[lowerIndex].value;
            int upperValue = bounds[upperIndex].value;

            // memmove(bounds + lowerIndex, bounds + lowerIndex + 1,
            // (upperIndex - lowerIndex - 1) * sizeof(b2Bound));
            // memmove(bounds[lowerIndex + 1], bounds[lowerIndex],
            // (upperIndex - lowerIndex) * sizeof(b2Bound));
            System.arraycopy(m_bounds[axis], lowerIndex + 1, m_bounds[axis],
                    lowerIndex, upperIndex - lowerIndex - 1);
            for (int i = 0; i < upperIndex - lowerIndex - 1; i++) {
                m_bounds[axis][lowerIndex + i] = new Bound(
                        m_bounds[axis][lowerIndex + i]);
            }
            // memmove(bounds + upperIndex-1, bounds + upperIndex + 1,
            // (edgeCount - upperIndex - 1) * sizeof(b2Bound));
            System.arraycopy(m_bounds[axis], upperIndex + 1, m_bounds[axis],
                    upperIndex - 1, edgeCount - upperIndex - 1);
            for (int i = 0; i < edgeCount - upperIndex - 1; i++) {
                m_bounds[axis][upperIndex - 1 + i] = new Bound(
                        m_bounds[axis][upperIndex - 1 + i]);
            }

            // Fix bound indices.
            for (int index = lowerIndex; index < edgeCount - 2; ++index) {
                Proxy proxyn = proxyPool[bounds[index].proxyId];
                if (bounds[index].isLower()) {
                    proxyn.lowerBounds[axis] = index;
                }
                else {
                    proxyn.upperBounds[axis] = index;
                }
            }

            // Fix stabbing count.
            for (int index = lowerIndex; index < upperIndex - 1; ++index) {
                --bounds[index].stabbingCount;
            }

            // Query for pairs to be removed. lowerIndex and upperIndex are not
            // needed.
            int[] ignored = new int[2];
            query(ignored, lowerValue, upperValue, bounds, edgeCount - 2, axis);
        }

        assert (queryResultCount < Settings.maxProxies);

        for (int i = 0; i < queryResultCount; ++i) {
            assert (proxy.isValid() && proxyPool[queryResults[i]].isValid());

            Proxy other = proxyPool[queryResults[i]];
            Object pairUserData = pairManager.remove(proxyId, queryResults[i]);
            pairCallback.pairRemoved(proxy.userData, other.userData,
                    pairUserData);
        }

        // Prepare for next query.
        queryResultCount = 0;
        incrementTimeStamp();

        // Invalidate the proxy.
        proxy.userData = null;
        proxy.overlapCount = BroadPhase.INVALID;

        // Return the proxy to the pool.
        proxy.setNext(freeProxy);
        freeProxy = proxyId;
        --proxyCount;

        // #if defined(_DEBUG)
        // if (s_validate)
        // {
        // Validate();
        // }
        // #endif
    }

    // Call MoveProxy as many times as you like, then when you are done
    // call Flush to finalized the proxy pairs (for your time step).
    void moveProxy(int proxyId, AABB aabb) {
        if (debugPrint) {
            System.out.println("MoveProxy()");
        }

        if (proxyId == PairManager.NULL_PROXY || Settings.maxProxies <= proxyId) {
            return;
        }

        assert (aabb.isValid()) : "invalid AABB";

        int edgeCount = 2 * proxyCount;

        Proxy proxy = proxyPool[proxyId];
        int lowerValues[] = new int[2];
        int upperValues[] = new int[2];
        computeBounds(lowerValues, upperValues, aabb);

        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];

            int lowerIndex = proxy.lowerBounds[axis];
            int upperIndex = proxy.upperBounds[axis];

            int lowerValue = lowerValues[axis];
            int upperValue = upperValues[axis];

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
                    Proxy prevProxy = proxyPool[prevEdge.proxyId];

                    ++prevEdge.stabbingCount;

                    if (prevEdge.isUpper() == true) {
                        if (testOverlap(proxy, prevProxy)) {
                            addBufferedPair(proxyId, prevProxyId);
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
                    Proxy nextProxy = proxyPool[nextProxyId];

                    ++nextEdge.stabbingCount;

                    if (nextEdge.isLower() == true) {
                        if (testOverlap(proxy, nextProxy)) {
                            addBufferedPair(proxyId, nextProxyId);
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
                    Proxy nextProxy = proxyPool[nextProxyId];

                    --nextEdge.stabbingCount;

                    if (nextEdge.isUpper()) {
                        removeBufferedPair(proxyId, nextProxyId);

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
                    Proxy prevProxy = proxyPool[prevProxyId];

                    --prevEdge.stabbingCount;

                    if (prevEdge.isLower() == true) {
                        removeBufferedPair(proxyId, prevProxyId);

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

    public void flush() {
        // Pair[] pairs = m_pairManager.GetPairs();
        int removeCount = 0;

        for (int i = 0; i < pairBufferCount; ++i) {
            Pair pair = pairManager.find(pairBuffer[i].proxyId1,
                    pairBuffer[i].proxyId2);
            assert (pair.isBuffered());

            Proxy proxy1 = proxyPool[pair.proxyId1];
            Proxy proxy2 = proxyPool[pair.proxyId2];

            assert (proxy1.isValid());
            assert (proxy2.isValid());

            // boolean overlap = TestOverlap(proxy1, proxy2);

            if (pair.isRemoved()) {
                assert (testOverlap(proxy1, proxy2) == false);

                if (pair.userData != null) {
                    pairCallback.pairRemoved(proxy1.userData, proxy2.userData,
                            pair.userData);
                }

                // Store the ids so we can actually remove the pair below.
                pairBuffer[removeCount].proxyId1 = pair.proxyId1;
                pairBuffer[removeCount].proxyId2 = pair.proxyId2;
                ++removeCount;
            }
            else {
                assert (testOverlap(proxy1, proxy2) == true);
                pair.clearBuffered();

                if (pair.isReceived() == false) {
                    pair.userData = pairCallback.pairAdded(proxy1.userData,
                            proxy2.userData);
                    pair.setReceived();
                }

            }
        }

        for (int i = 0; i < removeCount; ++i) {
            pairManager.remove(pairBuffer[i].proxyId1, pairBuffer[i].proxyId2);
        }

        pairBufferCount = 0;

    }

    // Query an AABB for overlapping proxies, returns the user data and
    // the count, up to the supplied maximum count.
    public Object[] query(AABB aabb, int maxCount) {
        if (debugPrint) {
            System.out.println("Query(2 args)");
        }

        int lowerValues[] = new int[2];
        int upperValues[] = new int[2];
        computeBounds(lowerValues, upperValues, aabb);

        int indexes[] = new int[2]; // lowerIndex, upperIndex;

        query(indexes, lowerValues[0], upperValues[0], m_bounds[0],
                2 * proxyCount, 0);
        query(indexes, lowerValues[1], upperValues[1], m_bounds[1],
                2 * proxyCount, 1);

        assert queryResultCount < Settings.maxProxies;

        Object[] results = new Object[maxCount];

        int count = 0;
        for (int i = 0; i < queryResultCount && count < maxCount; ++i, ++count) {
            assert queryResults[i] < Settings.maxProxies;
            Proxy proxy = proxyPool[queryResults[i]];
            proxy.isValid();
            results[i] = proxy.userData;
        }

        // Prepare for next query.
        queryResultCount = 0;
        incrementTimeStamp();

        return results;
    }

    public void validate() {
        if (debugPrint) {
            System.out.println("Validate()");
        }

        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];

            int pointCount = 2 * proxyCount;
            int stabbingCount = 0;

            for (int i = 0; i < pointCount; ++i) {
                Bound bound = bounds[i];
                if (i > 0) {
                    Bound prevEdge = bounds[i - 1];
                    assert prevEdge.value <= bound.value;
                }

                int proxyId = bound.proxyId;

                assert proxyId != PairManager.NULL_PROXY;

                Proxy proxy = proxyPool[bound.proxyId];

                assert (proxy.isValid());

                if (bound.isLower() == true) {
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

        Pair[] pairs = pairManager.getPairs();
        int pairCount = pairManager.getCount();
        assert (pairBufferCount <= pairCount);

        // this compiles, quite inefficient, but compiles!
        Arrays.sort(pairBuffer, 0, pairBufferCount);

        for (int i = 0; i < pairBufferCount; ++i) {
            if (i > 0) {
                assert (!pairBuffer[i].equals(pairBuffer[i - 1]));
            }

            Pair pair = pairManager.find(pairBuffer[i].proxyId1,
                    pairBuffer[i].proxyId2);
            assert (pair.isBuffered());

            Proxy proxy1 = proxyPool[pair.proxyId1];
            Proxy proxy2 = proxyPool[pair.proxyId2];

            assert (proxy1.isValid() == true);
            assert (proxy2.isValid() == true);

            boolean overlap = testOverlap(proxy1, proxy2);

            if (pair.isRemoved() == true) {
                assert (overlap == false);
            }
            else {
                assert (overlap == true);
            }
        }

        for (int i = 0; i < pairCount; ++i) {
            Pair pair = pairs[i];

            Proxy proxy1 = proxyPool[pair.proxyId1];
            Proxy proxy2 = proxyPool[pair.proxyId2];

            assert (proxy1.isValid() == true);
            assert (proxy2.isValid() == true);

            boolean overlap = testOverlap(proxy1, proxy2);

            if (pair.isBuffered()) {
                if (pair.isRemoved() == true) {
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

    void validatePairs() {
        if (debugPrint) {
            System.out.println("ValidatePairs()");
        }

        int pairCount = pairManager.getCount();
        assert (pairBufferCount <= pairCount);

        // this compiles, quite inefficient, but compiles!
        Collections.sort(Arrays.asList(pairBuffer));

        for (int i = 0; i < pairBufferCount; ++i) {
            if (i > 0) {
                // assert (BufferedPair.Equals(m_pairBuffer[i],
                // m_pairBuffer[i - 1]) == false);
                assert (!pairBuffer[i].equals(pairBuffer[i - 1]));
            }

            Pair pair = pairManager.find(pairBuffer[i].proxyId1,
                    pairBuffer[i].proxyId2);
            assert (pair.isBuffered());

            Proxy proxy1 = proxyPool[pair.proxyId1];
            Proxy proxy2 = proxyPool[pair.proxyId2];

            assert (proxy1.isValid() == true);
            assert (proxy2.isValid() == true);
        }
    }

    private void computeBounds(int[] lowerValues, int[] upperValues, AABB aabb) {
        if (debugPrint) {
            System.out.println("ComputeBounds()");
        }

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
        lowerValues[0] = (int) (quantizationFactor.x * (minVertex.x - m_worldAABB.minVertex.x))
                & (Integer.MAX_VALUE - 1);
        // System.out.println( (m_quantizationFactor.x * (minVertex.x -
        // m_worldAABB.minVertex.x))+"..."+lowerValues[0]);
        upperValues[0] = (int) (quantizationFactor.x * (maxVertex.x - m_worldAABB.minVertex.x)) | 1;
        // System.out.println( (m_quantizationFactor.x * (maxVertex.x -
        // m_worldAABB.minVertex.x))+"..."+upperValues[0]);

        lowerValues[1] = (int) (quantizationFactor.y * (minVertex.y - m_worldAABB.minVertex.y))
                & (Integer.MAX_VALUE - 1);
        upperValues[1] = (int) (quantizationFactor.y * (maxVertex.y - m_worldAABB.minVertex.y)) | 1;
    }

    private void addBufferedPair(int id1, int id2) {
        if (debugPrint) {
            System.out.println("AddPair()");
        }

        assert (proxyPool[id1].isValid() && proxyPool[id2].isValid());

        if (shouldCollide(id1, id2) == false) {
            return;
        }

        Pair pair = pairManager.add(id1, id2);

        if (pair == null) {
            return;
        }

        // If this pair is not in the pair buffer ...
        if (pair.isBuffered() == false) {
            // This must be a new pair.
            assert (pair.isReceived() == false);

            // If there is room in the pair buffer ...
            if (pairBufferCount < Settings.maxPairs) {
                // Add it to the pair buffer.
                pair.setBuffered();
                pairBuffer[pairBufferCount] = new BufferedPair();
                pairBuffer[pairBufferCount].proxyId1 = pair.proxyId1;
                pairBuffer[pairBufferCount].proxyId2 = pair.proxyId2;
                ++pairBufferCount;
            }

            assert (pairBufferCount <= pairManager.getCount());
        }

        // Confirm this pair for the subsequent call to Flush.
        pair.clearRemoved();

        // #if defined(_DEBUG) && B2BP_VALIDATE == 1
        // ValidatePairs();
        // #endif

    }

    private void removeBufferedPair(int id1, int id2) {
        if (debugPrint) {
            System.out.println("RemovePair()");
        }

        assert (proxyPool[id1].isValid() && proxyPool[id2].isValid());

        Pair pair = pairManager.find(id1, id2);

        if (pair == null) {
            return;
        }

        // If this pair is not in the pair buffer ...
        if (pair.isBuffered() == false) {
            // This must be an old pair.
            assert (pair.isReceived());

            if (pairBufferCount < Settings.maxPairs) {
                pair.setBuffered();
                pairBuffer[pairBufferCount].proxyId1 = pair.proxyId1;
                pairBuffer[pairBufferCount].proxyId2 = pair.proxyId2;
                ++pairBufferCount;
            }

            assert (pairBufferCount <= pairManager.getCount());
        }

        pair.setRemoved();

        // #if defined(_DEBUG) && B2BP_VALIDATE == 1
        // ValidatePairs();
        // #endif

    }

    private boolean testOverlap(Proxy p1, Proxy p2) {
        if (debugPrint) {
            System.out.println("TestOverlap()");
        }

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
    private void query(int[] results, int lowerValue, int upperValue,
            Bound[] bounds, int edgeCount, int axis) {
        if (debugPrint) {
            System.out.println("Query(6 args)");
        }

        int lowerQuery = binarySearch(bounds, edgeCount, lowerValue);
        int upperQuery = binarySearch(bounds, edgeCount, upperValue);

        // Easy case: lowerQuery <= lowerIndex(i) < upperQuery
        // Solution: search query range for min bounds.
        for (int i = lowerQuery; i < upperQuery; ++i) {
            if (bounds[i].isLower()) {
                incrementOverlapCount(bounds[i].proxyId);
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
                if (bounds[i].isLower()) {
                    Proxy proxy = proxyPool[bounds[i].proxyId];
                    if (lowerQuery <= proxy.upperBounds[axis]) {
                        incrementOverlapCount(bounds[i].proxyId);
                        --s;
                    }
                }
                --i;
            }
        }

        results[0] = lowerQuery;
        results[1] = upperQuery;
    }

    private void incrementOverlapCount(int proxyId) {
        if (debugPrint) {
            System.out.println("IncrementOverlapCount()");
        }

        Proxy proxy = proxyPool[proxyId];
        if (proxy.timeStamp < timeStamp) {
            proxy.timeStamp = timeStamp;
            proxy.overlapCount = 1;
        }
        else {
            proxy.overlapCount = 2;
            assert queryResultCount < Settings.maxProxies;
            queryResults[queryResultCount] = proxyId;
            ++queryResultCount;
        }
    }

    private void incrementTimeStamp() {
        if (debugPrint) {
            System.out.println("IncrementTimeStamp()");
        }

        if (timeStamp == Integer.MAX_VALUE) {
            for (int i = 0; i < Settings.maxProxies; ++i) {
                proxyPool[i].timeStamp = 0;
            }
            timeStamp = 1;
        }
        else {
            ++timeStamp;
        }
    }

    static int binarySearch(Bound[] bounds, int count, int value) {
        if (debugPrint) {
            System.out.println("BinarySearch()");
        }

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

    public boolean inRange(AABB aabb) {
        Vec2 d = Vec2.max(aabb.minVertex.sub(m_worldAABB.maxVertex),
                m_worldAABB.minVertex.sub(aabb.maxVertex));
        return (Math.max(d.x, d.y) < 0.0f);
    }
}
