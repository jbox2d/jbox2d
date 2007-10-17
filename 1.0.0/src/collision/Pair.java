package collision;

public class Pair implements Comparable<Pair> {
    private static final int e_bufferedPair = 0x0001;

    private static final int e_removePair = 0x0002;

    private static final int e_pairReceived = 0x0004;

    public Object userData;

    public int proxyId1;

    public int proxyId2;

    public int status;

    public Pair() {

    }

    /**
     * Copy constructor
     */
    public Pair(Pair other) {
        this.userData = other.userData;
        this.proxyId1 = other.proxyId1;
        this.proxyId2 = other.proxyId2;
        this.status = other.status;
    }

    public void SetBuffered() {
        status |= e_bufferedPair;
    }

    public void ClearBuffered() {
        status &= ~e_bufferedPair;
    }

    public boolean IsBuffered() {
        return (status & e_bufferedPair) != 0;
    }

    public void SetAdded() {
        status &= ~e_removePair;
    }

    public void SetRemoved() {
        status |= e_removePair;
    }

    public boolean IsRemoved() {
        return (status & e_removePair) == e_removePair;
    }

    public void SetReceived() {
        status |= e_pairReceived;
    }

    public boolean IsReceived() {
        return (status & e_pairReceived) == e_pairReceived;
    }

    public int compareTo(Pair p) {
        // XXX check
        return proxyId1 - p.proxyId1;
    }
}
