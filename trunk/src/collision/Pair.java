package collision;

public class Pair implements Comparable<Pair> {
    private static final int PAIR_BUFFERED = 0x0001;

    private static final int PAIR_REMOVED = 0x0002;

    private static final int PAIR_RECEIVED = 0x0004;

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

    public void setBuffered() {
        status |= PAIR_BUFFERED;
    }

    public void clearBuffered() {
        status &= ~PAIR_BUFFERED;
    }

    public boolean isBuffered() {
        return (status & PAIR_BUFFERED) == PAIR_BUFFERED;
    }

    public void clearRemoved() {
        status &= ~PAIR_REMOVED;
    }

    public void setRemoved() {
        status |= PAIR_REMOVED;
    }

    public boolean isRemoved() {
        return (status & PAIR_REMOVED) == PAIR_REMOVED;
    }

    public void setReceived() {
        status |= PAIR_RECEIVED;
    }

    public boolean isReceived() {
        return (status & PAIR_RECEIVED) == PAIR_RECEIVED;
    }

    public int compareTo(Pair p) {
        // XXX check
        return proxyId1 - p.proxyId1;
    }
}