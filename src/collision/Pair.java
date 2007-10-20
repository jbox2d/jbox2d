package collision;

public class Pair implements Comparable<Pair> {
    private static final int e_pairBuffered = 0x0001;

    private static final int e_pairRemoved = 0x0002;

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
        status |= e_pairBuffered;
    }

    public void ClearBuffered() {
        status &= ~e_pairBuffered;
    }

    public boolean IsBuffered() {
        return (status & e_pairBuffered) == e_pairBuffered;
    }

    public void ClearRemoved() {
        status &= ~e_pairRemoved;
    }

    public void SetRemoved() {
        status |= e_pairRemoved;
    }

    public boolean IsRemoved() {
        return (status & e_pairRemoved) == e_pairRemoved;
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
