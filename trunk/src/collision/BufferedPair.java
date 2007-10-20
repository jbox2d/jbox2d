package collision;

public class BufferedPair implements Comparable<BufferedPair> {
    int proxyId1;

    int proxyId2;

    private boolean equals(BufferedPair other) {
        return proxyId1 == other.proxyId1 && proxyId2 == other.proxyId2;
    }

    private boolean minor(BufferedPair other) {
        if (proxyId1 < other.proxyId1)
            return true;

        if (proxyId1 == other.proxyId1) {
            return proxyId2 < other.proxyId2;
        }

        return false;
    }

    public int compareTo(BufferedPair p) {
        if (minor(p)) {
            return -1;
        }
        else if (equals(p)) {
            return 0;
        }
        else {
            return 1;
        }
    }
}
