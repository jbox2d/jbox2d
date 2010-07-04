package collision;

public class BufferedPair implements Comparable<BufferedPair> {
    int proxyId1;

    int proxyId2;

    private boolean Equals(BufferedPair pair1, BufferedPair pair2) {
        return pair1.proxyId1 == pair2.proxyId1
                && pair1.proxyId2 == pair2.proxyId2;
    }

    private boolean minor(BufferedPair pair1, BufferedPair pair2) {
        if (pair1.proxyId1 < pair2.proxyId1)
            return true;

        if (pair1.proxyId1 == pair2.proxyId1) {
            return pair1.proxyId2 < pair2.proxyId2;
        }

        return false;
    }

    public int compareTo(BufferedPair p) {
        if (minor(this, p)) {
            return -1;
        }
        else if (Equals(this, p)) {
            return 0;
        }
        else {
            return 1;
        }
    }
}
