package collision;

public class Bound {
    public int value;

    int proxyId;

    int stabbingCount;

    public Bound() {
        value = 0;
        proxyId = 0;
        stabbingCount = 0;
    }

    public Bound(Bound b) {
        value = b.value;
        proxyId = b.proxyId;
        stabbingCount = b.stabbingCount;
    }

    public void set(Bound b) {
        value = b.value;
        proxyId = b.proxyId;
        stabbingCount = b.stabbingCount;
    }

    boolean IsLower() {
        return (value & 1) == 0;
    }

    boolean IsUpper() {
        return (value & 1) == 1;
    }

    @Override
    public String toString() {
        String ret = "Bound variable:\n";
        ret += "value: " + value + "\n";
        ret += "proxyId: " + proxyId + "\n";
        ret += "stabbing count: " + stabbingCount + "\n";
        return ret;
    }
}
