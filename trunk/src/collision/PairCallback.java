package collision;

public abstract class PairCallback {
    // This returns the new pair user data.
    public abstract Object PairAdded(Object proxyUserData1,
            Object proxyUserData2);

    // This should free the pair's user data.
    public abstract void PairRemoved(Object pairUserData);
}
