package dynamics;

public class StepInfo {
    public float dt; // time step

    public float inv_dt; // inverse time step (0 if dt == 0).

    public int iterations;
}
