package joints;

import engine.Ode4jEngine;
import snapshot.JointSnapshot;

public interface Joint {
    int id();
    JointSnapshot snapshot(Ode4jEngine engine);
}
