package bodies;

import engine.Ode4jEngine;
import geometry.BoundingBox;
import geometry.Vector3D;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.ode.OdeHelper;

import java.util.EnumMap;
import java.util.List;
import java.util.stream.Stream;

public class Cube extends Body {
    private final double sideLength;

    private enum Cache {
        ANGLE, BBOX
    }

    private final EnumMap<Cache, Double> cacheTime;
    private Vector3D angleCacher;
    private BoundingBox boundingBoxCacher;

    public Cube(double sideLength, double mass) {
        this.sideLength = sideLength;
        this.mass = OdeHelper.createMass();
        this.mass.setBox(mass, sideLength, sideLength, sideLength);
        cacheTime = new EnumMap<>(Cache.class);
    }

    @Override
    public double currentVolume(double t) {
        return sideLength * sideLength * sideLength;
    }

    @Override
    public BoundingBox boundingBox(double t) {
        if (cacheTime.get(Cache.BBOX) != t) {
            Vector3D angle = angle(t);
            List<Vector3D> rotatedVertices =
                    Stream.of(new Vector3D(1d, 0d, 0d), new Vector3D(-1d, 0d, 0d),
                    new Vector3D(0d, 1d, 0d), new Vector3D(0d, 1d, 0d),
                    new Vector3D(0d, 0d, 1d), new Vector3D(0d, 0d, 1d))
                    .map(v -> v.rotate(angle)).toList();
            Vector3D mins = rotatedVertices.stream().reduce((v1, v2) ->
                    new Vector3D(
                            Math.min(v1.x(), v2.x()),
                            Math.min(v1.y(), v2.y()),
                            Math.min(v1.z(), v2.z()))).get();
            Vector3D maxs = rotatedVertices.stream().reduce((v1, v2) ->
                    new Vector3D(
                            Math.max(v1.x(), v2.x()),
                            Math.max(v1.y(), v2.y()),
                            Math.max(v1.z(), v2.z()))).get();
            boundingBoxCacher = new BoundingBox(mins, maxs);
        }
        return boundingBoxCacher;
    }

    @Override
    public Vector3D angle(double t) {
        if (cacheTime.get(Cache.ANGLE) != t) {
            DMatrix3C rotationMatrix = body.getRotation();
            angleCacher = new Vector3D(
                    Math.atan2(rotationMatrix.get21(), rotationMatrix.get22()),
                    Math.asin(-rotationMatrix.get20()),
                    Math.atan2(rotationMatrix.get10(), rotationMatrix.get00()));
        }
        return angleCacher;
    }

    @Override
    public void assemble(Ode4jEngine engine, Vector3D position) {
        for (Cache c : Cache.values()) {
            cacheTime.put(c, -1d);
        }
        body = OdeHelper.createBody(engine.getWorld());
        body.setPosition(position.x(), position.y(), position.z());
        body.setMass(mass);
        collisionGeometry = OdeHelper.createBox(engine.getSpace(), sideLength, sideLength, sideLength);
        collisionGeometry.setBody(body);
    }

    @Override
    public void rotate(Vector3D eulerAngles) {
        Vector3D[] rotationBase = new Vector3D[]{
                new Vector3D(1d, 0d, 0d).rotate(eulerAngles),
                new Vector3D(0d, 1d, 0d).rotate(eulerAngles),
                new Vector3D(0d, 0d, 1d).rotate(eulerAngles)
        };
        DMatrix3 rotationMatrix = new DMatrix3(
                rotationBase[0].x(), rotationBase[1].x(), rotationBase[2].x(),
                rotationBase[0].y(), rotationBase[1].y(), rotationBase[2].y(),
                rotationBase[0].z(), rotationBase[1].z(), rotationBase[2].z());
        body.setRotation(new DMatrix3().eqMul(rotationMatrix, body.getRotation()));
    }
}
