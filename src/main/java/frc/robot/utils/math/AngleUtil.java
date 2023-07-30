package frc.robot.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class AngleUtil {
    public static final CoordinateSystem RIGHT_COUNTER_CLOCKWISE = CoordinateSystem.of(0, false);
    public static final CoordinateSystem RIGHT_CLOCKWISE = CoordinateSystem.of(0, true);
    public static final CoordinateSystem UP_COUNTER_CLOCKWISE = CoordinateSystem.of(90, false);
    public static final CoordinateSystem UP_CLOCKWISE = CoordinateSystem.of(90, true);
    public static final CoordinateSystem LEFT_COUNTER_CLOCKWISE = CoordinateSystem.of(180, false);
    public static final CoordinateSystem LEFT_CLOCKWISE = CoordinateSystem.of(180, true);
    public static final CoordinateSystem DOWN_COUNTER_CLOCKWISE = CoordinateSystem.of(270, false);
    public static final CoordinateSystem DOWN_CLOCKWISE = CoordinateSystem.of(270, true);

    public static double normalize(double angle) {
        while (angle < 0) {
            angle += 360;
        }
        return angle % 360;
    }

    public static double getAbsoluteAngle(CoordinateSystem coordinateSystem, double angle) {
        return new Angle(coordinateSystem, angle).getAbsoluteAngle();
    }

    public static class CoordinateSystem {
        public XDirection xDirection;
        public ThetaDirection thetaDirection;

        public CoordinateSystem(XDirection xDirection, ThetaDirection thetaDirection) {
            this.xDirection = xDirection;
            this.thetaDirection = thetaDirection;
        }

        public static CoordinateSystem of(int xDirection, boolean clockwise) {
            return new CoordinateSystem(
                    XDirection.of(xDirection),
                    ThetaDirection.of(clockwise)
            );
        }
    }

    public static class Angle {
        public CoordinateSystem coordinateSystem;
        public double value;

        public Angle(CoordinateSystem coordinateSystem, double value) {
            this.coordinateSystem = coordinateSystem;
            this.value = normalize(value);
        }

        public Angle(CoordinateSystem coordinateSystem, Rotation2d value) {
            this(coordinateSystem, value.getDegrees());
        }

        public double getAbsoluteAngle() {
            double absoluteAngle =
                    coordinateSystem.xDirection.zeroVal +
                    coordinateSystem.thetaDirection.get() * value;
            return normalize(absoluteAngle);
        }

        public double minus(Angle other) {
            return normalize(getAbsoluteAngle() - other.getAbsoluteAngle());
        }

        public double plus(Angle other) {
            return normalize(getAbsoluteAngle() + other.getAbsoluteAngle());
        }

        @Override
        public String toString() {
            return "Angle: \n" +
                    "   Coordinate System: " +
                        coordinateSystem.xDirection.name() + ", " +
                        coordinateSystem.thetaDirection.name() + "\n" +
                    "   Value: " + value;
        }
    }

    public enum XDirection {
        RIGHT(0),
        UP(90),
        LEFT(180),
        DOWN(270);

        public final int zeroVal;

        XDirection(int zeroVal) {
            this.zeroVal = zeroVal;
        }

        public static XDirection of(int zeroVal) {
            switch (zeroVal) {
                case 0:
                    return RIGHT;
                case 90:
                    return UP;
                case 180:
                    return LEFT;
            }
            return DOWN;
        }
    }

    public enum ThetaDirection {
        COUNTER_CLOCKWISE(false),
        CLOCKWISE(true);

        public final boolean clockwise;

        ThetaDirection(boolean clockwise) {
            this.clockwise = clockwise;
        }

        public int get() {
            return clockwise ? -1 : 1;
        }

        public static ThetaDirection of(boolean invert) {
            return invert ? CLOCKWISE : COUNTER_CLOCKWISE;
        }
    }
}
