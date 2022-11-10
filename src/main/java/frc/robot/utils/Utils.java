package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class Utils {
    public static final double EPSILON = 1e-9;

    /**
     * sets the value of the joystick to 0 if the value is less than the threshold
     *
     * @param val       the joystick value
     * @param threshold the threshold value
     * @return 0 if val is less than the threshold else val
     */
    public static double deadband(double val, double threshold) {
        if (Math.abs(val) < threshold)
            return 0;
        return val;
    }

    /**
     * @param input     the joystick input
     * @param threshold the joystick deadband threshold
     * @return the updated value after the deadband
     */
    public static double rotationalDeadband(double input, double threshold) {
        if (Math.abs(input) < threshold)
            return 0;
        return (input - (Math.signum(input) * threshold)) / (1 - threshold);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(double a, double b, double maxError) {
        return deadband(a - b, maxError) == 0;
    }

    public static double[] pose2dToArray(Pose2d pose) {
        return new double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    }
}