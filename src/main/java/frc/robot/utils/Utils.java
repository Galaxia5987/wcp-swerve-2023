package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

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
        return (val - Math.signum(val) * threshold) / (1 - threshold);
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

    public static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
        return new double[]{speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond};
    }

    public static ChassisSpeeds arrayToChassisSpeeds(double[] array) {
        return new ChassisSpeeds(array[0], array[1], array[2]);
    }

    public static ChassisSpeeds deadbandSpeeds(ChassisSpeeds speeds, double tolerance) {
        double toleranceXY = tolerance * Constants.MAX_VELOCITY_METERS_PER_SECOND;
        double toleranceRotation = tolerance * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_AUTO;

        return new ChassisSpeeds(
                deadband(speeds.vxMetersPerSecond, toleranceXY),
                deadband(speeds.vyMetersPerSecond, toleranceXY),
                deadband(speeds.omegaRadiansPerSecond, toleranceRotation));
    }
}