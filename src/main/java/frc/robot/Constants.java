package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    public static final int TALON_TIMEOUT = 10;
    public static final int NOMINAL_VOLTAGE = 10;
    public static final Pose2d HUB_POSE = new Pose2d(8.23, 4.115, new Rotation2d());

    // Swerve
    public static final double TICKS_PER_ROTATION = 2048;
    public static final int[] OFFSETS = {20091, 6542, 15910, 1685};

    public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.5224;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6624;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 7;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 8;
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 4;
    public static final int REAR_LEFT_MODULE_DRIVE_MOTOR_ID = 1;
    public static final int REAR_LEFT_MODULE_STEER_MOTOR_ID = 6;
    public static final int REAR_RIGHT_MODULE_DRIVE_MOTOR_ID = 5;
    public static final int REAR_RIGHT_MODULE_STEER_MOTOR_ID = 2;
    public static final double DRIVE_REDUCTION = 0.1;
    public static final double ANGLE_GEAR_RATIO = (14.0 / 72.0) * 0.5;
    public static final double WHEEL_DIAMETER = 0.1;
    // kP, kI, kD, kF, sCurveStrength, cruiseVelocity, acceleration, allowableError,
    // maxIntegralAccum, peakOutput
    public static final double[] FRONT_LEFT_MOTION_MAGIC_CONFIGS = {1.2, 0, 0, 0.2, 1, 21288, 28000, 10, 5, 1};
    public static final double[] FRONT_RIGHT_MOTION_MAGIC_CONFIGS = {1.2, 0, 0, 0.2, 1, 21288, 28000, 10, 5, 1};
    public static final double[] REAR_LEFT_MOTION_MAGIC_CONFIGS = {1.2, 0, 0, 0.2, 1, 21288, 28000, 10, 5, 1};
    public static final double[] REAR_RIGHT_MOTION_MAGIC_CONFIGS = {1.2, 0, 0, 0.2, 1, 21288, 28000, 10, 5, 1};
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            DRIVE_REDUCTION *
            WHEEL_DIAMETER * Math.PI;
    public static final double MAX_ACCELERATION = MAX_VELOCITY_METERS_PER_SECOND / 2;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double TARGET_ADJUST_Kp = 0;
    public static final double TARGET_ADJUST_Kf = 0;
    public static final double KP_XY_CONTROLLER = 0;
    public static final double KI_XY_CONTROLLER = 0;
    public static final double KD_XY_CONTROLLER = 0;
    public static final double KF_XY_CONTROLLER = 0;

    public static final double XY_SLEW_RATE_LIMIT = 3.0;
    public static final double ROTATION_SLEW_RATE_LIMIT = 6.0;
    public static boolean FRONT_LEFT_DRIVE_INVERTED = false;
    public static boolean FRONT_LEFT_ANGLE_INVERTED = true;
    public static boolean FRONT_LEFT_ANGLE_SENSOR_PHASE = false;
    public static boolean FRONT_RIGHT_DRIVE_INVERTED = false;
    public static boolean FRONT_RIGHT_ANGLE_INVERTED = true;
    public static boolean FRONT_RIGHT_ANGLE_SENSOR_PHASE = false;
    public static boolean REAR_LEFT_DRIVE_INVERTED = false;
    public static boolean REAR_LEFT_ANGLE_INVERTED = true;
    public static boolean REAR_LEFT_ANGLE_SENSOR_PHASE = false;
    public static boolean REAR_RIGHT_DRIVE_INVERTED = false;
    public static boolean REAR_RIGHT_ANGLE_INVERTED = true;
    public static boolean REAR_RIGHT_ANGLE_SENSOR_PHASE = false;

    public static double SMOOTHING_FACTOR = 2;

    public static double TORNADO_SPIN_DISTANCE = DRIVETRAIN_WHEELBASE_METERS / 2;
}
