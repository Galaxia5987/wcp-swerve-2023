package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.config.SwerveModuleConfigBase;
import frc.robot.utils.math.InterpolatingDoubleMap;
import org.photonvision.SimVisionTarget;

import java.util.HashMap;

import static frc.robot.Ports.SwerveDrive.*;

public final class Constants {
    public static final int TALON_TIMEOUT = 10;
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final double FIELD_WIDTH = 8.23; // Width of the field. [m]
    public static final double FIELD_LENGTH = 16.46; // Length of the field. [m]

    public static final class SwerveDrive {
        public static final int TICKS_PER_ROTATION_DRIVE_MOTOR = 2048;
        public static final int TICKS_PER_ROTATION_ANGLE_MOTOR = 1024;
        public static final double GEAR_RATIO_DRIVE_MOTOR = 7.5;
        public static final double GEAR_RATIO_ANGLE_MOTOR = 1;
        public static final double DRIVE_MOTOR_TICKS_PER_METER = GEAR_RATIO_DRIVE_MOTOR * TICKS_PER_ROTATION_DRIVE_MOTOR / (0.11 * Math.PI); // 4 * 0.0254
        public static final double ANGLE_MOTOR_TICKS_PER_RADIAN = GEAR_RATIO_ANGLE_MOTOR * TICKS_PER_ROTATION_ANGLE_MOTOR / (2 * Math.PI);

        public static final int MAX_CURRENT = 15; // [amps]

        // State Space
        public static final double VELOCITY_TOLERANCE = 5; // [rps]
        public static final double COST_LQR = 11;

        // Note that the values of MODEL_TOLERANCE and ENCODER_TOLERANCE should be a lot smaller (something like 1e-6)
        public static final double MODEL_TOLERANCE = 0.01;
        public static final double ENCODER_TOLERANCE = 0.01; // [ticks]

        // The heading is responsible for the angle of the whole chassis, while the angle is used in the angle motor itself.
        public static final double ALLOWABLE_ANGLE_ERROR = Math.toRadians(3); // [rad]
        public static final double WHEEL_RADIUS = 0.04688; // [m]

        public static final double ROBOT_LENGTH = 0.6624; // [m]
        public static final double ROBOT_WIDTH = 0.5224; // [m]

        // the rotational velocity of the robot, this constant multiplies the rotation output of the joystick
        public static final int ANGLE_CURVE_STRENGTH = 1;
        public static final int ANGLE_CRUISE_VELOCITY = 400;
        public static final int ANGLE_MOTION_ACCELERATION = 1300;
        public static final double XY_SLEW_RATE_LIMIT = 3.0;
        public static final double ROTATION_SLEW_RATE_LIMIT = 6.0;
        public static final double TARGET_ADJUST_Kp = 0.03;
        public static final double TARGET_ADJUST_Kf = 0.1;
        private static final double Rx = SwerveDrive.ROBOT_LENGTH / 2; // [m]
        private static final double Ry = SwerveDrive.ROBOT_WIDTH / 2; // [m]
        // Axis systems
        public static final Translation2d[] SWERVE_POSITIONS = new Translation2d[]{
                new Translation2d(Rx, -Ry),
                new Translation2d(Rx, Ry),
                new Translation2d(-Rx, -Ry),
                new Translation2d(-Rx, Ry)
        };
    }

    public static final class SwerveModule {
        public static final int TRIGGER_THRESHOLD_CURRENT = 2; // [amps]

        public static final double TRIGGER_THRESHOLD_TIME = 0.02; // [secs]
        public static final double RAMP_RATE = 0; // seconds from neutral to max


        // -1612, -840, 1189, 1562
        public static final int[] ZERO_POSITIONS = {114, -5997, -881, 304}; // fr, fl, rr, rl

        public static final SwerveModuleConfigBase frConfig = new SwerveModuleConfigBase.Builder(0)
                .configPorts(DRIVE_MOTOR_FR, ANGLE_MOTOR_FR)
                .configInversions(DRIVE_INVERTED_FR, ANGLE_INVERTED_FR, ANGLE_SENSOR_PHASE_FR)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[0])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase flConfig = new SwerveModuleConfigBase.Builder(1)
                .configPorts(DRIVE_MOTOR_FL, ANGLE_MOTOR_FL)
                .configInversions(DRIVE_INVERTED_FL, ANGLE_INVERTED_FL, ANGLE_SENSOR_PHASE_FL)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[1])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rrConfig = new SwerveModuleConfigBase.Builder(2)
                .configPorts(DRIVE_MOTOR_RR, ANGLE_MOTOR_RR)
                .configInversions(DRIVE_INVERTED_RR, ANGLE_INVERTED_RR, ANGLE_SENSOR_PHASE_RR)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[2])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rlConfig = new SwerveModuleConfigBase.Builder(3)
                .configPorts(DRIVE_MOTOR_RL, ANGLE_MOTOR_RL)
                .configInversions(DRIVE_INVERTED_RL, ANGLE_INVERTED_RL, ANGLE_SENSOR_PHASE_RL)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[3])
                .configJ(0.115)
                .build();
    }

    public static class Vision { //TODO: change for competition
        public static final double CAMERA_HEIGHT = 0.79; // [m]
        public static final double TARGET_HEIGHT_FROM_GROUND = 2.62; // [m] Pefzener 2.62
        public static final double CAMERA_PITCH = 34.67; // Pitch of the vision. [deg]

        public static final Pose2d HUB_POSE = new Pose2d( // Position of the hub relative to the field.
                new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2), new Rotation2d());
    }

    public static class Intake {
        public static final double DEFAULT_POWER = 0.6;
    }

    public static class Conveyor {
        public static final double DEFAULT_POWER = 0.5;
    }

    public static class Shooter {
        public static final double Kp = 1;
        public static final double Ki = 1;
        public static final double Kd = 1;
        public static final double Kf = 1;

        public static final double TICKS_PER_ROTATION = 2048;

        public static final InterpolatingDoubleMap SHOOT_MEASUREMENTS = new InterpolatingDoubleMap() {{
            put(-99999.0, 2700.0);
            put(1.7, 2700.0);
            put(1.9, 2800.0);
            put(1.95, 2800.0);
            put(2.12, 2800.0);
            put(2.43, 2770.0);
            put(2.74, 2870.0);
            put(3.02, 3000.0);
            put(3.3, 3100.0);
            put(3.62, 3150.0);
            put(3.89, 3200.0);
            put(4.08, 3200.0);
            put(4.36, 3360.0);
            put(4.4, 3360.0);
            put(4.66, 3400.0);
            put(4.78, 3400.0);
            put(4.83, 3550.0);
            put(4.91, 3500.0);
            put(5.26, 3650.0);
            put(5.46, 3720.0);
            put(5.73, 3860.0);
            put(99999.0, 3860.0);
        }};
    }

    public static class Hood {
        public static final double Kp = 1.3;
        public static final double Ki = 0;
        public static final double Kd = 0;
        public static final double Kf = 0.01;

        public static final double ALLOWABLE_ERROR = 0.1;

        public static final double GEAR_RATIO = 106.88;

        public static final double TICKS_PER_DEGREE = 2048 * GEAR_RATIO / 360.0;

        public static final double ZERO_POSITION = 1362;

        public static final double MAX_VELOCITY = 60; // [deg / s]
        public static final double MAX_ACCELERATION = 60; // [deg / s^2]

        public static final InterpolatingDoubleMap HOOD_MEASUREMENTS = new InterpolatingDoubleMap() {{
            put(-99999.0, 12.7);
            put(1.7, 12.7);
            put(1.9, 14.2);
            put(1.95, 14.3);
            put(2.12, 15.5);
            put(2.43, 16.5);
            put(2.74, 17.5);
            put(3.02, 17.7);
            put(3.3, 18.5);
            put(3.62, 19.3);
            put(3.89, 20.5);
            put(4.08, 21.5);
            put(4.36, 20.5);
            put(4.4, 21.0);
            put(4.66, 22.0);
            put(4.78, 22.5);
            put(4.83, 23.5);
            put(4.91, 23.7);
            put(5.26, 24.0);
            put(5.46, 24.0);
            put(5.73, 24.7);
            put(99999.0, 24.7);
        }};

    }
}
