package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.Utils;

import static frc.robot.Constants.*;

public class SwerveDrive extends LoggedSubsystem {
    private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Rear left
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Rear right
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(mKinematics, new Rotation2d(),
            new Pose2d());

    private final SwerveDriveLogInputs inputs;
    private final SwerveModule mFrontLeft;
    private final SwerveModule mFrontRight;
    private final SwerveModule mRearLeft;
    private final SwerveModule mRearRight;
    private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public SwerveDrive() {
        super(SwerveDriveLogInputs.getInstance());
        inputs = SwerveDriveLogInputs.getInstance();

        mFrontLeft = new SwerveModule(
                Module.FL,
                FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_MOTOR_ID,
                3,
                OFFSETS[Module.FL.number],
                FRONT_LEFT_DRIVE_INVERTED,
                FRONT_LEFT_ANGLE_INVERTED,
                FRONT_LEFT_ANGLE_SENSOR_PHASE,
                FRONT_LEFT_MOTION_MAGIC_CONFIGS);
        mFrontRight = new SwerveModule(
                Module.FR,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
                2,
                OFFSETS[Module.FR.number],
                FRONT_RIGHT_DRIVE_INVERTED,
                FRONT_RIGHT_ANGLE_INVERTED,
                FRONT_RIGHT_ANGLE_SENSOR_PHASE,
                FRONT_RIGHT_MOTION_MAGIC_CONFIGS);
        mRearLeft = new SwerveModule(
                Module.RL,
                REAR_LEFT_MODULE_DRIVE_MOTOR_ID,
                REAR_LEFT_MODULE_STEER_MOTOR_ID,
                0,
                OFFSETS[Module.RL.number],
                REAR_LEFT_DRIVE_INVERTED,
                REAR_LEFT_ANGLE_INVERTED,
                REAR_LEFT_ANGLE_SENSOR_PHASE,
                REAR_LEFT_MOTION_MAGIC_CONFIGS);
        mRearRight = new SwerveModule(
                Module.RR,
                REAR_RIGHT_MODULE_DRIVE_MOTOR_ID,
                REAR_RIGHT_MODULE_STEER_MOTOR_ID,
                1,
                OFFSETS[Module.RR.number],
                REAR_RIGHT_DRIVE_INVERTED,
                REAR_RIGHT_ANGLE_INVERTED,
                REAR_RIGHT_ANGLE_SENSOR_PHASE,
                REAR_RIGHT_MOTION_MAGIC_CONFIGS);
    }

    public SwerveDriveKinematics getKinematics() {
        return mKinematics;
    }

    public void updateOdometry() {
        mOdometry.update(Robot.gyroscope.getAngle(), mKinematics.toSwerveModuleStates(getSpeeds()));
    }

    public void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(pose, Robot.gyroscope.getAngle());
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);
    }

    public void drive(double vx, double vy, double theta) {
        if (Utils.epsilonEquals(vx, 0, 0.05) &&
                Utils.epsilonEquals(vy, 0, 0.05) &&
                Utils.epsilonEquals(theta, 0, 0.05)) {
            mFrontLeft.stop();
            mFrontRight.stop();
            mRearLeft.stop();
            mRearRight.stop();
            return;
        }

        mChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx,
                vy,
                theta,
                Robot.gyroscope.getAngle());
    }

    public void setStates(SwerveModuleState[] states) {
        mChassisSpeeds = mKinematics.toChassisSpeeds(states);
    }

    @Override
    public void updateInputs() {
        inputs.speeds = Utils.chassisSpeedsToArray(mKinematics.toChassisSpeeds(
                mFrontLeft.getState(),
                mFrontRight.getState(),
                mRearLeft.getState(),
                mRearRight.getState()));
        inputs.pose = Utils.pose2dToArray(getPose());
    }

    @Override
    public String getSubsystemName() {
        return "SwerveDrive";
    }

    public ChassisSpeeds getSpeeds() {
        return Utils.arrayToChassisSpeeds(inputs.speeds);
    }

    public void stop() {
        mFrontLeft.stop();
        mFrontRight.stop();
        mRearLeft.stop();
        mRearRight.stop();
    }

    @Override
    public void periodic() {
        updateOdometry();
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(mChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.set(states[Module.FL.number].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                states[Module.FL.number].angle);
        mFrontRight.set(states[Module.FR.number].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                states[Module.FR.number].angle);
        mRearLeft.set(states[Module.RL.number].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                states[Module.RL.number].angle);
        mRearRight.set(states[Module.RR.number].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                states[Module.RR.number].angle);

        if (SmartDashboard.getBoolean("Zero Swerve", false)) {
            SmartDashboard.putString("Zero Positions", "{" +
                    mFrontLeft.getEncoderTicks() + ", " +
                    mFrontRight.getEncoderTicks() + ", " +
                    mRearLeft.getEncoderTicks() + ", " +
                    mRearRight.getEncoderTicks() + "}");
        }
    }

    public enum Module {
        FL(0),
        FR(1),
        RL(2),
        RR(3);

        public final int number;

        Module(int number) {
            this.number = number;
        }
    }
}
