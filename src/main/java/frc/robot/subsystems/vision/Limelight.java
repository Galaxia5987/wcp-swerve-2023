package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.LoggedSubsystem;

import java.util.Optional;
import java.util.OptionalDouble;

public class Limelight extends LoggedSubsystem {
    private static Limelight INSTANCE = null;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tv = table.getEntry("tv");
    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry ty = table.getEntry("ty");

    private final LimelightLogInputs inputs;

    private Limelight() {
        super(LimelightLogInputs.getInstance());
        inputs = LimelightLogInputs.getInstance();
    }

    public static Limelight getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Limelight();
        }
        return INSTANCE;
    }

    public OptionalDouble getDistance() {
        return getDistance(
                Constants.Vision.TARGET_HEIGHT_FROM_GROUND,
                Constants.Vision.CAMERA_HEIGHT);
    }

    public OptionalDouble getDistance(double targetHeight, double cameraHeight) {
        if (hasTargets()) {
            double pitch = ty.getDouble(0.0);
            return OptionalDouble.of((targetHeight - cameraHeight) /
                    Math.tan(Math.toRadians(Constants.Vision.CAMERA_PITCH + pitch)));
        }
        return OptionalDouble.empty();
    }

    public OptionalDouble getYaw() {
        if (hasTargets()) {
            return OptionalDouble.of(tx.getDouble(0.0));
        }
        return OptionalDouble.empty();
    }

    public boolean hasTargets() {
        return tv.getDouble(0) != 0;
    }

    public Optional<Pose2d> estimatePose(Rotation2d robotAngle, double targetHeight, double cameraHeight) {
        OptionalDouble yaw = getYaw();
        OptionalDouble distance = getDistance(targetHeight, cameraHeight);

        if (yaw.isPresent() && distance.isPresent()) {
            double absoluteAngle = Math.toRadians(yaw.getAsDouble() + robotAngle.getDegrees());
            double dy = Math.sin(absoluteAngle) * distance.getAsDouble();
            double dx = Math.cos(absoluteAngle) * distance.getAsDouble();

            Translation2d translation = Constants.Vision.HUB_POSE
                    .getTranslation().minus(new Translation2d(dx, dy));

            return Optional.of(new Pose2d(translation, robotAngle));
        }

        return Optional.empty();
    }

    public Optional<Pose2d> estimatePose(Rotation2d robotAngle) {
        return estimatePose(robotAngle,
                Constants.Vision.TARGET_HEIGHT_FROM_GROUND,
                Constants.Vision.CAMERA_HEIGHT);
    }

    @Override
    public void updateInputs() {
        inputs.hasTargets = hasTargets();
        getYaw().ifPresent((value) -> inputs.estimatedYaw = value);
        getDistance().ifPresent((value) -> inputs.estimatedDistance = value);
    }

    @Override
    public String getSubsystemName() {
        return "Limelight";
    }
}
