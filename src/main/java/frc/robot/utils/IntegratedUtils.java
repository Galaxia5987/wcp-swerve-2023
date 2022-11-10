package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.math.AngleUtil;

/**
 * This class constitutes all the information provided by multiple subsystem.
 * <p>
 * Here one can implement useful functions without implementing suppliers
 * in the code, and ensure the same information for all commands.
 */
public class IntegratedUtils {
    private static final Shooter shooter = Shooter.getInstance();
    private static final Limelight limelight = Limelight.getInstance();
    private static final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private static final Intake intake = Intake.getInstance();

    /**
     * This function gets the distance to the target in two ways:
     * vision - if the vision has targets
     * odometry - if the vision doesn't have targets
     *
     * @return the distance to the target. [m]
     */
    public static double distanceToTarget() {
        return limelight.getDistance()
                .orElse(swerveDrive.getPose()
                        .minus(Constants.Vision.HUB_POSE)
                        .getTranslation().getNorm());
    }

    public static double angleToTarget() {
        return limelight.getYaw().orElseGet(() -> {
            var toTarget = swerveDrive.getPose().minus(Constants.Vision.HUB_POSE);
            var absoluteAngleToTarget = new AngleUtil.Angle(
                    AngleUtil.UP_COUNTER_CLOCKWISE,
                    Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX())));
                    var robotAngle = new AngleUtil.Angle(
                    AngleUtil.UP_COUNTER_CLOCKWISE,
                    Robot.getAngle());
            return AngleUtil.absoluteAngleToYaw(absoluteAngleToTarget.minus(robotAngle));
        });
    }
}
