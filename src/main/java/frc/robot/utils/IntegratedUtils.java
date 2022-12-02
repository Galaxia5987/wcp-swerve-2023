package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.math.AngleUtil;

/**
 * This class constitutes all the information provided by multiple subsystem.
 * <p>
 * Here one can implement useful functions without implementing suppliers
 * in the code, and ensure the same information for all commands.
 */
public class IntegratedUtils {
    private static final SwerveDrive swerveDrive = Robot.swerveSubsystem;

    public static double angleToTarget() {
        var toTarget = swerveDrive.getPose().minus(Constants.HUB_POSE);
        var absoluteAngleToTarget = new AngleUtil.Angle(
                AngleUtil.UP_COUNTER_CLOCKWISE,
                Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX())));
        var robotAngle = new AngleUtil.Angle(
                AngleUtil.UP_COUNTER_CLOCKWISE,
                Robot.gyroscope.getAngle());
        return AngleUtil.absoluteAngleToYaw(absoluteAngleToTarget.minus(robotAngle));
    }
}
