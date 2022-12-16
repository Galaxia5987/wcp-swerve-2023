package frc.robot.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.PIDFController;

public class FollowPath extends CommandBase {
    private final Timer timer = new Timer();
    private final SwerveDrive swerveDrive = Robot.swerveSubsystem;
    private final PathPlannerTrajectory trajectory;
    private final PIDFController xController;
    private final PIDFController yController;
    private final PIDFController thetaController = new PIDFController(Constants.AUTO_ROTATION_Kp, Constants.AUTO_ROTATION_Ki, Constants.AUTO_ROTATION_Kd, Constants.AUTO_ROTATION_Kf) {{
        enableContinuousInput(-Math.PI, Math.PI);
    }};
    private HolonomicDriveController holonomicDriveController;

    @SuppressWarnings("ParameterName")
    public FollowPath(PathPlannerTrajectory trajectory, boolean firstPath) {
        this.trajectory = trajectory;
        xController = new PIDFController(Constants.AUTO_XY_Kp, Constants.AUTO_XY_Ki, Constants.AUTO_XY_Kd, Constants.AUTO_XY_Kf);
        yController = new PIDFController(Constants.AUTO_ROTATION_Kp, Constants.AUTO_ROTATION_Ki, Constants.AUTO_ROTATION_Kd, Constants.AUTO_ROTATION_Kf);

        holonomicDriveController = new HolonomicDriveController(
                xController,
                yController,
                new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0., 0))
        );

        addRequirements(swerveDrive);

        if (firstPath) {
            swerveDrive.resetOdometry(trajectory.getInitialPose());
        }
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        xController.setPIDF(SmartDashboard.getNumber("PathFollowerCommand_xyKp", Constants.AUTO_XY_Kp),
                SmartDashboard.getNumber("PathFollowerCommand_xyKi", Constants.AUTO_XY_Ki),
                SmartDashboard.getNumber("PathFollowerCommand_xyKd", Constants.AUTO_XY_Kd),
                SmartDashboard.getNumber("PathFollowerCommand_xyKf", Constants.AUTO_XY_Kf));
        yController.setPIDF(SmartDashboard.getNumber("PathFollowerCommand_xyKp", Constants.AUTO_XY_Kp),
                SmartDashboard.getNumber("PathFollowerCommand_xyKi", Constants.AUTO_XY_Ki),
                SmartDashboard.getNumber("PathFollowerCommand_xyKd", Constants.AUTO_XY_Kd),
                SmartDashboard.getNumber("PathFollowerCommand_xyKf", Constants.AUTO_XY_Kf));
        thetaController.setPIDF(SmartDashboard.getNumber("PathFollowerCommand_rotationKp", 1.0),
                SmartDashboard.getNumber("PathFollowerCommand_rotationKi", 0.0),
                SmartDashboard.getNumber("PathFollowerCommand_rotationKd", 0.0),
                SmartDashboard.getNumber("PathFollowerCommand_rotationKf", 0.0));
        holonomicDriveController = new HolonomicDriveController(
                xController, yController,
                new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)
                ));
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = timer.get();

        var desiredState = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(curTime);

        var desiredSpeeds = holonomicDriveController.calculate(
                swerveDrive.getPose(),
                desiredState,
                desiredState.holonomicRotation
        );

        double rotation = thetaController.calculate(
                Robot.gyroscope.getAngle().getRadians(), desiredState.holonomicRotation.getRadians());

        double omega = rotation * Math.hypot(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond);
        ChassisSpeeds speeds = Utils.deadbandSpeeds(new ChassisSpeeds(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, omega + rotation), 0.05);
        swerveDrive.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
