package frc.robot.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.controllers.PIDFController;
import frc.robot.utils.valuetuner.WebConstant;
import org.littletonrobotics.junction.Logger;

public class PathFollowerCommand extends CommandBase {
    private final SwerveDrive swerveDrive = Robot.swerveSubsystem;

    private final PathPlannerTrajectory trajectory;
    private final boolean firstPath;
    private final Timer timer;
    private final PIDFController xController;
    private final PIDFController yController;
    private final PIDFController rotationController;
    private Trajectory.State lastState;

    public PathFollowerCommand(PathPlannerTrajectory trajectory, boolean firstPath) {
        this.trajectory = trajectory;
        this.firstPath = firstPath;
        timer = new Timer();

        xController = new PIDFController(0, 0, 0, 0);
        yController = new PIDFController(0, 0, 0, 0);
        rotationController = new PIDFController(0, 0, 0, 0);

        Logger.getInstance().recordOutput("Current Path Following Command", trajectory.toString());

        SmartDashboard.putNumber("PathFollowerCommand_" + "xyKp", 1.0);
        SmartDashboard.putNumber("PathFollowerCommand_" + "xyKi", 0.0);
        SmartDashboard.putNumber("PathFollowerCommand_" + "xyKd", 0.0);
        SmartDashboard.putNumber("PathFollowerCommand_" + "xyKf", 0.0);

        SmartDashboard.putNumber("PathFollowerCommand_" + "rotationKp", 1.0);
        SmartDashboard.putNumber("PathFollowerCommand_" + "rotationKi", 0.0);
        SmartDashboard.putNumber("PathFollowerCommand_" + "rotationKd", 0.0);
        SmartDashboard.putNumber("PathFollowerCommand_" + "rotationKf", 0.0);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        lastState = trajectory.sample(0);

        xController.setPIDF(
                SmartDashboard.getNumber("PathFollowerCommand_" + "xyKp", 1.0),
                SmartDashboard.getNumber("PathFollowerCommand_" + "xyKi", 0.0),
                SmartDashboard.getNumber("PathFollowerCommand_" + "xyKd", 0.0),
                SmartDashboard.getNumber("PathFollowerCommand_" + "xyKf", 0.0)
        );
        yController.setPIDF(
                SmartDashboard.getNumber("PathFollowerCommand_" + "xyKp", 1.0),
                SmartDashboard.getNumber("PathFollowerCommand_" + "xyKi", 0.0),
                SmartDashboard.getNumber("PathFollowerCommand_" + "xyKd", 0.0),
                SmartDashboard.getNumber("PathFollowerCommand_" + "xyKf", 0.0)
        );
        rotationController.setPIDF(
                SmartDashboard.getNumber("PathFollowerCommand_" + "rotationKp", 1.0),
                SmartDashboard.getNumber("PathFollowerCommand_" + "rotationKi", 0.0),
                SmartDashboard.getNumber("PathFollowerCommand_" + "rotationKd", 0.0),
                SmartDashboard.getNumber("PathFollowerCommand_" + "rotationKf", 0.0)
        );

        if (firstPath) {
            swerveDrive.resetOdometry(trajectory.getInitialPose());
        }
    }

    @Override
    public void execute() {
        double time = timer.get();
        var desiredState = trajectory.sample(time);
        var desiredVelocityXY = desiredState.poseMeters.minus(lastState.poseMeters).times(1 / (desiredState.timeSeconds - lastState.timeSeconds));
        double desiredVelocityRotation = desiredState.curvatureRadPerMeter * desiredVelocityXY.getTranslation().getNorm();
        var currentVelocity = swerveDrive.getSpeeds();

        double velocityX = xController.calculate(currentVelocity.vxMetersPerSecond, desiredVelocityXY.getX());
        double velocityY = yController.calculate(currentVelocity.vyMetersPerSecond, desiredVelocityXY.getY());
        double rotation = rotationController.calculate(currentVelocity.omegaRadiansPerSecond, desiredVelocityRotation);

        swerveDrive.drive(velocityX, velocityY, rotation);

        lastState = desiredState;
    }

    @Override
    public boolean isFinished() {
        return trajectory.getTotalTimeSeconds() < timer.get();
    }
}