package frc.robot.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.controllers.PIDFController;
import frc.robot.utils.valuetuner.WebConstant;

public class PathFollowerCommand extends CommandBase {
    private final SwerveDrive swerveDrive = Robot.swerveSubsystem;

    private final PathPlannerTrajectory trajectory;
    private final boolean firstPath;
    private final Timer timer;
    private Trajectory.State lastState;

    private final PIDFController xController;
    private final PIDFController yController;
    private final PIDFController rotationController;

    private final WebConstant xyKp = WebConstant.of("PathFollowerCommand", "xyKp", 0.0);
    private final WebConstant xyKi = WebConstant.of("PathFollowerCommand", "xyKi", 0.0);
    private final WebConstant xyKd = WebConstant.of("PathFollowerCommand", "xyKd", 0.0);
    private final WebConstant xyKf = WebConstant.of("PathFollowerCommand", "xyKf", 0.0);

    private final WebConstant rotationKp = WebConstant.of("PathFollowerCommand", "rotationKp", 0.0);
    private final WebConstant rotationKi = WebConstant.of("PathFollowerCommand", "rotationKi", 0.0);
    private final WebConstant rotationKd = WebConstant.of("PathFollowerCommand", "rotationKd", 0.0);
    private final WebConstant rotationKf = WebConstant.of("PathFollowerCommand", "rotationKf", 0.0);

    public PathFollowerCommand(PathPlannerTrajectory trajectory, boolean firstPath) {
        this.trajectory = trajectory;
        this.firstPath = firstPath;
        timer = new Timer();

        xController = new PIDFController(0, 0, 0, 0);
        yController = new PIDFController(0, 0, 0, 0);
        rotationController = new PIDFController(0, 0, 0, 0);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        lastState = trajectory.sample(0);

        xController.setPIDF(xyKp.get(), xyKi.get(), xyKd.get(), xyKf.get());
        yController.setPIDF(xyKp.get(), xyKi.get(), xyKd.get(), xyKf.get());
        rotationController.setPIDF(rotationKp.get(), rotationKi.get(), rotationKd.get(), rotationKf.get());

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