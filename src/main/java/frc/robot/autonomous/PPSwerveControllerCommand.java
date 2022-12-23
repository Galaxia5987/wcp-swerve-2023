//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class PPSwerveControllerCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;
    private final Supplier<Pose2d> poseSupplier;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final Consumer<ChassisSpeeds> outputChassisSpeeds;
    private final boolean useKinematics;
    private final Field2d field = new Field2d();
    private PPHolonomicDriveController controller;
    private PIDController xController;
    private PIDController yController;
    private PIDController omegaController;

    public PPSwerveControllerCommand(PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier, PIDController xController, PIDController yController, PIDController rotationController, Consumer<ChassisSpeeds> outputChassisSpeeds, Subsystem... requirements) {
        this.trajectory = trajectory;
        this.poseSupplier = poseSupplier;
        this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
        this.xController = xController;
        this.yController = yController;
        this.omegaController = rotationController;
        this.outputChassisSpeeds = outputChassisSpeeds;
        this.outputModuleStates = null;
        this.kinematics = null;
        this.useKinematics = false;
        this.addRequirements(requirements);
    }

    public PPSwerveControllerCommand(PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier, SwerveDriveKinematics kinematics, PIDController xController, PIDController yController, PIDController rotationController, Consumer<SwerveModuleState[]> outputModuleStates, Subsystem... requirements) {
        this.trajectory = trajectory;
        this.poseSupplier = poseSupplier;
        this.kinematics = kinematics;
        this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
        this.outputModuleStates = outputModuleStates;
        this.outputChassisSpeeds = null;
        this.useKinematics = true;
        this.addRequirements(requirements);
    }

    public void initialize() {
        SmartDashboard.putData("PPSwerveControllerCommand_field", this.field);
        this.field.getObject("traj").setTrajectory(this.trajectory);
        this.timer.reset();
        this.timer.start();
        PathPlannerServer.sendActivePath(this.trajectory.getStates());
        xController = new PIDController(
                SmartDashboard.getNumber("PathFollowerCommand_xyKp", Constants.AUTO_XY_Kp),
                SmartDashboard.getNumber("PathFollowerCommand_xyKi", Constants.AUTO_XY_Ki),
                SmartDashboard.getNumber("PathFollowerCommand_xyKd", Constants.AUTO_XY_Kd)
        );
        yController = new PIDController(
                SmartDashboard.getNumber("PathFollowerCommand_xyKp", Constants.AUTO_XY_Kp),
                SmartDashboard.getNumber("PathFollowerCommand_xyKi", Constants.AUTO_XY_Ki),
                SmartDashboard.getNumber("PathFollowerCommand_xyKd", Constants.AUTO_XY_Kd)
        );
        omegaController = new PIDController(
                SmartDashboard.getNumber("PathFollowerCommand_rotationKp", Constants.AUTO_ROTATION_Kp),
                SmartDashboard.getNumber("PathFollowerCommand_rotationKi", Constants.AUTO_ROTATION_Ki),
                SmartDashboard.getNumber("PathFollowerCommand_rotationKd", Constants.AUTO_ROTATION_Kd)
        );
        controller = new PPHolonomicDriveController(
                xController, yController, omegaController
        );
    }

    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) this.trajectory.sample(currentTime);
        Pose2d currentPose = this.poseSupplier.get();
        this.field.setRobotPose(currentPose);
        PathPlannerServer.sendPathFollowingData(new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation), currentPose);
        SmartDashboard.putNumber("PPSwerveControllerCommand_xError", currentPose.getX() - desiredState.poseMeters.getX());
        SmartDashboard.putNumber("PPSwerveControllerCommand_yError", currentPose.getY() - desiredState.poseMeters.getY());
        SmartDashboard.putNumber("PPSwerveControllerCommand_rotationError", currentPose.getRotation().getRadians() - desiredState.holonomicRotation.getRadians());
        ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);
        if (this.useKinematics) {
            SwerveModuleState[] targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);
            this.outputModuleStates.accept(targetModuleStates);
        } else {
            this.outputChassisSpeeds.accept(targetChassisSpeeds);
        }

    }

    public void end(boolean interrupted) {
        this.timer.stop();
        if (interrupted) {
            if (this.useKinematics) {
                this.outputModuleStates.accept(this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0D, 0.0D, 0.0D)));
            } else {
                this.outputChassisSpeeds.accept(new ChassisSpeeds());
            }
        }

    }

    public boolean isFinished() {
        return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds());
    }
}
