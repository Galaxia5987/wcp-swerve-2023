package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.IntegratedUtils;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.PIDFController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HolonomicDrive extends CommandBase {
    protected final SwerveDrive swerveDrive = Robot.swerveSubsystem;
    protected final PIDFController adjustController = new PIDFController(
            Constants.TARGET_ADJUST_Kp,
            0, 0,
            Constants.TARGET_ADJUST_Kf) {{
        enableContinuousInput(-180, 180);
    }};
    protected final SlewRateLimiter forwardLimiter = new SlewRateLimiter(Constants.XY_SLEW_RATE_LIMIT);
    protected final SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.XY_SLEW_RATE_LIMIT);
    protected final SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.ROTATION_SLEW_RATE_LIMIT);
    protected final DoubleSupplier forward;
    protected final DoubleSupplier strafe;
    protected final DoubleSupplier rotation;
    protected final BooleanSupplier turnToTarget;
    protected final BooleanSupplier lock;
    protected final BooleanSupplier robotOriented;

    public HolonomicDrive(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation,
                          BooleanSupplier turnToTarget, BooleanSupplier lock, BooleanSupplier robotOriented) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.turnToTarget = turnToTarget;
        this.lock = lock;
        this.robotOriented = robotOriented;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        swerveDrive.setFieldOriented(!robotOriented.getAsBoolean());
        if (turnToTarget.getAsBoolean()) {
            turnToTarget(speeds);
        } else if (lock.getAsBoolean()) {
            swerveDrive.lock();
        } else {
            swerveDrive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        }
    }

    protected ChassisSpeeds calculateVelocities() {
        double forwardVal = Utils.deadband(smooth(forwardLimiter.calculate(forward.getAsDouble())), 0.05);
        double strafeVal = Utils.deadband(smooth(strafeLimiter.calculate(strafe.getAsDouble())), 0.05);
        double rotationVal = Utils.deadband(smooth(rotationLimiter.calculate(rotation.getAsDouble())), 0.05);

        return new ChassisSpeeds(forwardVal * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                strafeVal * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                rotationVal * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }

    protected void turnToTarget(ChassisSpeeds speeds) {
        double rotationVal = adjustController.calculate(IntegratedUtils.angleToTarget(), 0);
        swerveDrive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, rotationVal);
    }

    protected double smooth(double val) {
        return Math.signum(val) * Math.pow(Math.abs(val), Constants.SMOOTHING_FACTOR);
    }
}
