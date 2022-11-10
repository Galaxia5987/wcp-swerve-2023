package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.IntegratedUtils;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.PIDFController;

import java.util.function.DoubleSupplier;

public class HolonomicDrive extends CommandBase {
    protected final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    protected final PIDFController adjustController = new PIDFController(
            Constants.SwerveDrive.TARGET_ADJUST_Kp,
            0, 0,
            Constants.SwerveDrive.TARGET_ADJUST_Kf) {{
        enableContinuousInput(-180, 180);
    }};
    protected final SlewRateLimiter forwardLimiter = new SlewRateLimiter(Constants.SwerveDrive.XY_SLEW_RATE_LIMIT);
    protected final SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.SwerveDrive.XY_SLEW_RATE_LIMIT);
    protected final SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.SwerveDrive.ROTATION_SLEW_RATE_LIMIT);
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;

    public HolonomicDrive(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        swerveDrive.holonomicDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    protected ChassisSpeeds calculateVelocities() {
        double forwardVal = Utils.deadband(forwardLimiter.calculate(forward.getAsDouble()), 0.05);
        double strafeVal = Utils.deadband(strafeLimiter.calculate(strafe.getAsDouble()), 0.05);
        double rotationVal = Utils.deadband(rotationLimiter.calculate(rotation.getAsDouble()), 0.05);

        return new ChassisSpeeds(forwardVal, strafeVal, rotationVal);
    }

    protected void turnToTarget() {
        ChassisSpeeds speeds = calculateVelocities();
        double rotationVal = adjustController.calculate(IntegratedUtils.angleToTarget(), 0);
        swerveDrive.holonomicDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, rotationVal);
    }
}
