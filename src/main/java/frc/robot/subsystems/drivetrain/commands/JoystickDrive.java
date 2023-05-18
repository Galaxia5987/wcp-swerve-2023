package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class JoystickDrive extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    private final Joystick leftJoystick;
    private final Joystick rightJoystick;

    public JoystickDrive(Joystick leftJoystick, Joystick rightJoystick) {
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        if (leftJoystick.getRawButton(Ports.UI.JOYSTICK_TOP_BOTTOM_BUTTON)) {
            swerveDrive.lock();
        } else {
            double vx = -leftJoystick.getY();
            double vy = -leftJoystick.getX();
            double omega = -rightJoystick.getX();

            double magnitude = Math.hypot(vx, vy);
            double angle = Math.atan2(vy, vx);
            magnitude = MathUtil.applyDeadband(magnitude, 0.05);
            vx = Math.cos(angle) * magnitude;
            vy = Math.sin(angle) * magnitude;
            omega = MathUtil.applyDeadband(omega, 0.05);

            vx = Math.copySign(vx * vx, vx);
            vy = Math.copySign(vy * vy, vy);

            DriveSignal signal = new DriveSignal(
                    vx * SwerveConstants.MAX_VELOCITY_AUTO,
                    vy * SwerveConstants.MAX_VELOCITY_AUTO,
                    0.7 * omega * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    new Translation2d(),
                    true);
            swerveDrive.drive(signal);
        }
    }
}