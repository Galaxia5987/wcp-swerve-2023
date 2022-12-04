package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class DriveJoysticks extends HolonomicDrive {
    private final Joystick leftJoystick;

    public DriveJoysticks(Joystick leftJoystick, Joystick rightJoystick) {
        super(() -> -leftJoystick.getY(), () -> -leftJoystick.getX(), rightJoystick::getX,
                rightJoystick::getTrigger, () -> rightJoystick.getRawButton(5), leftJoystick::getTrigger);
        this.leftJoystick = leftJoystick;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        int pov = leftJoystick.getPOV();
        if (pov >= 0) {
            double angle = Math.toRadians((360 - pov) % 360);
            swerveDrive.drive(speeds, new Translation2d(
                    Math.cos(angle) * Constants.TORNADO_SPIN_DISTANCE,
                    Math.sin(angle) * Constants.TORNADO_SPIN_DISTANCE));
        } else {
            super.execute();
        }
    }
}
