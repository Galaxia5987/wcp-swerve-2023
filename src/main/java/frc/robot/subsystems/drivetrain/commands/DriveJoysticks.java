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
        switch (pov) {
            case 0:
                swerveDrive.drive(speeds, new Translation2d(Constants.TORNADO_SPIN_DISTANCE, 0));
                break;
            case 90:
                swerveDrive.drive(speeds, new Translation2d(0, -Constants.TORNADO_SPIN_DISTANCE));
                break;
            case 180:
                swerveDrive.drive(speeds, new Translation2d(-Constants.TORNADO_SPIN_DISTANCE, 0));
                break;
            case 270:
                swerveDrive.drive(speeds, new Translation2d(0, Constants.TORNADO_SPIN_DISTANCE));
                break;
            default:
                super.execute();
                break;
        }
    }
}
