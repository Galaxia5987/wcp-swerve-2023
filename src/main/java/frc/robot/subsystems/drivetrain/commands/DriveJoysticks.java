package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Joystick;

public class DriveJoysticks extends HolonomicDrive {

    public DriveJoysticks(Joystick leftJoystick, Joystick rightJoystick) {
        super(() -> -leftJoystick.getY(), () -> -leftJoystick.getX(), rightJoystick::getX,
                rightJoystick::getTrigger, rightJoystick::getTop, leftJoystick::getTrigger);
    }

}
