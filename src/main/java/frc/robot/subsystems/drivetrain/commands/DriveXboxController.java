package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.XboxController;

import java.util.function.BooleanSupplier;

public class DriveXboxController extends HolonomicDrive {

    public DriveXboxController(XboxController controller) {
        super(controller::getLeftY, controller::getLeftX, controller::getRightX,
                controller::getAButton, controller::getYButton, controller::getXButton);
    }

}
