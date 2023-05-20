package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.JoystickDrive;
import frc.robot.subsystems.drivetrain.commands.XboxDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.command.HoldInPlace;
import frc.robot.subsystems.elevator.command.XboxControl;
import frc.robot.subsystems.gripper.command.Intake;
import frc.robot.subsystems.gyroscope.Gyroscope;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    private final XboxController xboxController = new XboxController(0);
    private final Trigger xboxRightTrigger = new Trigger(() -> xboxController.getRightTriggerAxis() > 0.2);
    private final Trigger xboxLeftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0.2);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xboxController, XboxController.Button.kB.value);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    private void configureDefaultCommands() {
        swerveDrive.setDefaultCommand(new XboxDrive(swerveDrive, xboxController));
//        elevator.setDefaultCommand(new HoldInPlace());
    }

    private void configureButtonBindings() {
        xboxLeftTrigger.onTrue(new InstantCommand(gyroscope::resetYaw));
        xboxRightTrigger.whileTrue(
                new InstantCommand(
                        ()-> elevator.setPower(xboxController.getRightTriggerAxis())));
        a.whileTrue(new Intake(0.1, 0.1, true));
        b.whileTrue(new Intake(0.1, 0.1, false));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
