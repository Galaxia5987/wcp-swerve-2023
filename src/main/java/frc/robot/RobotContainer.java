package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.SwerveDrive;
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

    private final XboxController xboxController1 = new XboxController(0);
    private final XboxController xboxController2 = new XboxController(1);
    private final JoystickButton a = new JoystickButton(xboxController1, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xboxController1, XboxController.Button.kB.value);

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
        swerveDrive.setDefaultCommand(new XboxDrive(swerveDrive, xboxController2));
        elevator.setDefaultCommand(new XboxControl(xboxController1));
    }

    private void configureButtonBindings() {
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
