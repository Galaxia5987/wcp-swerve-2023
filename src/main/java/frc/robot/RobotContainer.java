package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.XboxDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.command.TriggerControl;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.command.Intake;
import frc.robot.subsystems.gripper.command.Outtake;
import frc.robot.subsystems.gyroscope.Gyroscope;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Gripper gripper = Gripper.getInstance();

    private final XboxController xboxController1 = new XboxController(0);
    private final XboxController xboxController2 = new XboxController(1);
    private final Trigger xboxRightTrigger = new Trigger(() -> xboxController1.getRightTriggerAxis() > 0.2);
    private final Trigger xboxLeftTrigger = new Trigger(() -> xboxController1.getLeftTriggerAxis() > 0.2);
    private final JoystickButton a = new JoystickButton(xboxController1, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xboxController1, XboxController.Button.kB.value);
    private final JoystickButton x = new JoystickButton(xboxController1, XboxController.Button.kX.value);
    private final JoystickButton lb2 = new JoystickButton(xboxController2, XboxController.Button.kLeftBumper.value);
    private final JoystickButton lb = new JoystickButton(xboxController1, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rb = new JoystickButton(xboxController1, XboxController.Button.kRightBumper.value);
    private final JoystickButton y = new JoystickButton(xboxController1, XboxController.Button.kY.value);

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
        swerveDrive.setDefaultCommand(new XboxDrive(swerveDrive, xboxController1));
        elevator.setDefaultCommand(new TriggerControl(xboxController1));
    }

    private void configureButtonBindings() {
        lb.onTrue(new InstantCommand(gyroscope::resetYaw));

        a.whileTrue(new Intake(0.4, 1191.0));
        b.whileTrue(new Outtake(0.1, 0.4));

        y.onTrue(new InstantCommand(gripper::resetEncoder));
        x.onTrue(new InstantCommand(elevator::resetEncoder));
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
