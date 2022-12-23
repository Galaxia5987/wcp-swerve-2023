package frc.robot;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.PPSwerveControllerCommand;
import frc.robot.subsystems.drivetrain.commands.DriveXboxController;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;
    private final XboxController xboxController = new XboxController(0);
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
//    private final Trigger leftTrigger = new JoystickButton(leftJoystick, Joystick.ButtonType.kTrigger.value);

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
//        Robot.swerveSubsystem.setDefaultCommand(new DriveJoysticks(leftJoystick, rightJoystick));
        Robot.swerveSubsystem.setDefaultCommand(new DriveXboxController(xboxController));
//        Robot.swerveSubsystem.setDefaultCommand(new RunCommand(Robot.swerveSubsystem::vroom){{ addRequirements(Robot.swerveSubsystem); }});
//        Robot.swerveSubsystem.setDefaultCommand(new Test());
    }

    private void configureButtonBindings() {
//        leftTrigger.whileActiveOnce(new InstantCommand(Robot.gyroscope::resetAngle));
        rb.whenPressed(new InstantCommand(Robot.gyroscope::resetAngle));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Robot.swerveSubsystem.setFieldOriented(false);
        Robot.swerveSubsystem.resetOdometry(PathPlanner.loadPath("Rotation Path",
                Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_LINEAR_ACCELERATION).getInitialPose());
        return new PPSwerveControllerCommand(
                PathPlanner.loadPath("Rotation Path",
                        Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_LINEAR_ACCELERATION),
                Robot.swerveSubsystem::getPose,
                new PIDController(Constants.AUTO_XY_Kp, Constants.AUTO_XY_Ki, Constants.AUTO_XY_Kd),
                new PIDController(Constants.AUTO_XY_Kp, Constants.AUTO_XY_Ki, Constants.AUTO_XY_Kd),
                new PIDController(Constants.AUTO_ROTATION_Kp, Constants.AUTO_ROTATION_Ki, Constants.AUTO_ROTATION_Kd),
                Robot.swerveSubsystem::drive,
                Robot.swerveSubsystem
        );
    }
}
