package frc.robot.subsystems.drivetrain.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class JoystickDrive extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Joystick joystick1;
    private final Joystick joystick2;

    public JoystickDrive(Joystick joystick1, Joystick joystick2){
        this.joystick1 = joystick1;
        this.joystick2 = joystick2;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(
                    MathUtil.applyDeadband(-joystick1.getY(), 0.1),
                    MathUtil.applyDeadband(-joystick1.getX(), 0.1),
                    MathUtil.applyDeadband(-joystick2.getX(), 0.1),
                true
        );
    }
}