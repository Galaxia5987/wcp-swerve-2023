// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.valuetuner.NetworkTableConstant;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.LogSocketServer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    public static final Gyroscope gyroscope = new Gyroscope();
    public static final SwerveDrive swerveSubsystem = new SwerveDrive();

    public static boolean debug = false;

    private RobotContainer robotContainer;
    private Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = RobotContainer.getInstance();
        autonomousCommand = robotContainer.getAutonomousCommand();

        setUseTiming(isReal()); // Run as fast as possible during replay
        LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables are logged by default).
        Logger.getInstance().recordMetadata("ProjectName", "Wcp-Swerve-2023"); // Set a metadata value

        Logger.getInstance().addDataReceiver(new ByteLogReceiver("/media/sda1/")); // Log to USB stick (name will be selected automatically)
        Logger.getInstance().addDataReceiver(new LogSocketServer(5804)); // Provide log data over the network, viewable in Advantage Scope.

        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        NetworkTableConstant.initializeAllConstants();

        SmartDashboard.putBoolean("Zero Swerve", false);
        SmartDashboard.putBoolean("Swerve Tune Motion Magic", false);
        SmartDashboard.putNumber("PathFollowerCommand_xyKp", Constants.AUTO_XY_Kp);
        SmartDashboard.putNumber("PathFollowerCommand_xyKi", Constants.AUTO_XY_Ki);
        SmartDashboard.putNumber("PathFollowerCommand_xyKd", Constants.AUTO_XY_Kd);
        SmartDashboard.putNumber("PathFollowerCommand_xyKf", Constants.AUTO_XY_Kf);
        SmartDashboard.putNumber("PathFollowerCommand_rotationKp", Constants.AUTO_ROTATION_Kp);
        SmartDashboard.putNumber("PathFollowerCommand_rotationKi", Constants.AUTO_ROTATION_Ki);
        SmartDashboard.putNumber("PathFollowerCommand_rotationKd", Constants.AUTO_ROTATION_Kd);
        SmartDashboard.putNumber("PathFollowerCommand_rotationKf", Constants.AUTO_ROTATION_Kf);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        LoggedSubsystem.getSubsystems().forEach(LoggedSubsystem::updateSubsystem);
        CommandScheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
