package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.List;

public abstract class LoggedSubsystem extends SubsystemBase {
    private static final List<LoggedSubsystem> subsystems = new ArrayList<>();
    private final LoggableInputs loggerInputs;

    public LoggedSubsystem(LoggableInputs inputs) {
        subsystems.add(this);
        loggerInputs = inputs;
    }

    public static List<LoggedSubsystem> getSubsystems() {
        return subsystems;
    }

    public LoggableInputs getLoggerInputs() {
        return loggerInputs;
    }

    public void updateSubsystem() {
        updateInputs();
        Logger.getInstance().processInputs(getSubsystemName(), loggerInputs);
    }

    public abstract void updateInputs();

    public abstract String getSubsystemName();
}
