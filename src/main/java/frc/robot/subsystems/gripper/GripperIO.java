package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {

    void updateInputs(GripperInputs inputs);

    boolean getState();

    void open();

    void close();

    void toggle();

    @AutoLog
    class GripperInputs{
        public boolean isOpen = false;
    }
}
