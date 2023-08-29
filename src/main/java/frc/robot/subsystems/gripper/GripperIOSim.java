package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import frc.robot.Ports;

public class GripperIOSim implements GripperIO{
    private final SolenoidSim solenoid = new SolenoidSim(PneumaticsModuleType.CTREPCM, Ports.Gripper.SOLENOID_ID);

    public GripperIOSim(){
    }

    @Override
    public void updateInputs(GripperInputs inputs) {
        inputs.isOpen = getState();
    }

    @Override
    public boolean getState() {
        return solenoid.getOutput();
    }

    @Override
    public void open() {
        solenoid.setOutput(true);
    }

    @Override
    public void close() {
        solenoid.setOutput(false);
    }

    @Override
    public void toggle() {
        solenoid.setOutput(!getState());
    }
}
