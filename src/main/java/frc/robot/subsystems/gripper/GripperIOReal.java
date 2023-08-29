package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;

public class GripperIOReal implements GripperIO{
    private GripperIOReal INSTANCE = null;
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.SOLENOID_ID);

    public GripperIOReal(){
    }

    public GripperIOReal getInstance(){
        if (INSTANCE == null){
            INSTANCE = new GripperIOReal();
        }
        return INSTANCE;
    }
    @Override
    public void updateInputs(GripperInputs inputs) {
        inputs.isOpen = getState();
    }

    @Override
    public boolean getState() {
        return solenoid.get();
    }

    @Override
    public void open() {
        solenoid.set(true);
    }

    @Override
    public void close() {
        solenoid.set(false);
    }

    @Override
    public void toggle() {
        solenoid.toggle();
    }
}
