package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;

public class GripperIOReal implements GripperIO{

    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.SOLENOID_ID);

    public GripperIOReal(){

    }
}
