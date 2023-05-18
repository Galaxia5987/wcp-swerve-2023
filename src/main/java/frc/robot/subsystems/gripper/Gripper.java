package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

import javax.sound.sampled.Port;

public class Gripper extends SubsystemBase {
    private static final Gripper INSTANCE = null;

    private final TalonFX mainMotor = new TalonFX(Ports.GripperPorts.GRIPPER_MAIN_MOTOR);
    private final TalonSRX spinMotorLeft = new TalonSRX(Ports.GripperPorts.GRIPPER_SPIN_MOTOR_LEFT);
    private final TalonSRX spinMotorRight = new TalonSRX(Ports.GripperPorts.GRIPPER_SPIN_MOTOR_RIGHT);

    public Gripper() {

    }
}
