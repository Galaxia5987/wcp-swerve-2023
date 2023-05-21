package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.utils.units.UnitModel;

public class Elevator extends SubsystemBase {
    private static Elevator INSTANCE = null;
    private final TalonFX motor = new TalonFX(Ports.ELE_MOTOR);
    private final UnitModel unitModel = new UnitModel(ElevatorConstants.TICKS_PER_METER);

    public Elevator() {
        motor.setInverted(ElevatorConstants.INVERT_TYPE);
        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(ElevatorConstants.VOLT_COMP_SATURATION);
        motor.config_kP(0, ElevatorConstants.kP, ElevatorConstants.FALCON_TIMEOUT);
        motor.config_kI(0, ElevatorConstants.kI, ElevatorConstants.FALCON_TIMEOUT);
        motor.config_kD(0, ElevatorConstants.kD, ElevatorConstants.FALCON_TIMEOUT);
        motor.config_kF(0, ElevatorConstants.kF, ElevatorConstants.FALCON_TIMEOUT);
    }

    public static Elevator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Elevator();
        }
        return INSTANCE;
    }

    /**
     * Get the position of the elevator
     * @return [m]
     */
    public double getPosition() {
        return motor.getSelectedSensorPosition();
    }

    /**
     * Set the position of the elevator
     * @param position [ticks]
     */
    public void setPosition(double position){
        motor.set(ControlMode.Position, position);
    }

    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public void resetEncoder(){
        motor.setSelectedSensorPosition(0);
    }

    public void printPosition(){
        System.out.println(getPosition());
    }

}
