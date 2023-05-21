package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE = null;

    private final TalonFX mainMotor = new TalonFX(Ports.GripperPorts.GRIPPER_MAIN_MOTOR);
    private final TalonSRX mainSpinMotor = new TalonSRX(Ports.GripperPorts.GRIPPER_MAIN_SPIN_MOTOR);
    private final TalonSRX auxSpinMotor = new TalonSRX(Ports.GripperPorts.GRIPPER_AUX_SPIN_MOTOR);

    public Gripper() {
        mainMotor.setNeutralMode(NeutralMode.Coast);
        mainMotor.enableVoltageCompensation(true);
        mainMotor.configVoltageCompSaturation(GripperConstants.VOLT_COMP_SATURATION);
        mainMotor.config_kP(0, GripperConstants.kP, GripperConstants.FALCON_TIMEOUT);
        mainMotor.config_kI(0, GripperConstants.kI, GripperConstants.FALCON_TIMEOUT);
        mainMotor.config_kD(0, GripperConstants.kD, GripperConstants.FALCON_TIMEOUT);

        mainSpinMotor.enableVoltageCompensation(true);
        mainSpinMotor.configVoltageCompSaturation(GripperConstants.VOLT_COMP_SATURATION);
        mainSpinMotor.setNeutralMode(NeutralMode.Coast);
        mainSpinMotor.setInverted(true);

        auxSpinMotor.enableVoltageCompensation(true);
        auxSpinMotor.configVoltageCompSaturation(GripperConstants.VOLT_COMP_SATURATION);
        auxSpinMotor.setNeutralMode(NeutralMode.Coast);
        auxSpinMotor.follow(mainSpinMotor);
        auxSpinMotor.setInverted(InvertType.OpposeMaster);
    }

    public static Gripper getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Gripper();
        }
        return INSTANCE;
    }

    public void setMainMotorPower(double power) {
        mainMotor.set(ControlMode.PercentOutput, power);
    }

    public void setMainMotorPower(double power, double timeOut) {
        mainMotor.set(ControlMode.PercentOutput, power);
    }

    public double getMainMotorPower() {
        return mainMotor.getMotorOutputPercent();
    }

    public void setSpinPower(double power) {
        mainSpinMotor.set(ControlMode.PercentOutput, power);
    }

    public double getSpinPower() {
        return mainSpinMotor.getMotorOutputPercent();
    }

    public double getPosition(){
        return mainMotor.getSelectedSensorPosition();
    }

    public void setPosition(double position){
        mainMotor.set(ControlMode.Position, position);
    }

    public void resetEncoder(){
        mainMotor.setSelectedSensorPosition(0);
    }

//    public void sing(){
//        mainMotor.set(ControlMode.MusicTone, );
//    }

    public void printValues(double value){
        System.out.println(getPosition());
    }
}
