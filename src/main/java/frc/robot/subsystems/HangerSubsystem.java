package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase{
    
    //private TalonFX m_hanger;
    private VictorSPX motor;
    public HangerSubsystem() {
        motor = new VictorSPX(Constants.HangerConstants.hangerMotorID);
        motor.setNeutralMode(NeutralMode.Brake);
        //m_hanger = new TalonFX(Constants.HangerConstants.hangerMotorID);
    }
    
    public void setPower(double speed) {
        //m_hanger.set(speed);
        motor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public double getPower(){
        return motor.getMotorOutputPercent();
        //return m_hanger.get();
    }
}
