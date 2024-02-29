package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase{
    
    private TalonFX m_hanger;
    
    public HangerSubsystem() {
        m_hanger = new TalonFX(Constants.HangerConstants.hangerMotorID);
    }
    
    public void setPower(double speed) {
        m_hanger.set(speed);
    }

    public double getPower(){
        return m_hanger.get();
    }
}
