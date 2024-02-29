package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServerShared;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

@Deprecated
public class HangerProfiledSybsystem extends ProfiledPIDSubsystem{

    CANSparkMax m_hangerMotor;

    public HangerProfiledSybsystem() {
        super(Constants.HangerConstants.hangerController);
        m_hangerMotor = new CANSparkMax(1, MotorType.kBrushless);
    }

    @Override
    public void useOutput(double output, State setpoint) {
        m_hangerMotor.set(output);
    }

    @Override
    public double getMeasurement() {
        SmartDashboard.putNumber("Hanger Position: ", m_hangerMotor.getEncoder().getPosition());
        return m_hangerMotor.getEncoder().getPosition();
    }
}
