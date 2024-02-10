package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax motor;
    public IntakeSubsystem() {
        motor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID,MotorType.kBrushless);
    }
    public void setSpeed(double speed) {
        motor.set(speed);
    }
    public CANSparkMax getMotor() {
        return motor;
    }
}
