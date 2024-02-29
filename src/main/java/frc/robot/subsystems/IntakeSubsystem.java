package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax motor;
    boolean hasNote = false;
    public IntakeSubsystem() {
        motor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID,MotorType.kBrushless);
    }
    public void setSpeed(double speed) {
        motor.set(speed);
    }
    public CANSparkMax getMotor() {
        return motor;
    }
    public boolean hasNote() {
        return hasNote;
    }
    public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
    }
}
