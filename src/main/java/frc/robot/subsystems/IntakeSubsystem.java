package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    VictorSPX motor;
    boolean hasNote = false;
    public IntakeSubsystem() {
        motor = new VictorSPX(Constants.IntakeConstants.intakeMotorID);
        motor.setInverted(true);
        //motor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID,MotorType.kBrushless);
    }
    public void setSpeed(double speed) {
        motor.set(VictorSPXControlMode.PercentOutput, speed);
       // motor.set(speed);
    }
    public VictorSPX getMotor() {
        return motor;
    }
    public boolean hasNote() {
        return hasNote;
    }
    public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
    }
}
