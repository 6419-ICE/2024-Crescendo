package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class IntakeProfiledPIDSubystem extends ProfiledPIDSubsystem {
    CANSparkMax motor;

    public IntakeProfiledPIDSubystem() {
        super(Constants.IntakeConstants.pidController);
        motor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID,MotorType.kBrushless);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        motor.set(output);
    }

    @Override
    protected double getMeasurement() {
        return getPosition();
    }
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }
    
}
