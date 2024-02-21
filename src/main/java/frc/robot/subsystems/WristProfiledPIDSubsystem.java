package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class WristProfiledPIDSubsystem extends ProfiledPIDSubsystem {
    TalonFX motor;
    public WristProfiledPIDSubsystem() {
        super(Constants.IntakeConstants.wristPIDController);
        motor = new TalonFX(Constants.IntakeConstants.wristMotorID);
    }

    @Override
    public void useOutput(double output, State setpoint) {
        motor.set(output);
    }

    @Override
    public double getMeasurement() {
        return motor.getPosition().getValueAsDouble(); //should work?
    }
    
}
