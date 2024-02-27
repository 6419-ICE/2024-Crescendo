package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class ArmProfiledPIDSubsystem extends ProfiledPIDSubsystem {
    TalonFX motor;
    public ArmProfiledPIDSubsystem() {
        super(Constants.ArmConstants.controller);
        motor = new TalonFX(Constants.ArmConstants.motorID);
    }

    @Override
    public void useOutput(double output, State setpoint) {
        motor.set(output);
    }

    @Override
    public double getMeasurement() {
        SmartDashboard.putString("Motor signal",motor.getPosition().getStatus().getName());
        SmartDashboard.putString("Desc", motor.getPosition().getStatus().getDescription());
        SmartDashboard.updateValues();
        return motor.getPosition().getValueAsDouble();
    }
    
}
