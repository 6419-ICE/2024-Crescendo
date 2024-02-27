package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class WristProfiledPIDSubsystem extends ProfiledPIDSubsystem {
    TalonFX motor;
    StatusSignal<Double> position;
    StatusSignal<Double> velocity;
    public WristProfiledPIDSubsystem() {
        super(Constants.IntakeConstants.wristPIDController);
        motor = new TalonFX(Constants.IntakeConstants.wristMotorID);
        motor.setPosition(0);
        position = motor.getPosition();
        velocity = motor.getVelocity();
        disable();
        //getController().setTolerance(Constants.IntakeConstants.tolerance);
    }
    @Override
    public void useOutput(double output, State setpoint) {
        motor.set(Math.max(Math.abs(output),Constants.IntakeConstants.minPower) * (output > 0 ? 1 : -1));
    }
    
    @Override
    public double getMeasurement() {
        //return getDegrees(encoder.getDistance()); ///new
        return getDegrees(position.refresh().getValueAsDouble());
    }
    public double getVelocity() {
        return velocity.getValueAsDouble();
    }
    public State getState() {
        return new State(getMeasurement(),getVelocity());
    }
    public double getGoal() {
        return getController().getGoal().position;
    }
    public boolean atGoal() {
        double error = MathUtil.applyDeadband(getGoal() - getMeasurement(),Constants.IntakeConstants.tolerance);
        return error == 0;
    }
    public static double getDegrees(double ticks) {
        //return ticks/Constants.IntakeConstants.throughBorePulsesPerDegree;
        return ticks/Constants.IntakeConstants.ticksPerDegree;
    }
}
