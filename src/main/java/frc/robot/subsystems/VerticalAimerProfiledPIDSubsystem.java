package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class VerticalAimerProfiledPIDSubsystem extends ProfiledPIDSubsystem {
    CANSparkMax motor;
    public VerticalAimerProfiledPIDSubsystem() {
        super(Constants.LauncherConstants.verticalAimPID);
        motor = new CANSparkMax(Constants.LauncherConstants.verticalAimMotor, MotorType.kBrushless);
        motor.getEncoder().setPosition(0);
        getController().setTolerance(2);
    }
    @Override
    public void useOutput(double output, State setpoint) {
        motor.set(output);
    }
    @Override
    public double getMeasurement() {
        return getDegrees(motor.getEncoder().getPosition());
    }
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }
    public static double getDegrees(double ticks) {
        return ticks/Constants.LauncherConstants.ticksPerDegree;
    }

    public double getGoal() {
        return getController().getGoal().position;
    }
    public boolean atGoal() {
        return getController().atGoal();
    }
    public State getState() {
        return new State(getMeasurement(),getVelocity());
    }
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Launcher Angle",getMeasurement());
    }
}