package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class VerticalAimerProfiledPIDSubsystem extends ProfiledPIDSubsystem {
    Encoder encoder;
    CANSparkMax motor;
    public VerticalAimerProfiledPIDSubsystem() {
        super(Constants.LauncherConstants.verticalAimPID);
        motor = new CANSparkMax(Constants.LauncherConstants.verticalAimMotor, MotorType.kBrushless);
        encoder = new Encoder(0, 1);
        motor.getEncoder().setPosition(0);
        encoder.reset();
        encoder.setReverseDirection(false);
        encoder.setDistancePerPulse(360.0/2048.0);
        getController().setTolerance(2);
    }
    @Override
    public void useOutput(double output, State setpoint) {
        motor.set(output);
    }
    @Override
    public double getMeasurement() {
        return getDegrees(encoder.getDistance());
    }
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }
    public static double getDegrees(double ticks) {
        return ticks;
       // return ticks/Constants.LauncherConstants.ticksPerDegree;
    }

    public double getGoal() {
        return getController().getGoal().position;
    }
    public boolean atGoal() {
        System.out.println(getGoal()-getMeasurement());
        boolean atGoal = getGoal()-getMeasurement() < 2 && getGoal()-getMeasurement() > -2;
        return atGoal;
    }
    public State getState() {
        return new State(getMeasurement(),getVelocity());
    }
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Launcher Angle",getMeasurement());
    }
    public void resetPosition() {
        motor.getEncoder().setPosition(0);
        encoder.reset();
    }
}