package frc.robot.subsystems;

import java.nio.file.FileSystem;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.Filesystem;
>>>>>>> origin/JackCorsoBranch
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class ArmProfiledPIDSubsystem extends ProfiledPIDSubsystem {
    TalonFX motor;
    StatusSignal<Double> encoderVal;
    StatusSignal<Double> velocityVal;
    public ArmProfiledPIDSubsystem() {
        super(Constants.ArmConstants.controller);
        motor = new TalonFX(Constants.ArmConstants.motorID);
        motor.setPosition(0);
        encoderVal = motor.getPosition();
        velocityVal = motor.getVelocity();
    }
    @Override
    public void useOutput(double output, State setpoint) {
        motor.set(Math.max(Math.abs(output),Constants.ArmConstants.minPower)*(output > 0 ? 1 : -1));
    }

    @Override
    public double getMeasurement() {
<<<<<<< HEAD
        SmartDashboard.putString("Motor signal",motor.getPosition().getStatus().getName());
        SmartDashboard.putString("Desc", motor.getPosition().getStatus().getDescription());
        SmartDashboard.updateValues();
        return motor.getPosition().getValueAsDouble();
=======
        return getDegrees(encoderVal.refresh().getValueAsDouble());
    }
    public double getVelocity() {
        return velocityVal.getValueAsDouble();
    }
    public static double getDegrees(double ticks) {
        return ticks/Constants.ArmConstants.ticksPerDegree;
    }
    //setGoal already exists in super class
    public double getGoal() {
        return getController().getGoal().position;
    }
    public boolean atGoal() {
       double error = MathUtil.applyDeadband(getGoal() - getMeasurement(),0.5);
        return error == 0;
    }
    /**
     * 
     * @return state containing the current position (in degrees) and current velocity of the motor
     */
    public State getState() {
        return new State(getMeasurement(),getVelocity());
>>>>>>> origin/JackCorsoBranch
    }
}
