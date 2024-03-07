package frc.robot.subsystems;



import java.util.HashSet;
import java.util.List;
import java.util.function.Supplier; 

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.core.io.NumberInput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class WristProfiledPIDSubsystemThroughBore extends FilteredProfiledPIDSubsystem {

    public static final State intake = new State(210,0);
    public static final State inside = new State(80,0);
    public static final State load = new State(0,0);
    public static final State amp = new State(140,0);
    TalonFX motor;
    //Encoder encoder = new Encoder(0,1); //new
    public WristProfiledPIDSubsystemThroughBore() {
        super(Constants.IntakeConstants.wristBorePIDController,
            intake,
            inside,
            load,
            amp
        );
        motor = new TalonFX(Constants.IntakeConstants.wristMotorID);
        motor.setPosition(0);
      //  encoder.reset(); //new
      //  encoder.setDistancePerPulse(360.0/2048.0); //note: make sure to clarify the numbers are decimal values (float, double) or it will do integer division (bad)
        setTolerance(Constants.IntakeConstants.tolerance);
        //getController().setTolerance(Constants.IntakeConstants.tolerance);
    }
    @Override
    public void useOutput(double output, State setpoint) {
        //This limits the power to within the maxPower using Math.min().
        //The multiplier at the end converts the absolute value back to negative if needed
        double cappedOutput = Math.min(Math.abs(output), Constants.IntakeConstants.maxPower) * (output > 0 ? 1 : -1); 
        motor.set(cappedOutput); //set the motor to the new, capped speed
        SmartDashboard.putNumber("motor out", cappedOutput);
    }
    @Override
    public double getMeasurement() {
      //  return getDegrees(encoder.getDistance()); ///new
        //return getDegrees(position.refresh().getValueAsDouble());
        return 0;
    }
    @Override
    public double getVelocity() {
       // return encoder.getRate(); //gets the velocity (automatically applies the distancePerPulse value too)
       return 0;
    }
    public static double getDegrees(double ticks) {
        return ticks; //conversion done using setDistancePerPulse(), nothing needed here

        //return ticks/Constants.IntakeConstants.throughBorePulsesPerDegree;
        //return ticks/Constants.IntakeConstants.ticksPerDegree;
    }
}
