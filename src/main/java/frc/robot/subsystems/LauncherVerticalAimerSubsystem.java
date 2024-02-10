package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class LauncherVerticalAimerSubsystem extends PIDSubsystem {
    private CANSparkMax motor;
    public LauncherVerticalAimerSubsystem() {
        super(Constants.LauncherConstants.verticalAimPID);
        motor = new CANSparkMax(Constants.LauncherConstants.verticalAimMotor, MotorType.kBrushless);
        motor.getEncoder().setPositionConversionFactor(1/360); //Makes the encoder return degrees instead of rotations. Change this if ratio isnt 1:1
        disable();
        getController().setTolerance(5);
    }
    /**
     * Enables the PID controller and sets the setpoint to the given argument, making the motor start moving to that point
     * @param setPoint the setpoint to give to the PID Controller
     */
    public void start(double setPoint) {
        setSetpoint(setPoint);
        enable();
    }
    @Override
    protected void useOutput(double output, double setpoint) {
        motor.set(output);
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return motor.getEncoder().getPosition();
    }
    
}
