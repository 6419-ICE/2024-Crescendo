package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    public LauncherSubsystem() {
        leftMotor = new TalonFX(Constants.LauncherConstants.leftMotorID);
        rightMotor = new TalonFX(Constants.LauncherConstants.rightMotorID);
       // rightMotor.setInverted(true); not inverted, motors are working together 
    
    }
    public void setPower(double speed) {
        rightMotor.set(speed);
        leftMotor.set(speed);
    }
}
