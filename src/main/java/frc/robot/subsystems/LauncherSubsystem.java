package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
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
        //leftMotor.setInverted(true); this isnt working??? maybe its already inversed so I need to set it false?
    
    }
    public void setPower(double speed) {
        rightMotor.set(-speed);
        leftMotor.set(-speed); //inversed didnt work, so I reverse the speed
    }
    public void setVelocity(double velo) {
        rightMotor.setControl(new VelocityVoltage(velo));
        leftMotor.setControl(new VelocityVoltage(-velo));
    }
}
