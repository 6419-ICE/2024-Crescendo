package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherStagingSubsystem extends SubsystemBase {
    private CANSparkMax motor;
    public LauncherStagingSubsystem() {
        motor = new CANSparkMax(0, MotorType.kBrushless);
    }
    public void setPower(double power) {
        motor.set(power);
    }
}
