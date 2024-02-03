package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SingleMotorLauncherSubsystem extends SubsystemBase {
    TalonFX motor;
    public SingleMotorLauncherSubsystem() {
        motor = new TalonFX(Constants.LauncherConstants.singleMotorID);
    }
    public void setPower(double speed) {
        motor.set(speed);
    }
}
