package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SingleMotorLauncherSubsystem;

public class FireSingleMotorLauncherCommand extends Command {
    SingleMotorLauncherSubsystem m_launcher;
    public FireSingleMotorLauncherCommand(SingleMotorLauncherSubsystem m_launcher) {
        this.m_launcher = m_launcher;
    }
    @Override
    public void initialize() {
        m_launcher.setPower(0.85);
    }
    @Override
    public void end(boolean interrupted) {
        m_launcher.setPower(0);
    }
    
}
