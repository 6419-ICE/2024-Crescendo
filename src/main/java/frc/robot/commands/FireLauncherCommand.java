package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
/**
 * Command to spin up the launcher. 
 */
public class FireLauncherCommand extends Command {
    private LauncherSubsystem m_launcher;
    private double targetVelocity = 0;
    
    public FireLauncherCommand(LauncherSubsystem m_launcher) {
        this.m_launcher = m_launcher;
        addRequirements(m_launcher);
    }
    
    @Override
    public void initialize() {
        //m_launcher.setStaging(0.2);
        m_launcher.setPower(0.8);
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {
        m_launcher.setPower(0);
    }
    @Override
    public boolean isFinished() {
        return (targetVelocity != 0) && (m_launcher.getAverageVelocity() > targetVelocity);
    }
    /**
     * Makes this command automatically finish once the launcher 
     * {@link LauncherSubsystem#getAverageVelocity() average velocity} reaches/surpasses the given target velocity
     * @param targetVelocity the velocity to end at
     * @return returns itself, allowing method chaining and direct usage as a command
     */
    public FireLauncherCommand untilTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
        return this;
    }
} 
