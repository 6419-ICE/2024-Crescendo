package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class FireLauncherCommand extends Command {
    LauncherSubsystem m_launcher;
    boolean isFinished = false;
    Timer time = new Timer(); // used to keep track of time since Command start
    public FireLauncherCommand(LauncherSubsystem m_launcher) {
        this.m_launcher = m_launcher;
        addRequirements(m_launcher);
        //Make sure the timer doesnt start until init
        time.stop();
    }
    @Override
    public void initialize() {
        //m_launcher.setStaging(0.2);
        m_launcher.setPower(0.8);
        //start the timer
        time.restart();
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {
        m_launcher.setPower(0);
    }
    @Override
    public boolean isFinished() {
        //stop after 0.5 seconds
        return time.hasElapsed(999);
    }
} 
