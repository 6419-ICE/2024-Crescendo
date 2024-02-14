package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightTestCommand extends Command {
    LimelightSubsystem m_limeLight;

    public LimelightTestCommand(LimelightSubsystem m_limeLight) {
        this.m_limeLight = m_limeLight;
        addRequirements(this.m_limeLight);
    }
    @Override
    public void initialize() {}
    @Override
    public void execute() {
        SmartDashboard.putString("HostName", m_limeLight.getHostName());
        SmartDashboard.putNumberArray("AprilTag Position", new double[]{m_limeLight.getX(),m_limeLight.getY()});
        SmartDashboard.putNumber("Area",m_limeLight.getArea());
        SmartDashboard.updateValues();
    }
    @Override
    public void end(boolean interrupted) {}
}
