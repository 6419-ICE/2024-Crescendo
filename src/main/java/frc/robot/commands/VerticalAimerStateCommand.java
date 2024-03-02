package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;

public class VerticalAimerStateCommand extends Command {
    public enum Position {
        load(0),
        fire(-34);
        private double position;
        Position(double position) {
            this.position = position;
        }
        public double getPosition() {
            return position;
        }
        public void setPosition(double pos) {
            if (this != fire) throw new IllegalCallerException("This method should only be used for \"fire\"");
            position = pos;
        }
    }
    private Position pos;
    private VerticalAimerProfiledPIDSubsystem m_aim;
    public VerticalAimerStateCommand(VerticalAimerProfiledPIDSubsystem m_aim2,Position fire) {
        pos = fire;
        this.m_aim = m_aim2;
    }
    @Override
    public void initialize() {
        m_aim.setGoal(pos.getPosition());
        m_aim.enable();
    }
    
}
