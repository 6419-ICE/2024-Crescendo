package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;

public class VerticalAimerStateCommand extends Command {
    public enum Position {
        load(0),
        fire(-34),
        down(-48); //TODO TEST THIS
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
        public static Position toPosition(double pos) {
            if (pos == Position.load.getPosition()) return Position.load;
            else if (pos == Position.fire.getPosition()) return Position.fire;
            else if (pos == Position.down.getPosition()) return Position.down;
            return null;
        }
    }
    private Position pos;
    private VerticalAimerProfiledPIDSubsystem m_aim;
    public VerticalAimerStateCommand(VerticalAimerProfiledPIDSubsystem m_aim,Position initialPos) {
        pos = initialPos;
        this.m_aim = m_aim;
    }
    @Override
    public void initialize() {
        m_aim.setGoal(pos.getPosition());
        m_aim.enable();
    }
    
}
