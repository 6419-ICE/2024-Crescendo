package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;

public class ArmStateCommand extends Command {
    public enum Position {
        //positions
        intake(0.0),
        load(0.0),
        amp(155.0),
        balance(90),
        inside(0.0); //30
        //class stuff DONT TOUCH!!!
        private final double pos;
        Position(double position) {
            pos = position;
        }
        public double getPos() {
            return pos;
        }
    }
    private ArmProfiledPIDSubsystem m_arm;
    private Position pos;
    public ArmStateCommand(ArmProfiledPIDSubsystem m_arm,Position initialPos) {
        pos = initialPos;
        this.m_arm = m_arm;
        addRequirements(m_arm);
    }
    @Override
    public void initialize() {
        m_arm.setGoal(pos.getPos());
        m_arm.enable();
    }
    public void setTarget(Position target) {
        pos = target;
    }
    public Position getTarget() {
        return pos;
    }
}
