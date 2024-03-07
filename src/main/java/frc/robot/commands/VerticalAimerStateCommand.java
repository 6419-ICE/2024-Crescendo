package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;

public class VerticalAimerStateCommand extends Command {
    public enum Position {
        load(0),
        fire(0),
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
    private boolean runOnce = false;
    private Position pos;
    private VerticalAimerProfiledPIDSubsystem m_aim;
    /**
     * NOTE: when using this command for only one movement (like in auto), the {@link #once()} method can be used to end the command automatically upon reaching the intialpos
     * @param m_aim Vertical Aimer Subsystem
     * @param initialPos Goal to go to when the command starts
     */
    public VerticalAimerStateCommand(VerticalAimerProfiledPIDSubsystem m_aim,Position initialPos) {
        pos = initialPos;
        this.m_aim = m_aim;
        runOnce = false;
    }
    @Override
    public void initialize() {
        m_aim.setGoal(pos.getPosition());
        m_aim.enable();
    }
    /**
     * makes this command run until at goal, then automatically finish. Useful for Autonomous.
     * @return returns itself, allowing for this method to be stacked with others, and used directly when passing commands into groups.
     */
    public VerticalAimerStateCommand once() {
        runOnce = true;
        return this;
    }
    @Override
    public boolean isFinished() { 
        return runOnce && m_aim.atGoal(); //if runOnce is false, this command never naturally finishes
    }
}
