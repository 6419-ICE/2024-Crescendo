package frc.robot.commands;

import java.lang.annotation.Inherited;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;

public class ArmStateCommand extends Command {
    public enum Position {
        //positions
        intake(0.0),
        load(0.0),
        amp(155.0), //TODO change for new robot
        balance(165), //TODO change for new robot
        trap(125),
        ampLow(0), //25->18->25
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
    private boolean runOnce;
    private ArmProfiledPIDSubsystem m_arm;
    private Position pos;
    public ArmStateCommand(ArmProfiledPIDSubsystem m_arm,Position initialPos) {
        pos = initialPos;
        this.m_arm = m_arm;
        addRequirements(m_arm);
        runOnce = false;
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
    /**
     * look at {@link VerticalAimerStateCommand#once()} for info on what this means 
     * (Im too lazy to write the same docs multiple times. I might make this part of an abstract class at some point)
     * @see VerticalAimerStateCommand#once()
     */
    public ArmStateCommand once() {
        runOnce = true;
        return this;
    }
    @Override
    public boolean isFinished() {
        return runOnce && m_arm.atGoal();
    }
}
