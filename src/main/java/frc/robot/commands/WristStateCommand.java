package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class WristStateCommand extends Command {
    public enum Position {
        //positions // -14 when hard stops are put in
        intake(200.0), //210->205
        load(14.0), //0
        amp(107.0), //140 TODO change this for new robot
        balance(110.0),//90 TODO change this for new robot
        trap(45.0), //TODO find actual value for this
        ampLow(90.0), //85 //109->105
        inside(45.0); //90->60
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
    private WristProfiledPIDSubsystem m_wrist;
    private Position pos;
    public WristStateCommand(WristProfiledPIDSubsystem m_wrist,Position initialPos) {
        pos = initialPos;
        this.m_wrist = m_wrist;
        addRequirements(m_wrist);
        runOnce = false;
    }
    @Override
    public void initialize() {
        m_wrist.setGoal(pos.getPos());
        m_wrist.enable();
    }
    /**
     * look at {@link VerticalAimerStateCommand#once()} for info on what this means 
     * (Im too lazy to write the same docs multiple times. I might make this part of an abstract class at some point)
     * @see VerticalAimerStateCommand#once()
     */
    public WristStateCommand once() {
        runOnce = true;
        return this;
    }
    @Override
    public boolean isFinished() {
        return runOnce && m_wrist.atGoal();
    }
    
}
