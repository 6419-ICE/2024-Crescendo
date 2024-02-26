package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class WristStateCommand extends Command {
    public enum Position {
        //positions
        intake(210.0),
        load(0.0),
        amp(140.0),
        inside(90.0); //30
        //class stuff DONT TOUCH!!!
        private final double pos;
        Position(double position) {
            pos = position;
        }
        public double getPos() {
            return pos;
        }
    }
    private WristProfiledPIDSubsystem m_wrist;
    private Position pos;
    public WristStateCommand(WristProfiledPIDSubsystem m_wrist,Position initialPos) {
        pos = initialPos;
        this.m_wrist = m_wrist;
        addRequirements(m_wrist);
    }
    @Override
    public void initialize() {
        m_wrist.setGoal(pos.getPos());
        m_wrist.enable();
    }
}
