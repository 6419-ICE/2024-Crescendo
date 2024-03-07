package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;

/**
 * @deprecated use {@link VerticalAimerStateCommand} instead
 */
@Deprecated
public class VerticalAimerProfiledPIDStateCommand extends ProfiledPIDCommand {

    public enum Position {
        //positions
        load(0.0),
        fire(0.0);
        //class stuff DONT TOUCH!!!
        private final double pos;
        Position(double position) {
            pos = position;
        }
        public double getPos() {
            return pos;
        }
    }
    private Position currentPos;
    private VerticalAimerProfiledPIDSubsystem m_aim;
    public VerticalAimerProfiledPIDStateCommand(VerticalAimerProfiledPIDSubsystem m_aim,Position initialPos) {
        super(m_aim.getController(), 
        m_aim::getMeasurement, 
        m_aim::getGoal, 
        m_aim::useOutput, 
        m_aim);
        this.m_aim = m_aim;
        setTarget(initialPos);
    }
    @Override
    public void initialize() {
        setTarget(currentPos); //refresh subsystem
        super.initialize();
    }
    public void setTarget(Position target) {
        currentPos = target;
        m_aim.setGoal(currentPos.pos);
    }
    public Position getTarget() {
        return currentPos;
    }
    public double getPosition() {
        return m_aim.getMeasurement();
    }
    public boolean atGoal() {
        return m_aim.atGoal();
    }
    
}
