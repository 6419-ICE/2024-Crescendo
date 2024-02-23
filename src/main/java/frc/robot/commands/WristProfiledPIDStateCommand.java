package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class WristProfiledPIDStateCommand extends ProfiledPIDCommand{
    public enum Position {
        //positions
        intake(180),
        load(0.0),
        amp(0.0),
        inside(0.0);
        //class stuff DONT TOUCH!!!
        private final double pos;
        Position(double position) {
            pos = position;
        }
        public double getPos() {
            return pos;
        }
    }
    private Position targetPosition;
    private WristProfiledPIDSubsystem m_wrist;
    public WristProfiledPIDStateCommand(WristProfiledPIDSubsystem m_wrist,Position initialPos) {
        super(
            m_wrist.getController(), //controller
            m_wrist::getMeasurement, //getMeasurement
            m_wrist::getGoal, //setGoal
            m_wrist::useOutput,
            m_wrist
        );
        this.m_wrist = m_wrist;
        setTarget(initialPos);
    }
    
    public void setTarget(Position target) {
        targetPosition = target;
        m_wrist.setGoal(targetPosition.pos);
    }
    public Position getTarget() {
        return targetPosition;
    }
    public double getPosition() {
        return m_wrist.getMeasurement();
    }
    public boolean atGoal() {
        return m_wrist.atGoal();
    }
    
}
