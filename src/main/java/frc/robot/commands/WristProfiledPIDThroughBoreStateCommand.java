package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystemThroughBore;

public class WristProfiledPIDThroughBoreStateCommand extends ProfiledPIDCommand{
    public enum Position {
        //positions
        intake(180.0),
        load(0.0),
        amp(60.0),
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
    private Position targetPosition;
    private WristProfiledPIDSubsystemThroughBore m_wrist;
    public WristProfiledPIDThroughBoreStateCommand(WristProfiledPIDSubsystemThroughBore m_wrist,Position initialPos) {
        super(
            m_wrist.getController(), //controller
            m_wrist::getMeasurement, //getMeasurement
            m_wrist::getGoal, //setGoal
            m_wrist::useOutput,
            m_wrist
        );
        this.m_wrist = m_wrist;
        targetPosition = initialPos;
    }
    @Override
    public void initialize() {
        setTarget(targetPosition); //refresh subsystem target position
        super.initialize();
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
    @Override
    public void end(boolean interrupted) {}
}
