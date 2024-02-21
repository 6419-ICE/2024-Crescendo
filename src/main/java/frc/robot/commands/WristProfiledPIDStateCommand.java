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
        intake(0.0),
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
    public WristProfiledPIDStateCommand(WristProfiledPIDSubsystem m_wrist,Position initialPos) {
        super(
            m_wrist.getController(), //controller
            m_wrist::getMeasurement, //getMeasurement
            initialPos::getPos, //setGoal
            m_wrist::useOutput,
            m_wrist
        );
            targetPosition = initialPos;
            m_goal = ()-> new State(targetPosition.getPos(),0); //set goal to use targetPos instead of initial
    }
    public void setTarget(Position target) {
        targetPosition = target;
    }
    public Position getTarget() {
        return targetPosition;
    }
    
}
