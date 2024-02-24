package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;

public class ArmProfiledPIDStateCommand extends ProfiledPIDCommand {

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

    private ArmProfiledPIDSubsystem m_arm;
    private Position currentPos;
    public ArmProfiledPIDStateCommand(ArmProfiledPIDSubsystem m_arm,Position initialPos) {
        super(
            m_arm.getController(),
            m_arm::getMeasurement,
            m_arm::getGoal,
            m_arm::useOutput,
            m_arm
        );
        this.m_arm = m_arm;
        setTarget(initialPos);
    }
    @Override
    public void initialize() {
        setTarget(currentPos); //refresh subsystem
        super.initialize();
    }
    public void setTarget(Position target) {
        currentPos = target;
        m_arm.setGoal(currentPos.pos);
    }
    public Position getTarget() {
        return currentPos;
    }
    public double getPosition() {
        return m_arm.getMeasurement();
    }
    public boolean atGoal() {
        return m_arm.atGoal();
    }
}
