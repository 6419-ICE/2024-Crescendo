package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.HangerProfiledSybsystem;

public class HangerPIDStateCommand extends ProfiledPIDCommand{

    private double goal = 0;

    public HangerPIDStateCommand(HangerProfiledSybsystem m_hanger) {
        super(m_hanger.getController(), m_hanger::getMeasurement, 0, m_hanger::useOutput, m_hanger);
        m_goal  = ()-> new State(goal, 0);
    }

    public void setGoal(double goal) {
        this.goal = goal;
    }
    public double getGoal() {
        return goal;
    }
}