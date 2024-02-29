package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
/**
 * subclass of {@link ProfiledPIDSubsystem} that allows for "valid goals" to be given on construction. When an invalid goal is given, 
 * this subsystem will use the last valid goal (or 0 position, 0 velocity) if no previous valid goal exists).
 * This subclass 
 */
public abstract class FilteredProfiledPIDSubsystem extends ProfiledPIDSubsystem {
    private final List<State> validPositions;
    private double tolerance = 0;
    private State prevPosition;
    protected FilteredProfiledPIDSubsystem(ProfiledPIDController controller,State... validPositions) {
        super(controller);
        this.validPositions = List.of(validPositions);
        prevPosition = new State(0,0);
    }
    @Override
    public void periodic() {
        if (!prevPosition.equals(getGoal())) {
            if (validPositions.contains(getGoal())) prevPosition = getGoal();
            else {
                DriverStation.reportWarning("Invalid goal given",new StackTraceElement[]{});
                setGoal(prevPosition);
            }
        }
        super.periodic();
    }
    /**
     * Checks if the given goal is valid or if there are no registered valid positions. If either of these conditions are true, this method sets the goal, otherwise it does nothing
     * @param goal the state to set the goal to
     * @return true if the goal was set, false if the goal was invalid
     * @deprecated this is now built-in, just use {@link #useOutput(double, State) useOutput} normally, I would remove this but I spent more than 2 seconds on this & documented it so im leaving it in
     */
    @Deprecated
    public boolean setValidGoal(State goal) {
        if (validPositions.contains(goal) || validPositions.isEmpty()) {
            setGoal(goal);
            return true;
        }
        return false;
    }
    /**
     * Packages the given goal into a new State with 0 velocity, then calls {@link #setValidGoal(State)} and returns the result
     * @param goal the number to set the goal to
     * @return true if the goal was set, false if the goal was invalid
     * @deprecated this is now built-in, just use {@link #useOutput(double, State) useOutput} normally, I would remove this but I spent more than 2 seconds on this & documented it so im leaving it in
     */
    @Deprecated
    public boolean setValidGoal(double goal) {
        return setValidGoal(new State(goal,0));
    }
    
    public abstract double getVelocity();
     /**
     * packages the values of {@link #getMeasurement()} and {@link #getVelocity()} into a State
     * @return a new State object with values from getMeasurement() and getVelocity()
     */
    public final State getState() {
        return new State(getMeasurement(),getVelocity());
    }
    public final void setTolerance(double tolerance) {
        this.tolerance = tolerance;
        getController().setTolerance(tolerance);
    }
    public final double getTolerance() {
        return tolerance;
    }
    public State getGoal() {
        return getController().getGoal();
    }
    public double getGoalPosition() {
        return getGoal().position;
    }
    public boolean atGoal() {
        double error = getGoalPosition() - getMeasurement();
        double toleranceError = MathUtil.applyDeadband(Math.abs(error), getTolerance()); //The applyDeadband(value, deadband) method returns 0 if the given value (value) is within deadband of 0.
        toleranceError *= (error > 0 ? 1 : -1); //convert absolute value back to normal
        return toleranceError == 0;
    }
}
