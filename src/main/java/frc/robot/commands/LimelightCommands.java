package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCommands {

public static class TurnTo extends PIDCommand {

    private LimelightSubsystem m_limelight;
    private IntSupplier ID;
    private DriveSubsystem m_drive;
    public TurnTo(LimelightSubsystem m_limelight,DriveSubsystem m_drive, IntSupplier ID) {
        super(
            Constants.LimelightConstants.rotatePID,
            m_limelight::getX,
            0,
            (output)->m_drive.turn(0,0, output, false),
            m_drive,m_limelight
        );
        ;
        this.m_limelight = m_limelight;
        this.ID = ID;
        addRequirements(m_limelight,m_drive);
        getController().setTolerance(0.1);
    }
    @Override
    public void initialize() {}
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}

public static class DriveTo extends Command {
    private DriveSubsystem m_drive;
    private LimelightSubsystem m_limelight;
    private PIDController xPID;
    private PIDController yPID;
    private PIDController headingPID;
    public DriveTo(DriveSubsystem m_drive,LimelightSubsystem m_limelight,double distance) {
        //store subsystems and require them
        this.m_drive = m_drive;
        this.m_limelight = m_limelight;
        addRequirements(this.m_drive,this.m_limelight);
        //assign the PID controllers in constants to local variables for easier access, controllers use placeholder values right now (0,0,0)
        xPID = Constants.LimelightConstants.xPID;
        yPID = Constants.LimelightConstants.yPID;
        headingPID = Constants.LimelightConstants.headingPID; //may be able to reuse the TurnTo PID
        //set goals for the PID controllers
        xPID.setSetpoint(distance);
        yPID.setSetpoint(0);
        headingPID.setSetpoint(0);
        //set tolerance, placeholdder values right now
        xPID.setTolerance(Units.inchesToMeters(1));
        yPID.setTolerance(Units.inchesToMeters(1));
        headingPID.setTolerance(0.1);
    }
    @Override
    public void initialize() {}
    @Override
    public void execute() {
        //get the 3d pos of the apriltag from the subsytem
        double[] tagPos = m_limelight.get3dPose();
        
        //xPID uses the Z pos of the aprilTag, but controls the x speed of the drivetrain
        //yPID uses x pos, but controls the y speed
        //headingPID uses the "2d" x output, keeping the apriltag centered with the robot
        m_drive.drive(
            xPID.calculate(tagPos[2]), 
            -yPID.calculate(tagPos[0]), //limelight is on back of robot, so values are reversed
            -headingPID.calculate(tagPos[5]), // right is positive in limelight, reversed to fit with drivetrain
            true
        );
        System.out.printf("X: %f \nY: %f \nHeading: %f\n",tagPos[2],tagPos[0],m_limelight.getX());
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        //finish if all 3 PIDs are at their goal
        return 
        xPID.atSetpoint() && 
        yPID.atSetpoint() && 
        headingPID.atSetpoint();
    }
}
/**
 * Drives to the closest point that is X distance from the main in-view apriltag. This is private to prevent it from being used without {@link TurnTo}. This can only be run through {@link AlignAtDistance}
 * <p>
 * This is done by first centering the apriltag using the {@link TurnTo} command, then moving until the apriltag is at the target Y
 * </p>
 * @see AlignAtDistance
 */
private static class DriveToDistance extends PIDCommand {
    LimelightSubsystem m_limelight;
    DriveSubsystem m_drivetrain;
    double inches;
    public DriveToDistance(LimelightSubsystem limelight,DriveSubsystem driveTrain, double inches) {
        super(
            Constants.LimelightConstants.xPID, //reuse xpid from DriveTo
            limelight::getY,
            0, //placeholder setpoint until I can use conversion method (distanceToTargetHeight())
            (output)->{driveTrain.drive(output, 0, 0, false);} //move back/forward to correct for error
            );
        m_limelight = limelight;   
        m_drivetrain = driveTrain; 
        getController().setTolerance(0.1);
        addRequirements(m_limelight,m_drivetrain);
        this.inches = inches;
    }
    @Override
    public void initialize() {
        super.initialize();
        getController().setSetpoint(distanceToTargetHeight(inches)); //set the actual setpoint
    }
    @Override
    public boolean isFinished() {
        return getController().atSetpoint(); //finish if the controller is done
    }
    /**
     * converts the given distance into the y-position the target needs to be at on-screen
     * @param inches the given distance
     * @return the y-coordinate the apriltag will be at when at that distance
     */
    private double distanceToTargetHeight(double inches) {
        double targetHeight = (-m_limelight.get3dPose()[1]) + Constants.LimelightConstants.limelightHeight; //estimate target height (h2). 
        double h1 = Constants.LimelightConstants.limelightHeight;
        double a2 = Units.degreesToRadians(Constants.LimelightConstants.limelightAngleDegrees); //convert to radians
        return Math.atan((targetHeight - h1) / inches) - a2; // I isolated a1 (the y pos needed) in the equation d = (h2-h1) / tan(a1+a2) to get this (equation from https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance)
    }
}
/**
 * runs {@link DriveToDistance} and {@link TurnTo} in the correct order to drive within a given distance of the main in-view apriltag
 * @see DriveToDistance 
 * @see TurnTo 
 */
public static class AlignAtDistance extends SequentialCommandGroup {
    public AlignAtDistance(LimelightSubsystem m_limelight, DriveSubsystem m_drivetrain,double inches) {
        addCommands(
            new LimelightCommands.TurnTo(m_limelight,m_drivetrain,null), //0 out the x pos
            new LimelightCommands.DriveToDistance(m_limelight, m_drivetrain, inches) //move to the correct distance
        );
        addRequirements(m_limelight,m_drivetrain);
    }
}
}