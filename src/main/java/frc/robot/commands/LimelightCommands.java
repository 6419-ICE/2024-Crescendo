package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
        xPID.setTolerance(0.1);
        yPID.setTolerance(0.1);
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
            yPID.calculate(tagPos[0]), 
            headingPID.calculate(m_limelight.getX()), false
        );
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
}