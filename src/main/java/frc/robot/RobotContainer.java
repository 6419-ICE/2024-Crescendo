// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmProfiledPIDStateCommand;
import frc.robot.commands.ArmStateCommand;
import frc.robot.commands.ArmTestAuto;
import frc.robot.commands.AutoDriveOutOfCommunity;
import frc.robot.commands.AutoTestForPaths;
import frc.robot.commands.DistanceTestCommand;
import frc.robot.commands.EncoderTest;
import frc.robot.commands.FireLauncherCommand;
import frc.robot.commands.FireSingleMotorLauncherCommand;
import frc.robot.commands.IntakeStateCommand;
import frc.robot.commands.LauncherAngleTestAuto;
import frc.robot.commands.LimelightCommands;
import frc.robot.commands.LimelightTestCommand;
import frc.robot.commands.MoveArmAndWristCommand;
import frc.robot.commands.PathWeaverTestAuto;
import frc.robot.commands.VerticalAimerStateCommand;
import frc.robot.commands.WristStateCommand;
import frc.robot.commands.ArmProfiledPIDStateCommand.Position;
import frc.robot.commands.IntakeStateCommand.State;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SingleMotorLauncherSubsystem;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystemThroughBore;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimelightSubsystem m_limeLightChassis = new LimelightSubsystem(Constants.LimelightConstants.chassisHostName);
  private final LimelightSubsystem m_limeLightTurret = new LimelightSubsystem(Constants.LimelightConstants.turretHostName);
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final SingleMotorLauncherSubsystem m_singleMotorLauncher = new SingleMotorLauncherSubsystem();
  private final ArmProfiledPIDSubsystem m_arm = new ArmProfiledPIDSubsystem();
  private final WristProfiledPIDSubsystem m_wrist = new WristProfiledPIDSubsystem();
  private final WristProfiledPIDSubsystemThroughBore m_wristBore = new WristProfiledPIDSubsystemThroughBore();
  private final VerticalAimerProfiledPIDSubsystem m_aim = new VerticalAimerProfiledPIDSubsystem();
  //private final Arm m_Arm = new Arm();
  // The driver's controller
  static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  static Joystick mechanismJoystick = new Joystick(Constants.ButtonBoxID);
  static JoystickButton coneFlipperUpButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperUp);
  static JoystickButton coneFlipperDownButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperDown);
  static JoystickButton GrabberOpenButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.GrabberOpen);
  static JoystickButton GrabberCloseButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.GrabberClose);
  //private HandleArm handleArm = new HandleArm(m_Arm);
  private static DutyCycleEncoder coneFlipperEncoder = new DutyCycleEncoder(Constants.FlipperEncoderID);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private static SendableChooser<Command> autoChooser;
  private static SendableChooser<Boolean> alllianceChooser;
  public static GenericEntry kpEntry;
  public static GenericEntry kiEntry;
  public static GenericEntry kdEntry;
  public RobotContainer() {
    alllianceChooser = new SendableChooser<>();
    alllianceChooser.setDefaultOption("None", null);
    alllianceChooser.addOption("Red", true);
    alllianceChooser.addOption("Blue", false);
    // Configure the button bindings
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Limelight test",new LimelightTestCommand(m_limeLightChassis).alongWith(new LimelightTestCommand(m_limeLightTurret)));
    //autoChooser.addOption("Auto Drive Out Of Community", new AutoDriveOutOfCommunity(m_robotDrive));
    autoChooser.addOption("Auto Test For Paths", new AutoTestForPaths(m_robotDrive));
    autoChooser.addOption("Turn to", new LimelightCommands.TurnTo(m_limeLightTurret, m_robotDrive, null));
    autoChooser.addOption("Launch", new FireLauncherCommand(m_launcher));
    autoChooser.addOption("Single Motor Launch", new FireSingleMotorLauncherCommand(m_singleMotorLauncher));
    autoChooser.addOption("align at distance",new LimelightCommands.AlignAtDistance(m_limeLightTurret, m_robotDrive, Units.metersToInches(2)));
    autoChooser.addOption("Move 2 meter", new DistanceTestCommand(m_robotDrive));
    autoChooser.addOption("PathWeaver test", new PathWeaverTestAuto(m_robotDrive,m_launcher));
    autoChooser.addOption("Arm test", new ArmTestAuto(m_wrist,m_arm));
    autoChooser.addOption("launch test", new LauncherAngleTestAuto(m_aim));
    autoChooser.addOption("encoder test", new EncoderTest(m_wristBore));
      // autoChooser.addOption("Auto Engage on Charging Station Center", new AutoEngageOnChargingStation(m_robotDrive));
    //autoChooser.addOption("Auto Charge on Charging Station Left", new AutoDriveOutAndChargeLeft(m_robotDrive));
    //autoChooser.addOption("Auto Charge on Charging Station Right ", new AutoDriveOutAndChargeRight(m_robotDrive));
    //autoChooser.addOption("Auto Run Until Angle", new AutoDriveUntilAngle(m_robotDrive, boolSupplier));
    SmartDashboard.putData("Autonomous", autoChooser);
    SmartDashboard.putData("Alliance",alllianceChooser);
    kpEntry = Shuffleboard.getTab("SmartDashboard")
   .add("kp", 1)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
   .getEntry();
   kiEntry = Shuffleboard.getTab("SmartDashboard")
   .add("ki", 1)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
   .getEntry();
   kdEntry =  Shuffleboard.getTab("SmartDashboard")
   .add("kd", 1)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
   .getEntry();
    //Shuffleboard.getTab("Gryo tab").add(m_robotDrive.m_gyro);
    configureButtonBindings();
    JoystickButton armIntake = new JoystickButton(mechanismJoystick, 2);
    JoystickButton armInside = new JoystickButton(mechanismJoystick, 1);
    JoystickButton armAmp = new JoystickButton(mechanismJoystick, 3);
    JoystickButton intakeUntilNote = new JoystickButton(mechanismJoystick, 4);
    JoystickButton intakeIn = new JoystickButton(mechanismJoystick, 8);
    JoystickButton intakeOut = new JoystickButton(mechanismJoystick, 5);
    JoystickButton fireLauncher = new JoystickButton(mechanismJoystick, 6);
    JoystickButton turnToLimelight = new JoystickButton(m_driverController, 1);
    JoystickButton driveToLimelight = new JoystickButton(m_driverController, 2);
    //JoystickButton fireLauncher = new JoystickButton(m_driverController, 3);
    //JoystickButton intake = new JoystickButton(m_driverController, Constants.IntakeConstants.intakeButton);
    //JoystickButton outtake = new JoystickButton(m_driverController, Constants.IntakeConstants.outtakeButton);
    //fireLauncher.toggleOnTrue(new FireSingleMotorLauncherCommand(m_singleMotorLauncher));
    //turnToLimelight.onTrue(new LimelightCommands.TurnTo(m_limeLightTurret, m_robotDrive, ()->0).withTimeout(3));
    //driveToLimelight.onTrue(new LimelightCommands.DriveTo(m_robotDrive, m_limeLightTurret, 1));
    armInside.onTrue(stowArm(m_wrist, m_arm, m_intake));
    armIntake.onTrue(toIntake(m_wrist, m_arm, m_intake));
    armAmp.onTrue(toAmp(m_wrist, m_arm, m_intake,m_aim));
    intakeUntilNote.onTrue(intakeUntilNote(m_wrist, m_arm, m_intake));
    fireLauncher.onTrue(fireLauncher(m_wrist, m_arm, m_intake, m_aim, m_launcher));
    //Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.06),
                true),
            m_robotDrive));
    m_limeLightChassis.setDefaultCommand(new LimelightTestCommand(m_limeLightChassis));
    m_limeLightTurret.setDefaultCommand(new LimelightTestCommand(m_limeLightTurret));
    IntakeStateCommand intakeStateCmd = new IntakeStateCommand(m_intake,false);
    intakeStateCmd.setButtons(intakeIn,intakeOut); //configure keybinds for intake
    m_intake.setDefaultCommand(intakeStateCmd);
    //armIntake.onTrue(new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.intake));
   // armInside.onTrue(new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.inside));
    //armAmp.onTrue(new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.amp));
  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive
            ));
            
            coneFlipperUpButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperUp);
            coneFlipperDownButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperDown);
  }

  public static SequentialCommandGroup stowArm(WristProfiledPIDSubsystem m_wrist,ArmProfiledPIDSubsystem m_arm, IntakeSubsystem m_intake) {
    return new SequentialCommandGroup(
      new ConditionalCommand(
        new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.load),
        Commands.sequence(
          new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.inside),
          new IntakeStateCommand(m_intake, false,State.intake).withTimeout(2),
          new MoveArmAndWristCommand(m_arm,m_wrist,MoveArmAndWristCommand.Position.load)
        ),
        ()-> m_wrist.getGoal()==WristStateCommand.Position.intake.getPos()
      )
     
    );
  }
  public static SequentialCommandGroup toIntake(WristProfiledPIDSubsystem m_wrist, ArmProfiledPIDSubsystem m_arm,IntakeSubsystem m_intake) {
    return new SequentialCommandGroup(
      // stowArm(m_wrist, m_arm, m_intake).onlyIf(()->
      //   m_wrist.getGoal() != WristStateCommand.Position.inside.getPos() && //check if its not inside
      //   m_wrist.getGoal() != WristStateCommand.Position.intake.getPos()//check if its not already at intake), //if the arm isnt ready to move, it stows first
      // ),
      new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.intake),
      new InstantCommand(()->m_intake.setHasNote(false))
    );
  }
  public static SequentialCommandGroup toAmp(WristProfiledPIDSubsystem m_wrist, ArmProfiledPIDSubsystem m_arm,IntakeSubsystem m_intake,VerticalAimerProfiledPIDSubsystem m_aim) {
    return new SequentialCommandGroup(
      // stowArm(m_wrist, m_arm, m_intake).onlyIf(()->
      //   m_wrist.getGoal() != WristStateCommand.Position.inside.getPos() && //check if its not inside
      //   m_wrist.getGoal() != WristStateCommand.Position.amp.getPos()//check if its not already at amp), //if the arm isnt ready to move, it stows first
      // ),
      Commands.parallel(
      new VerticalAimerStateCommand(m_aim, VerticalAimerStateCommand.Position.fire),
      new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.amp)
      ),
      new InstantCommand(()->m_intake.setHasNote(false))
    );
  }
  public static SequentialCommandGroup intakeUntilNote(WristProfiledPIDSubsystem m_wrist, ArmProfiledPIDSubsystem m_arm, IntakeSubsystem m_intake) {
    return new SequentialCommandGroup(
      new IntakeStateCommand(m_intake,true,State.intake).until(m_intake::hasNote),
      stowArm(m_wrist, m_arm, m_intake)
    ).onlyIf(()->m_wrist.getGoal() == WristStateCommand.Position.intake.getPos()).andThen(Commands.none());
  }
  // public static SequentialCommandGroup alignForLaunch(DriveSubsystem m_drive, LimelightSubsystem m_limelight) {
  //   return new SequentialCommandGroup(new LimelightCommands.AlignAtDistance(m_limelight, m_drive, 0));
  // }
  public static SequentialCommandGroup fireLauncher(WristProfiledPIDSubsystem m_wrist, ArmProfiledPIDSubsystem m_arm, IntakeSubsystem m_intake, VerticalAimerProfiledPIDSubsystem m_aim,LauncherSubsystem m_launch) {
    return new SequentialCommandGroup(
     // stowArm(m_wrist, m_arm, m_intake),
      new VerticalAimerStateCommand(m_aim,VerticalAimerStateCommand.Position.fire).until(m_aim::atGoal),
      Commands.race(
        new FireLauncherCommand(m_launch),
        Commands.sequence(
          new WaitCommand(2.5),
          new IntakeStateCommand(m_intake,false,IntakeStateCommand.State.outtake).withTimeout(3.5)
        )
      )
    );
  }
  public static boolean getIntakeButton() {
    return m_driverController.getLeftBumper();
  }
  public static boolean getOuttakeButton() {
    return m_driverController.getRightBumper();
  }
  public static boolean GetConeFlipperUpButton() {
    return mechanismJoystick.getRawButton(Constants.GamePadConstants.ConeFlipperUp);
  }
  public static boolean GetConeFlipperDownButton() {
    return mechanismJoystick.getRawButton(Constants.GamePadConstants.ConeFlipperDown);
  }
 public static boolean GetGrabberCloseButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.GrabberClose);
  
 } 
 public static boolean GetGrabberOpenButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.GrabberOpen);
 } 

 public static boolean GetCubeInButton() {
  return mechanismJoystick.getRawButton(1);
 } 
 public static boolean GetCubeOutButton() {
  return mechanismJoystick.getRawButton(2);
 } 

 public static boolean GetArmExtendButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.ArmExtend);
 }
 public static boolean GetArmRetractButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.ArmRetract);
 }
 public static boolean GetArmGroundButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.ArmGround);
 }
 public static DutyCycleEncoder GetFlipperEncoder() {
  return coneFlipperEncoder;
}
public static boolean GetGrabberCloseCubeButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.GrabberCloseCube);
 } 

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return  autoChooser.getSelected();
  }
  /**
   * <p>true -> red alliance</p>
   * <p>false -> blue alliance</p>
   * <p>null -> no input</p>
   */
  public boolean isRedAlliance() {
      return alllianceChooser.getSelected();
  }
  
  public void disablePIDSubsystems() {
    m_arm.disable();
    m_wrist.disable();
    m_aim.disable();
    m_arm.setGoal(0);
    m_wrist.setGoal(0);
    m_aim.setGoal(0);
  }
  //Comment To Test Merges
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /* 
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
  */
}
