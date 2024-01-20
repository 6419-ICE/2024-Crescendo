// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoTestForPaths;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final Arm m_Arm = new Arm();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  static Joystick mechanismJoystick = new Joystick(Constants.ButtonBoxID);
  static JoystickButton coneFlipperUpButton = new JoystickButton(mechanismJoystick,
      Constants.GamePadConstants.ConeFlipperUp);
  static JoystickButton coneFlipperDownButton = new JoystickButton(mechanismJoystick,
      Constants.GamePadConstants.ConeFlipperDown);
  static JoystickButton GrabberOpenButton = new JoystickButton(mechanismJoystick,
      Constants.GamePadConstants.GrabberOpen);
  static JoystickButton GrabberCloseButton = new JoystickButton(mechanismJoystick,
      Constants.GamePadConstants.GrabberClose);
  private static DutyCycleEncoder coneFlipperEncoder = new DutyCycleEncoder(Constants.FlipperEncoderID);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private static SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Configure the button bindings
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Auto Test For Paths", new AutoTestForPaths(m_robotDrive));
    SmartDashboard.putData("Autonomous", autoChooser);

    configureButtonBindings();
    
    // Configure default commands
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
            m_robotDrive));
    coneFlipperUpButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperUp);
    coneFlipperDownButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperDown);
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
    return autoChooser.getSelected();
  }

  public void disablePIDSubsystems() {
  }
}
