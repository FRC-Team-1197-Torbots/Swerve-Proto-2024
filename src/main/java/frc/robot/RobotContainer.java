// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.AprilTagAuto;
import frc.robot.commands.LockRobot;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TurnRed;
import frc.robot.commands.ScanAprilTag;
import frc.robot.subsystems.Blinkin;
import frc.robot.commands.ToggleTest;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeProto;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeProto m_Intake = new IntakeProto();
  private final Blinkin m_Blinkin = new Blinkin();

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  private Trigger exTrigger = new Trigger(m_robotDrive::checkLocked);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    // NamedCommands.registerCommand("ScanAprilTag", Commands.print("runinng"));//new ScanAprilTag()
    // NamedCommands.registerCommand("ScanAprilTag", new ScanAprilTag());
    
    configureButtonBindings();

    NamedCommands.registerCommand("Justin's Command", new PrintCommand("This should run!!"));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
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
    //new JoystickButton(m_driverController, Button.kR1.value)
        //.whileTrue(new RunCommand(
           // () -> m_robotDrive.setX(),
           // m_robotDrive));


    m_driverController.a().whileTrue(new RunIntake(m_Intake));
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.changeState()));//.andThen(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive).until(m_robotDrive::checkLocked)));//.)until(m_robotDrive::checkLocked));
    exTrigger.whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    // m_driverController.rightBumper().whileTrue(new TurnRed(m_Blinkin));
    m_driverController.x().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    m_driverController.rightTrigger().whileTrue(new ScanAprilTag(m_robotDrive));
    //m_driverController.a().whileTrue(new RunIntake(m_Intake));
    //m_driverController.leftBumper().onTrue(new ToggleTest(m_Intake));
    //m_driverController.leftBumper().onTrue(new ConditionalCommand(new InstantCommand(() -> m_robotDrive.setX(), ), getAutonomousCommand(), null))
  }

  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("AprilTag auto");

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    double[] botpose = LimelightHelpers.getBotPose_wpiBlue("limelight");
    // System.out.println("TX: " + botpose[0] + " TY: " + botpose[1]);
    double goalTX = 14.14; //4.65
    double goalTY = 5.548; //1.78

    double midTX = (goalTX + botpose[0])/2;
    double midTY = (goalTY + botpose[1])/2;


    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction

        new Pose2d(botpose[0], botpose[1], new Rotation2d(0)), //prev 0,0
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(midTX,midTY)),//prev (1,1), (2,-1)
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(goalTX, goalTY, new Rotation2d(0)),//prev 3,0
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
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
