package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

public class MoveToStart extends Command {
    
    public MoveToStart() {
        //    Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        double[] botpose = LimelightHelpers.getBotPose("limelight");
        // System.out.println("TX: " + botpose[0] + " TY: " + botpose[1]);
        double goalTX = 6.31;
        double goalTY = 1.71;
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

        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //     exampleTrajectory,
        //     m_robotDrive::getPose, // Functional interface to feed supplier
        //     DriveConstants.kDriveKinematics,

        //     // Position controllers
        //     new PIDController(AutoConstants.kPXController, 0, 0),
        //     new PIDController(AutoConstants.kPYController, 0, 0),
        //     thetaController,
        //     m_robotDrive::setModuleStates,
        //     m_robotDrive);

        // // Reset odometry to the starting pose of the trajectory.
        // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // //Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

}
