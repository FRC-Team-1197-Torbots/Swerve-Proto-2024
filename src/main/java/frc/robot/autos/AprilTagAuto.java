package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTagAuto extends SequentialCommandGroup{
    public AprilTagAuto(DriveSubsystem drive){
        addCommands(
            new SequentialCommandGroup(
                //move to starting pose by comparing the limelight position and starting position
                //follow path plan    
                new MoveToStart().andThen(new PathPlannerAuto("InsertAuto"))            
            )
        );
    }
}
