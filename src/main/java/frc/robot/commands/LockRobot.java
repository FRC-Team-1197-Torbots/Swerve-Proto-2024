package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class LockRobot extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final DriveSubsystem m_DriveSubsystem;

    public LockRobot(DriveSubsystem subsystem){
        m_DriveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      m_DriveSubsystem.changeState();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
    }

    @Override
    public void end(boolean interrupted) {
    
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
