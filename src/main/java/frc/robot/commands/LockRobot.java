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
      //System.out.println("running");
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_DriveSubsystem.setX();
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.setZero();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
