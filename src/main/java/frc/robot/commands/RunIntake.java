package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeProto;

public class RunIntake extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeProto m_intake;

    public RunIntake(IntakeProto subsystem) {
        m_intake = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

      @Override
      public void initialize() {
        //System.out.println("running");
        
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        System.out.println("Going up");
        m_intake.runMotor();
      }

      @Override
        public void end(boolean interrupted) {
            m_intake.stopMotors();
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
