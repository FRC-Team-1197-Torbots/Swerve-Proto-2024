package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.IntakeProto;

public class TurnRed extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Blinkin m_blinkin;

    public TurnRed(Blinkin subsystem) {
        m_blinkin = subsystem;
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
        m_blinkin.turnRed();
      }

      @Override
        public void end(boolean interrupted) {
          m_blinkin.turnWhite();
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
