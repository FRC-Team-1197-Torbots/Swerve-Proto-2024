/*TEST CODE FOR TOGGLING A COMMAND USING A BUTTON, will probably delete in the future */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeProto;

public class ToggleTest extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeProto m_intake;

    public ToggleTest(IntakeProto subsystem) {
        m_intake = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
      public void initialize() {
        System.out.println(m_intake.m_ToggleState);
        m_intake.switchToggleState();
        m_intake.runToggleState();
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        //System.out.println("Going up");
        //m_intake.runMotor();
      }

      @Override
        public void end(boolean interrupted) {
            //m_intake.stopMotors();
        }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
