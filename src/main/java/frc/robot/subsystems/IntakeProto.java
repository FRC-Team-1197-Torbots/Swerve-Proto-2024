package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeProto extends SubsystemBase{
    private CANSparkMax MotorA;
    private CANSparkMax MotorB;

    public IntakeProto(){
        MotorA = new CANSparkMax(IntakeConstants.MotorA, MotorType.kBrushless);
        MotorB = new CANSparkMax(IntakeConstants.MotorB, MotorType.kBrushless);
    
    }
    public void runMotor(){
        MotorA.set(0.50);
        MotorB.set(0.75);
    }

    public void stopMotors(){
        MotorA.set(0);
        MotorB.set(0);
      }

    @Override
    public void periodic() {
    //System.out.println(LimitSwitch.get());
    // This method will be called once per scheduler run
    }
}
