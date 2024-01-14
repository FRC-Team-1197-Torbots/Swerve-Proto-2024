package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeProto extends SubsystemBase{
    private CANSparkMax MotorA;
    private CANSparkMax MotorB;

    public ToggleState m_ToggleState = ToggleState.OFF; //might delete later

    private enum ToggleState {//might delete later
        ON, OFF
    }

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

    public void runToggleState(){ //might delete later
        switch(m_ToggleState){
            case ON:
                System.out.println("running");
            break;
            case OFF:
                System.out.println("Stopped");
            break;
        }
    }

    public void switchToggleState(){ //might delete later
        if(m_ToggleState == ToggleState.OFF){
            m_ToggleState = ToggleState.ON;
        }
        else {
            m_ToggleState = ToggleState.OFF;
        }
    }

    @Override
    public void periodic() {
    //System.out.println(LimitSwitch.get());
    // This method will be called once per scheduler run
    }
}
