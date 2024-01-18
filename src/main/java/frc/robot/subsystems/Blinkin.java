package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase{
    private Spark Blinkin;

    public Blinkin(){
        Blinkin = new Spark(9);    
    }
    public void turnRed(){
        Blinkin.set(0.61);
    }
    public void turnRainbow(){
        Blinkin.set(0.87);
    }
    public void turnGreen(){
        Blinkin.set(0.77);
    }
    public void turnWhite(){
        Blinkin.set(0.93);
    }
    //rainbow in auto, white without note, red with note

    @Override
    public void periodic() {
    //System.out.println(LimitSwitch.get());
    // This method will be called once per scheduler run
    }
}
