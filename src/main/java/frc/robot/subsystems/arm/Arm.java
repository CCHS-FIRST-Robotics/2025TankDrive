package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

public class Arm {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();


public Arm(ArmIO io){
    this.io = io;

}
public void setPosition(Measure<Angle> degress){
    io.setPosition(degress);
}


 public void updateInputs(){
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
}
    











}