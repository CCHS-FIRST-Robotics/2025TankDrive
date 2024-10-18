package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase{
    private final SolenoidIOPCM piston1, piston2;

    public Pneumatics(int pistonId1, int pistonId2){
        piston1 = new SolenoidIOPCM(pistonId1);
        piston2 = new SolenoidIOPCM(pistonId2);
    }

    public void extendPiston1(){
        piston1.set(true);
    }

    public void compressPiston1(){
        piston1.set(false);
    }

    public void togglePiston1(){
        piston1.toggle();
    }

    public void extendPiston2(){
        piston2.set(true);
    }

    public void compressPiston2(){
        piston2.set(false);
    }

    public void togglePiston2(){
        piston2.toggle();
    }
}