// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.MoveForwardCommand;
import frc.robot.commands.MoveTo;
import frc.robot.subsystems.drive.*; 

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_PORT_1);
    
    private final Drive drive;

    public RobotContainer() {
        switch(Constants.MODE){
            case REAL: // real
                drive = new Drive(
                    new GyroIONavX(),
                    new DriveSideIOTalonSRX(Constants.TALONSRX_ID_1, Constants.TALONSRX_ID_2, false), 
                    new DriveSideIOTalonSRX(Constants.TALONSRX_ID_3, Constants.TALONSRX_ID_4, true)
                );
                break;
            case SIM: // simulated
                drive = new Drive(
                    new GyroIO(){},
                    new DriveSideIOSim(), 
                    new DriveSideIOSim()
                );
                break;
            default: // replayed
                drive = new Drive(
                    new GyroIONavX(),
                    new DriveSideIOTalonSRX(Constants.TALONSRX_ID_1, Constants.TALONSRX_ID_2, false), 
                    new DriveSideIOTalonSRX(Constants.TALONSRX_ID_3, Constants.TALONSRX_ID_4, true)
                );
                break;
        }

        configureBindings();
    }
   

    private void configureBindings() {
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive, 
                () -> -controller.getLeftY(), // xboxcontroller is flipped
                () -> controller.getRightX()
            )
        );
    }

    public Command getAutonomousCommand() {
        //Test these tmr
        return new MoveForwardCommand(drive, Degrees.of(0), MetersPerSecond.of(1), Meters.of(1));
        //return new MoveTo(drive, Meters.of(3), Meters.of(2), MetersPerSecond.of(1));
    }
}