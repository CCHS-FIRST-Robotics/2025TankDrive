// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drive.tankDrive.*;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_PORT_1);
    
    private final Drive drive;

    public RobotContainer() {
        drive = new Drive(
            new DriveSideIOTalonSRX(Constants.TALONSRX_ID_1, Constants.TALONSRX_ID_2, false), 
            new DriveSideIOTalonSRX(Constants.TALONSRX_ID_3, Constants.TALONSRX_ID_4, true)
        );

        configureBindings();
    }

    private void configureBindings() {
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive, 
                () -> controller.getLeftY(), 
                () -> controller.getRightX()
            )
        );

        controller.b().onTrue(new InstantCommand(() -> drive.setVoltage(Volts.of(8), Volts.of(8))));
    }
}