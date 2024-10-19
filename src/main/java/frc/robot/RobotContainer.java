// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;




import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOTalonSRX;


public class RobotContainer {
    CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_PORT);
    ArmIOTalonSRX io = new ArmIOTalonSRX(Constants.ARM_ID);
    Arm Arm = new Arm(io);
    Drive drive = new Drive();
    private int[] armPositions;


    public RobotContainer() {
      configureBindings();
      armPositions = new int[] {90, 180, 50, 0, 45};
      

    }

    private void configureBindings() {
        // joystick controls
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getRightX()
            )
        );

        controller.x().onTrue(new InstantCommand(() -> Arm.setPosition(Degrees.of(armPositions[1]))));
        
    }
}
