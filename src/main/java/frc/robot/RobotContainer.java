// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commands.*;
import frc.robot.subsystems.motors.*;
// import frc.robot.subsystems.pneumatics.*;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_PORT_1);

    private final GroupOfMotors motors;
    // private final Pneumatics pneumatics;

    private final Trigger nintendo1 = new Trigger(new DigitalInput(Constants.SWITCH_PORT_1)::get);
    private final Trigger nintendo2 = new Trigger(new DigitalInput(Constants.SWITCH_PORT_2)::get);
    private final Trigger irSensor = new Trigger(new DigitalInput(Constants.IR_SENSOR_PORT)::get);

    public RobotContainer() {
        motors = new GroupOfMotors();
        // pneumatics = new Pneumatics(Constants.PISTON_ID_1, Constants.PISTON_ID_2);

        motors.addMotor(new Motor(new MotorIOTalonFX(Constants.TALONFX_ID), 0));
        motors.addMotor(new Motor(new MotorIOTalonSRX(Constants.TALONSRX_ID_1), 1));

        configureBindings();
    }

    private void configureBindings() {
        //-----Motors-----//
        // motors.setDefaultCommand(
        //     new VoltageWithJoysticks(
        //         motors,
        //         () -> controller.getLeftX()
        //     )
        // );

        motors.setDefaultCommand(
            new PositionWithJoysticks(
                motors,
                () -> controller.getLeftX()
            )
        );

        controller.a().onTrue(new InstantCommand(() -> motors.toggleMotors()));

        //-----Pneumatics-----//
        // controller.b().onTrue(new InstantCommand(() -> pneumatics.togglePiston1()));
        // controller.a().onTrue(new InstantCommand(() -> pneumatics.togglePiston2()));


        //-----DIO-----//
        nintendo1.onTrue(new InstantCommand(() -> System.out.println("switch 1")));
        nintendo2.onTrue(new InstantCommand(() -> System.out.println("switch 2")));
        irSensor.onTrue(new InstantCommand(() -> System.out.println("beam broken")));
    }
}