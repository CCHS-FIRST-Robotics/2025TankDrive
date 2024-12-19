package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*; 

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_PORT);
    
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

    /*public Command getAutonomousCommand() {
         return new MoveForwardCommand(drive, 2)
         .andThen(new Turn(drive, -90))
         .andThen(new MoveForwardCommand(drive, 1.5))
         .andThen(new Turn(drive, -180))
         .andThen(new MoveForwardCommand(drive, 2));
    }
    */


    public Command getAutonomousCommand() {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile("Forward1m");

        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Nope: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
}