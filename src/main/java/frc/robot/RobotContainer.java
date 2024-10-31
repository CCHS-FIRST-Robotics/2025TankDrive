
package frc.robot;

import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.DriveWithJoysticks;
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
}