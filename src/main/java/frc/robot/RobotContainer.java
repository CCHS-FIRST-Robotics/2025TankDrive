package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drive.*;
import frc.robot.utils.AutoCommandSequenceBuilder; 

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
        drive.setDefaultCommand(new DriveWithJoysticks(drive, () -> -controller.getLeftY(),() -> controller.getRightX())); // controler throttle flipped
    }

     public Command getAutonomousCommand() {
        return new AutoCommandSequenceBuilder(AutoPathConstants.twoStraight, drive).getAutoCommandSequence();
    }
}