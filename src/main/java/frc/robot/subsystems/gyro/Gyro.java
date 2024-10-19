package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;


import org.littletonrobotics.junction.Logger;

public class Gyro {
    NavxIO io;
    private GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro() {
        io = new NavxIO();
    }

     public void updateInputs(){
        io.updateInputs(inputs);
        Logger.processInputs("NavX", inputs);
    }
   
    /**
     * Returns the current gyro yaw as a Rotation2d on the Z rotation.
     * @return The current yaw of the gyro.
     */
    public Rotation2d getYaw() {
        return io.getYaw();
    }

    /**
     * Returns the current gyro pitch as a Rotation2d on a Y rotation.
     * @return The current pitch of the gyro.
     */
    public Rotation2d getPitch() {
        return io.getPitch();
    }

    /**
     * Returns the current gyro roll as a Rotation2d on the X rotation.
     * @return The current roll of the gyro.
     */
    public Rotation2d getRoll() {
        return io.getRoll();
    }
    /**
     * Resets the Yaw to 0.
     */
    public void resetImu() {
        io.resetImu();;
    }

   
}
