package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

import com.kauailabs.navx.frc.AHRS;
import static edu.wpi.first.units.Units.*;

public class NavxIO implements GyroIO {
    private final AHRS NavX;
    
    public NavxIO(){
        NavX = new AHRS();
        NavX.reset();

    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.imu_Connected = NavX.isConnected();
        inputs.gyroYawDeg = Degrees.of(NavX.getYaw());
        inputs.gyroPitchDeg = Degrees.of(NavX.getPitch());
        inputs.gyroRollDeg = Degrees.of(NavX.getRoll());
        inputs.gyroYawRad = Radians.of(NavX.getYaw());
        inputs.gyroPitchRad = Radians.of(NavX.getPitch());
        inputs.gyroRollRad = Radians.of(NavX.getRoll());
        inputs.turn_times = Rotations.of(NavX.getAngle() / 360.0); // i know there is no reason for this but why not
        inputs.xAccelMps = MetersPerSecond.of(NavX.getWorldLinearAccelX() * 9.81);
        inputs.yAccelMps = MetersPerSecond.of(NavX.getWorldLinearAccelY() * 9.81);
        //inputs.xVelocityMps += MetersPerSecond.of(inputs.xAccelMps.in(MetersPerSecond) * 0.02);
        //inputs.yVelocityMps += MetersPerSecond.of(inputs.yAccelMps.in(MetersPerSecond) * 0.02);
    }

   
    /**
     * Returns the current gyro yaw as a Rotation2d on the Z rotation.
     * @return The current yaw of the gyro.
     */
    @Override
    public Rotation2d getYaw() {
        return new Rotation2d(NavX.getYaw());
    }

    /**
     * Returns the current gyro pitch as a Rotation2d on a Y rotation.
     * @return The current pitch of the gyro.
     */
    @Override
    public Rotation2d getPitch() {
        return new Rotation2d(NavX.getPitch());
    }

    /**
     * Returns the current gyro roll as a Rotation2d on the X rotation.
     * @return The current roll of the gyro.
     */
    @Override
    public Rotation2d getRoll() {
        return new Rotation2d(NavX.getRoll());
    }
    /**
     * Resets the Yaw to 0.
     */
    @Override
    public void resetImu() {
        NavX.reset();
    }

   
}
