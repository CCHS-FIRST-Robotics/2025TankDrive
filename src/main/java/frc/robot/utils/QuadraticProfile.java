package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class QuadraticProfile {
    double period;

    public QuadraticProfile() {
        this(.02);
    }

    public QuadraticProfile(double period) {
        this.period = period;
    }

    public Translation2d[] getCombinedSetPoints(Translation2d initialPosition, Translation2d goal, double speed,
            double acceleration) {
        Translation2d[] accelSetpoints, stoppingSetpoints, constantSpeedSetpoints;

        // displacement from the initial (x, y) to goal
        Translation2d displacement = goal.minus(initialPosition);
        double angleDisplacement = displacement.getAngle().getRadians(); // radians

        double timeToAccelerate = speed / acceleration;

        // check if we'll go past the setpoint/can't achieve max velocity and slow down
        // in time
        if (0.5 * acceleration * Math.pow(timeToAccelerate, 2) > 0.5 * displacement.getNorm()) {
            // calculate time to go halfway
            timeToAccelerate = Math.sqrt(displacement.getNorm() / acceleration);

            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement, acceleration);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement,
                    acceleration * timeToAccelerate, acceleration);

            // never reach max velocity so there should be no setpoints at a constant
            // velocity
            Translation2d[] temp = { new Translation2d(0, 0) };
            constantSpeedSetpoints = temp;
        } else {
            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement, acceleration);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement,
                    acceleration * timeToAccelerate, acceleration);

            Translation2d lastAccelPoint = accelSetpoints[accelSetpoints.length - 1];
            Translation2d constantSpeedDisplacement = displacement.minus(lastAccelPoint.times(2));
            double timeConstantSpeed = constantSpeedDisplacement.getNorm() / speed;
            constantSpeedSetpoints = getConstantSpeedSetpoints(timeConstantSpeed, angleDisplacement, speed);
        }

        System.out.println("#Accel: " + accelSetpoints.length);
        System.out.println("#Speed: " + constantSpeedSetpoints.length);
        System.out.println("#Stop: " + stoppingSetpoints.length);

        return combineSetPoints(accelSetpoints, constantSpeedSetpoints, stoppingSetpoints, initialPosition);
    }

    public Translation2d[] getAcceleratingSetPoints(double timeToEnd, double angle, double acceleration) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Translation2d[] angles = new Translation2d[numSteps];

        for (int i = 0; i < numSteps; i++) {
            angles[i] = new Translation2d(
                    0.5 * Math.cos(angle) * acceleration * Math.pow(i * period, 2),
                    0.5 * Math.sin(angle) * acceleration * Math.pow(i * period, 2));
        }
        return angles;
    }

    public Translation2d[] getStoppingSetpoints(double timeToEnd, double angle, double velocity, double acceleration) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Translation2d[] angles = new Translation2d[numSteps];

        for (int i = 0; i < numSteps; i++) {
            double dt = i * period;
            angles[i] = new Translation2d(
                    (velocity - 0.5 * acceleration * dt) * Math.cos(angle) * dt,
                    (velocity - 0.5 * acceleration * dt) * Math.sin(angle) * dt);
        }
        return angles;
    }

    public Translation2d[] getConstantSpeedSetpoints(double timeToEnd, double angle, double velocity) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Translation2d[] angles = new Translation2d[numSteps];

        for (int i = 0; i < numSteps; i++) {
            angles[i] = new Translation2d(
                    (velocity * i * period) * Math.cos(angle),
                    (velocity * i * period) * Math.sin(angle));
        }
        return angles;
    }

    public Translation2d[] combineSetPoints(Translation2d[] accel, Translation2d[] constant, Translation2d[] stopping,
            Translation2d initialPosition) {
        Translation2d[] combined = new Translation2d[accel.length + constant.length + stopping.length];

        for (int i = 0; i < accel.length; i++) {
            combined[i] = accel[i].plus(initialPosition);
        }
        for (int i = 0; i < constant.length; i++) {
            combined[i + accel.length] = constant[i].plus(combined[accel.length - 1]);
        }
        for (int i = 0; i < stopping.length; i++) {
            combined[i + accel.length + constant.length] = stopping[i]
                    .plus(combined[accel.length + constant.length - 1]);
        }

        return combined;
    }
}