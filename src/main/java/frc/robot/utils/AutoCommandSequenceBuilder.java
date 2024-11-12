/*package frc.robot.utils;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.Pair;
import com.choreo.lib.*;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants.*;

public final class AutoCommandSequenceBuilder {
    private Drive drive;

    private Command autoCommandSequence;

    public AutoCommandSequenceBuilder(ArrayList<String> pathList, Drive drive) {
        this.drive = drive;

        //add a AutoCommand per path section to the sequence
        for (String path : pathList) {
            addCommand(path);
        }

    }

    public void addCommand(String path) {
        List<Pair<Double, Command>> events = new ArrayList<Pair<Double, Command>>();

        //time for moving forward
        double driveTime = Math.max(AutoPathConstants.MAX_MOVE_TIME, Choreo.getTrajectory(path).getTotalTime());
        double totalTime = driveTime;

        // drive to the next waypoint
        events.add(Pair.of(0.0, drive.followTrajectory(DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path))));
        

        // append to the command sequence
        autoCommandSequence = autoCommandSequence == null ? (new AutoCommand(events, totalTime)) : autoCommandSequence.andThen(new AutoCommand(events, totalTime));
    }

    public Command getAutoCommandSequence() {
        return autoCommandSequence;
    }
}
*/