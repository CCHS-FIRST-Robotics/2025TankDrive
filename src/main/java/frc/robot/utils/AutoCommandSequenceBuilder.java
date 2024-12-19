package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutoCommandSequenceBuilder {
    private Drive drive;
    private Command autoCommand;

    public AutoCommandSequenceBuilder(ArrayList<String> pathList, Drive drive) {
        this.drive = drive;

        for (String path : pathList) {
            addCommand(path);
        }
    }

    public void addCommand(String path) {
        List<Pair<Double, Command>> events = new ArrayList<Pair<Double, Command>>();

        events.add(Pair.of(0.0, drive.))
    }

}
