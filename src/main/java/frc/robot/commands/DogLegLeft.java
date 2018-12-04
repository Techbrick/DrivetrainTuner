package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class DogLegLeft extends CommandGroup{
    public DogLegLeft(Robot robot){
        addSequential(new DriveDistanceAndDirection(robot, 12, 0));
        addSequential(new TurnInPlace(robot, 315));
        addSequential(new DriveDistanceAndDirection(robot, 12,315));
        addSequential(new TurnInPlace(robot, 0));
        addSequential(new DriveDistanceAndDirection(robot, 12,0));
        
    }
}