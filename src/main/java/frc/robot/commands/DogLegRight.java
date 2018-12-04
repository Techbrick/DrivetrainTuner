package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class DogLegRight extends CommandGroup{
    public DogLegRight(Robot robot){
        addSequential(new DriveDistanceAndDirection(robot, 12, 0));
        addSequential(new TurnInPlace(robot, 45));
        addSequential(new DriveDistanceAndDirection(robot, 12,45));
        addSequential(new TurnInPlace(robot, 0));
        addSequential(new DriveDistanceAndDirection(robot, 12,0));
        
    }
}