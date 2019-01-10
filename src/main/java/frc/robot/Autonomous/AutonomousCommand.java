package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Intake.IntakeSpin;
import frc.robot.Robot.Position;
import frc.robot.Robot.Priority;

public class AutonomousCommand extends CommandGroup{
    private String gameData;
    private Position startingPosition;
    private Priority priority;

    private char switchSide;
    private char baselineSide;

    public AutonomousCommand(String gameData, Position startingPosition, Priority priority){
        this.gameData = gameData;
        this.startingPosition = startingPosition;
        this.priority = priority;

        this.switchSide = gameData.charAt(0);

        if(switchSide == 'L'){
            baselineSide = 'R';
        } else if (switchSide == 'R'){
            baselineSide = 'L';
        }

        if(startingPosition == Position.CENTER){

            if(priority == Priority.BASELINE){
                System.out.println("Running center to baseline!");
                new PathFollower("Cto"+baselineSide+"Baseline").start();

            } else if (priority == Priority.DEFAULT){
                System.out.println("Running center to " + switchSide + " switch!");
                addSequential(new RaiseElevatorTimed(0.4, 2));
                addSequential(new PathFollower("Cto"+switchSide+"Switch"));
                addSequential(new IntakeSpin(0.7));

            } else {
                System.out.println("Auto failed :(");
            }  

        } else if (startingPosition == Position.LEFT){

            if(priority == Priority.BASELINE){
                System.out.println("Running left to baseline!");
                new PathFollower("Straight15ft").start();

            } else if (priority == Priority.DEFAULT){
                System.out.println("Running left to left switch!");
                new PathFollower("LtoLSwitch").start();

            } else {
                System.out.println("Auto failed :("); 
            }

        } else if (startingPosition == Position.RIGHT){

            if(priority == Priority.BASELINE){
                System.out.println("Running right to baseline!");
                new PathFollower("Straight15ft").start();

            } else if (priority == Priority.DEFAULT){
                System.out.println("Running right to right switch!");
                new PathFollower("RtoRSwitch").start();
                
            } else {
                System.out.println("Auto failed :(");
                
            }
        }

    }


}