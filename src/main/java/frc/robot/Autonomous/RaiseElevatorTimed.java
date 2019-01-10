package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Elevator.Elevate;
import frc.robot.Elevator.ElevatorSubsystem;

public class RaiseElevatorTimed extends Command{
    private double timeout;
    private double speed;

    public RaiseElevatorTimed(double speed, double timeout){
        requires(Robot.drivetrain);
        this.timeout = timeout;
        this.speed = speed;
    }

    protected void initialize(){
        System.out.println("Raising elevator at " + speed*100 + " percent for " + timeout +" seconds!");
        setTimeout(timeout);
    }

    protected void execute(){
        ElevatorSubsystem.elevate(-0.4);
    }

    protected void end(){
        ElevatorSubsystem.elevate(Elevate.holdVoltage);
    }

    protected boolean isFinished(){
        return isTimedOut();
    }
}