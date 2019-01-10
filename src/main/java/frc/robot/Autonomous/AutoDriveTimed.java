package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Drivetrain.DrivetrainSubsystem;

public class AutoDriveTimed extends Command{
    private double timeout;
    private double speed = 0.4;

    public AutoDriveTimed(double timeout){
        requires(Robot.drivetrain);
        this.timeout = timeout;
    }

    protected void initialize(){
        System.out.println("Running timed straight driving at " + speed*100 + " percent for " + timeout +" seconds!");
        setTimeout(timeout);
    }

    protected void execute(){
        DrivetrainSubsystem.drive(speed,speed);
    }

    protected void end(){
        DrivetrainSubsystem.drive(0,0);
    }

    protected boolean isFinished(){
        return isTimedOut();
    }
}