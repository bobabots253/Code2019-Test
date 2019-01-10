package frc.robot.Intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeSpin extends Command {

    private double speed;

    //Constructor used for buttons or auto
    public IntakeSpin(double speed){
        this.speed = speed;
        requires(Robot.intake);
    }

    public IntakeSpin(){
        requires(Robot.intake);
    }

    protected void execute(){
        IntakeSubsystem.spinMotors(speed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}