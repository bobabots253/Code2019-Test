package frc.robot.Intake;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeVertical extends Command {
    private boolean clampStatus;

    public IntakeVertical(boolean clampStatus){
        this.clampStatus = clampStatus;
    }

    protected void execute(){
        /*if(clampStatus){
            IntakeSubsystem.raiseIntake();
        } else{
            IntakeSubsystem.lowerIntake();
        }*/
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}