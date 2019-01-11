package frc.robot.Intake;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeClamp extends Command {
    private boolean clampStatus;

    public IntakeClamp(boolean clampStatus){
        this.clampStatus = clampStatus;
    }

    protected void execute(){
        if(clampStatus){
            IntakeSubsystem.clampIntake();
            //DONE
        } else{
            IntakeSubsystem.releaseIntake();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}