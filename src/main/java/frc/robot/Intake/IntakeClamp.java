package frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.Drivetrain.Drive;
import frc.robot.OI.OI;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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