package frc.robot.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class IntakeSubsystem extends Subsystem {

    private static IntakeSubsystem instance = null;

    public static IntakeSubsystem getInstance(){
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    private static VictorSP intakeMotorA = new VictorSP(1);
    //private static final int kTimeout = 10;

    //private static DoubleSolenoid liftSolenoid = new DoubleSolenoid(0, 0, 0);
    private static DoubleSolenoid clampSolenoid = new DoubleSolenoid(1, 4, 5);

    private IntakeSubsystem(){
        /*intakeMotorA.configPeakCurrentLimit(40, kTimeout);
        intakeMotorA.configPeakCurrentDuration(500, kTimeout);
        intakeMotorA.configContinuousCurrentLimit(35, kTimeout);*/
    }

    /* Manipulator intaking controls */
    public static void spinMotors(double speed){
        intakeMotorA.setSpeed(speed);
    }

    public static void switchIntakeClamp(){
        if(clampSolenoid.get() == Value.kForward){
            releaseIntake();
        } else {
            clampIntake();
        }
    }

    public static void clampIntake(){
        clampSolenoid.set(Value.kForward);
    }

    public static void releaseIntake(){
        clampSolenoid.set(Value.kForward);
    }

    /* Manipulator height controls */
    /*public static void setIntakeHeight(DoubleSolenoid.Value position){
        liftSolenoid.set(position);
    }
    
    public static void switchIntakeHeight(){
        if(liftSolenoid.get() == Value.kForward){
            lowerIntake();
        } else {
            raiseIntake();
        }
    }

    public static void raiseIntake(){
        liftSolenoid.set(Value.kForward);
    }

    public static void lowerIntake(){
        liftSolenoid.set(Value.kReverse);
    }*/

    @Override
    protected void initDefaultCommand() {
        new IntakeSpin();
    }
}