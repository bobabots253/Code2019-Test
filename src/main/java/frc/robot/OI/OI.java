package frc.robot.OI;

import static frc.robot.OI.XBPovButton.DOWN;
import static frc.robot.OI.XBPovButton.DOWN_LEFT;
import static frc.robot.OI.XBPovButton.DOWN_RIGHT;
import static frc.robot.OI.XBPovButton.LEFT;
import static frc.robot.OI.XBPovButton.NONE;
import static frc.robot.OI.XBPovButton.RIGHT;
import static frc.robot.OI.XBPovButton.UP;
import static frc.robot.OI.XBPovButton.UP_LEFT;
import static frc.robot.OI.XBPovButton.UP_RIGHT;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.Drivetrain.Drive;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Intake.IntakeSpin;
import frc.robot.Intake.IntakeSubsystem;

public class OI {
    public XboxController xboxcontroller;
    public JoystickButton ButtonA;
    public JoystickButton ButtonB;
    public JoystickButton ButtonX;
    public JoystickButton ButtonY;
    public JoystickButton ButtonRB;
    public JoystickButton ButtonLB;
    public JoystickButton ButtonRT;
    public JoystickButton ButtonLT;
    public Joystick intakestick;
    public static Joystick elevatorstick;

    public JoystickButton dpadUP;
    public JoystickButton dpadUP_RIGHT;
    public JoystickButton dpadRIGHT;
    public JoystickButton dpadDOWN_RIGHT;
    public JoystickButton dpadDOWN;
    public JoystickButton dpadDOWN_LEFT;
    public JoystickButton dpadLEFT;
    public JoystickButton dpadUP_LEFT;
    public JoystickButton dpadNONE;

    public JoystickButton Button1;
    public JoystickButton Button2;
    public JoystickButton Button3;

    public OI(){
        xboxcontroller = new XboxController(1);
        intakestick = new Joystick(3);
        elevatorstick = new Joystick(2);

        ButtonA = new JoystickButton(xboxcontroller, 1);
        ButtonB = new JoystickButton(xboxcontroller, 2);
        ButtonX = new JoystickButton(xboxcontroller, 3);
        ButtonY = new JoystickButton(xboxcontroller, 4);
        ButtonRB = new JoystickButton(xboxcontroller, 6);
        ButtonLB = new JoystickButton(xboxcontroller, 5);
        ButtonRT = new JoystickButton(xboxcontroller, 7);
        ButtonLT = new JoystickButton(xboxcontroller, 8);

        dpadUP = new XBPovButton(xboxcontroller, UP);
        dpadUP_RIGHT = new XBPovButton(xboxcontroller, UP_RIGHT);
        dpadRIGHT = new XBPovButton(xboxcontroller, RIGHT);
        dpadDOWN_RIGHT = new XBPovButton(xboxcontroller, DOWN_RIGHT);
        dpadDOWN = new XBPovButton(xboxcontroller, DOWN);
        dpadDOWN_LEFT = new XBPovButton(xboxcontroller, DOWN_LEFT);
        dpadLEFT = new XBPovButton(xboxcontroller, LEFT);
        dpadUP_LEFT = new XBPovButton(xboxcontroller, UP_LEFT);
        dpadNONE = new XBPovButton(xboxcontroller, NONE);

        //Driver overrides in case joystick buttons fail
        ButtonRB.whileHeld(new IntakeSpin(-0.7));
        ButtonRB.whenReleased(new IntakeSpin(0));

        ButtonA.whileHeld(new IntakeSpin(0.7));
        ButtonA.whenReleased(new IntakeSpin(0));

        //ButtonLB.whileHeld(new IntakeSpin(0.7));
        //ButtonLB.whenReleased(new IntakeSpin(0));

        Button1 = new JoystickButton(intakestick, 1);
        Button2 = new JoystickButton(intakestick, 2);
        Button3 = new JoystickButton(intakestick, 3);

        //Drivatrain and intake toggles
        dpadLEFT.whenPressed(new RunCommand( () -> DrivetrainSubsystem.shiftGear() ));
        Button3.whenPressed(new RunCommand( () -> IntakeSubsystem.switchIntakeClamp() ));
        dpadRIGHT.whenPressed(new RunCommand( () -> IntakeSubsystem.switchIntakeClamp() ));

        //Spin in
        Button1.whileHeld(new IntakeSpin(-0.7));
        Button1.whenReleased(new IntakeSpin(0));

        //Spin out
        Button2.whileHeld(new IntakeSpin(0.7));
        Button2.whenReleased(new IntakeSpin(0));

        dpadUP.whenPressed(new RunCommand( () -> DrivetrainSubsystem.changeCamVal() ));
        
    }

    public double throttleValue() {
        //Controllers y-axes are natively up-negative, down-positive
        return -xboxcontroller.getY(Hand.kLeft);
    }

    public double turnValue() {
        return xboxcontroller.getX(Hand.kRight);
    }

    public static double elevateValue(){ 
        return elevatorstick.getY();
    }

    public double intakeSpeed(){
        return intakestick.getY();

    }

    /*
     * Methods for controlling quickturn 
     */
    public double getLeftTrigger(){
        double deadband = 0.05;
        return Drive.deadbandX(xboxcontroller.getTriggerAxis(Hand.kLeft), deadband);
    }
    public double getRightTrigger(){
        double deadband = 0.05;
        return Drive.deadbandX(xboxcontroller.getTriggerAxis(Hand.kRight), deadband);
    }
    public boolean isQuickturn(){

        boolean leftActive = getLeftTrigger() != 0; 
        boolean rightActive = getRightTrigger() != 0; ; 

        return leftActive || rightActive;

    }

    /*
     * Methods for controlling quickturn (triggered)
     */
    public double getThrottleX(){
        return xboxcontroller.getX(Hand.kRight);
    }
    public boolean isQuickturnTwo(){
        return xboxcontroller.getBumper(Hand.kLeft);
    }
}