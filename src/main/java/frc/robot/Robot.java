package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autonomous.PathFollower;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.OI.OI;

public class Robot extends TimedRobot {

  public static OI oi;
  public static DrivetrainSubsystem drivetrain;
  public static IntakeSubsystem intake;
  public static ElevatorSubsystem elevator;

  UsbCamera camera;
  public String gameData;

  SendableChooser<Position> positionChooser;
  SendableChooser<Priority> priorityChooser;

  Command autonomousCommand;

  @Override
  public void robotInit() {
    intake = IntakeSubsystem.getInstance();
    elevator = ElevatorSubsystem.getInstance();
    drivetrain = DrivetrainSubsystem.getInstance();
    oi = new OI();
    
    /*camera = CameraServer.getInstance().startAutomaticCapture(0);

    camera.setResolution(320, 240);
    camera.setFPS(15);*/

    positionChooser = new SendableChooser<Position>();
    positionChooser.addDefault("Center", Position.CENTER);
    positionChooser.addObject("Left", Position.LEFT);
    positionChooser.addObject("Right", Position.RIGHT);
    SmartDashboard.putData("Position", positionChooser);

    priorityChooser = new SendableChooser<Priority>();
    priorityChooser.addDefault("Default priority", Priority.DEFAULT);
    priorityChooser.addObject("Prioritize Baseline", Priority.BASELINE);
    SmartDashboard.putData("Priority Chooser", priorityChooser);

  }

  @Override
  public void disabledInit() {
    DrivetrainSubsystem.resetEncoders();
    DrivetrainSubsystem.resetGyro();
    Scheduler.getInstance().removeAll();

    DrivetrainSubsystem.leftMotorB.setSelectedSensorPosition(0, 0, 10);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Motor Output", DrivetrainSubsystem.leftMotorA.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Motor Output", DrivetrainSubsystem.rightMotorA.getMotorOutputPercent());

    SmartDashboard.putNumber("Raw Gyro Heading", DrivetrainSubsystem.gyro.getAngle());
    SmartDashboard.putNumber("Right Encoder", DrivetrainSubsystem.rightMotorA.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Left Encoder", DrivetrainSubsystem.leftMotorA.getSelectedSensorPosition(0));

  }

  @Override
  public void autonomousInit() {

    gameData = DriverStation.getInstance().getGameSpecificMessage();

    Position position = positionChooser.getSelected();
    Priority priority = priorityChooser.getSelected();

    //DrivetrainSubsystem.shiftGear(Value.kReverse);

    //autonomousCommand = new AutonomousCommand(gameData, position, priority);

    new PathFollower("Straight15ft").start();
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  
  }
 
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();


  }

  public void teleopInit(){
    DrivetrainSubsystem.setBrakeMode();
    DrivetrainSubsystem.leftMotorB.setSelectedSensorPosition(0, 0, 10);

    //DrivetrainSubsystem.shiftGear(Value.kReverse);

  }

  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
  }

  public void disabledPeriodic(){
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    SmartDashboard.putString("gameData", gameData);
  }

  public enum Position{
    LEFT('L'), CENTER('C'), RIGHT('R');
    
    private final char pos;
    
    private Position(char pos){
      this.pos = pos;
    }
    
    public char getPos(){
      return pos;
    }
  }
  
  public enum Goal{
    SCALE, SWITCH, EXCHANGE, BASELINE;
  }
  
  public enum AutoMode{
    SAFE, TEST, EXP;
  }
  
  public enum Priority{
    DEFAULT, BASELINE;
  }

}
