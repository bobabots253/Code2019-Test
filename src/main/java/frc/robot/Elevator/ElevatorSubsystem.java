package frc.robot.Elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class ElevatorSubsystem extends Subsystem {
    
    private static ElevatorSubsystem instance = null;
    public static ElevatorSubsystem getInstance(){
        if (instance == null) instance = new ElevatorSubsystem();
        return instance;
    }

    private static TalonSRX elevatorMotorA = new TalonSRX(5);
    private static VictorSPX elevatorMotorB = new VictorSPX(2);

    private static final int kTimeout = 10;
    private static final int kPIDIndex = 0;

    private static final int kCruiseVelo = 500;
    private static final int kAccel = 1000;

    private static final int kP = 0, kI = 0, kD = 0;
    private static final double kF = 0.3808637379;

    private ElevatorSubsystem() {
        elevatorMotorB.follow(elevatorMotorA);

        elevatorMotorA.setInverted(true);
        elevatorMotorB.setInverted(true);

        //Current and voltage settings
        elevatorMotorA.configPeakCurrentLimit(40, kTimeout);
        elevatorMotorA.configPeakCurrentDuration(500, kTimeout);
        elevatorMotorA.configContinuousCurrentLimit(20, kTimeout);
        elevatorMotorA.configVoltageCompSaturation(12, kTimeout);
        elevatorMotorA.enableVoltageCompensation(true);
        elevatorMotorA.enableCurrentLimit(true);

        //PID Gains and settings
        elevatorMotorA.selectProfileSlot(0, kPIDIndex);
        elevatorMotorA.config_kF(kPIDIndex, kF, kTimeout);
        elevatorMotorA.config_kP(kPIDIndex, kP, kTimeout);
        elevatorMotorA.config_kI(kPIDIndex, kI, kTimeout);
        elevatorMotorA.config_kD(kPIDIndex, kD, kTimeout);

       /* elevatorMotorB.selectProfileSlot(0, kPIDIndex);
        elevatorMotorB.config_kF(kPIDIndex, kF, kTimeout);
        elevatorMotorB.config_kP(kPIDIndex, kP, kTimeout);
        elevatorMotorB.config_kI(kPIDIndex, kI, kTimeout);
        elevatorMotorB.config_kD(kPIDIndex, kD, kTimeout);
        */
        elevatorMotorA.configMotionCruiseVelocity(kCruiseVelo, kTimeout);
        elevatorMotorA.configMotionAcceleration(kAccel, kTimeout);

       //elevatorMotorB.configMotionCruiseVelocity(kCruiseVelo, kTimeout);
       // elevatorMotorB.configMotionAcceleration(kAccel, kTimeout);

        elevatorMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1, 10);
        elevatorMotorA.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        elevatorMotorA.setSensorPhase(false);
        
    }

    public static double elevAVoltage(){
        return elevatorMotorA.getMotorOutputVoltage();
    }
    public static double elevBVoltage(){
        return elevatorMotorB.getMotorOutputVoltage();
    }

    public static void elevate(double speed){
        elevatorMotorA.set(ControlMode.PercentOutput, speed);
    }

    public static void elevate(ElevatorHeight height){
        elevatorMotorA.set(ControlMode.MotionMagic, height.encoderTarget);
    }

    public static void elevateToTick(int ticks){
        elevatorMotorA.set(ControlMode.MotionMagic, ticks);
    }

    public static void resetEncoders(){
        elevatorMotorA.setSelectedSensorPosition(0, kPIDIndex, kTimeout);
    }

    public static int getEncoder(){
        return elevatorMotorA.getSelectedSensorPosition(kPIDIndex);
    }

    public enum ElevatorHeight {
        SCALE (0),
        SWITCH (0),
        GROUND (0),
        INTAKE_HEIGHT(0);

        private int encoderTarget;
        ElevatorHeight(int encoderTarget){
            this.encoderTarget =  encoderTarget;
        }

        private int getValue() {
            return encoderTarget;
        }

    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Elevate());
    }

}
