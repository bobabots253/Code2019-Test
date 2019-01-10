package frc.robot.Drivetrain;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.networktables.NetworkTableInstance;

public class DrivetrainSubsystem extends Subsystem {

    private static DrivetrainSubsystem instance = null;

    public static DrivetrainSubsystem getInstance() {
        if (instance == null)
            instance = new DrivetrainSubsystem();
        return instance;
    }

    private static final int kTimeout = 10;
    private static final int kPIDIndex = 0;
    private static final int kCruiseVelo = 500;
    private static final int kAccel = 1000;

    public static Compressor compressor = new Compressor(1);
    public static final DoubleSolenoid shifter = new DoubleSolenoid(1, 6, 7);
    public static final AHRS gyro = new AHRS(SPI.Port.kMXP);
    public static final TalonSRX leftMotorA = new TalonSRX(3), leftMotorB = new TalonSRX(4),
            rightMotorA = new TalonSRX(2), rightMotorB = new TalonSRX(1);

    // Creates arrays for various motors so I can call the same methods for each at
    // the same time
    public static final TalonSRX[] motors = { leftMotorA, leftMotorB, rightMotorB, rightMotorA };
    private static final TalonSRX[] leftMotors = { leftMotorA, leftMotorB };
    private static final TalonSRX[] rightMotors = { rightMotorA, rightMotorB };

    private static int ledval = 0;

    public void initDefaultCommand() {
        setDefaultCommand(new CurvatureDrive());
    }

    private DrivetrainSubsystem() {
        // Setting leader and follower talons
        leftMotorB.follow(leftMotorA);
        rightMotorB.follow(rightMotorA);

        leftMotorB.configOpenloopRamp(0, kTimeout);
        rightMotorB.configOpenloopRamp(0, kTimeout);

        // DrivetrainSubsystem negation settings
        Arrays.stream(leftMotors).forEach(motor -> motor.setInverted(false));
        Arrays.stream(rightMotors).forEach(motor -> motor.setInverted(true));

        // Setting common settings for Talons
        for (TalonSRX motor : motors) {

            // Current and voltage settings
            motor.configPeakCurrentLimit(30, kTimeout);
            motor.configPeakCurrentDuration(500, kTimeout);
            motor.configContinuousCurrentLimit(35, kTimeout);
            motor.configVoltageCompSaturation(12, kTimeout);
            motor.enableVoltageCompensation(true);
            motor.enableCurrentLimit(true);

            // PID Gains and settings
            motor.selectProfileSlot(0, kPIDIndex);
            motor.config_kF(0, 0.3808637379, kTimeout);
            motor.config_kP(0, 0.1, kTimeout);
            motor.config_kI(0, 0, kTimeout);
            motor.config_kD(0, 0, kTimeout);

            motor.configMotionCruiseVelocity(kCruiseVelo, kTimeout);
            motor.configMotionAcceleration(kAccel, kTimeout);

        }

        // Left drivetrain encoder
        leftMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1, 10);
        leftMotorA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        leftMotorA.setSensorPhase(true);

        // Right drivetrain encoder
        rightMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1, 10);
        rightMotorA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        rightMotorA.setSensorPhase(false);

    }

    public static void setOpenLoopRamp(double ramp) {
        leftMotorA.configOpenloopRamp(ramp, 10);
        rightMotorA.configOpenloopRamp(ramp, 10);
    }

    // Sets drivetrain sides to speed parameters
    public static void drive(double leftspeed, double rightspeed) {

        leftMotorA.set(ControlMode.PercentOutput, leftspeed);
        rightMotorA.set(ControlMode.PercentOutput, rightspeed);

    }

    // Sets drivetrain sides to an encoder target
    public static void driveDistance(double targetLeft, double targetRight) {

        leftMotorA.set(ControlMode.MotionMagic, targetLeft);
        rightMotorA.set(ControlMode.MotionMagic, targetRight);

    }

    public static void shiftGear() {
        switch (shifter.get()) {
        case kForward:
            shifter.set(DoubleSolenoid.Value.kReverse);
            System.out.println("Shifting to low gear!");
            break;
        default:
            shifter.set(DoubleSolenoid.Value.kForward);
            System.out.println("Shifting to high gear!");
            break;
        }
    }

    public static void shiftGear(DoubleSolenoid.Value shiftTo) {
        shifter.set(shiftTo);
        System.out.println("Shifting gears!");
    }

    public static void resetEncoders() {
        leftMotorA.setSelectedSensorPosition(0, 0, 10);
        rightMotorA.setSelectedSensorPosition(0, 0, 10);
    }

    public static void resetGyro() {
        gyro.reset();
    }

    public static void setBrakeMode() {
        Arrays.stream(motors).forEach(motor -> motor.setNeutralMode(NeutralMode.Brake));
    }

    public static void setCoastMode() {
        Arrays.stream(motors).forEach(motor -> motor.setNeutralMode(NeutralMode.Coast));
    }

    public static void stopCompressor() {
        compressor.stop();
    }

    public static void changeCamVal(){
        ledval=(ledval+1)%3;
        NetworkTableInstance.getDefault()
                    .getTable("limelight")
                    .getEntry("ledMode")
                    .setNumber(ledval);
    }

    public static void changeCamVal(int val){
        val = val%3;
        NetworkTableInstance.getDefault()
                    .getTable("limelight")
                    .getEntry("ledMode")
                    .setNumber(val);
    }
}
