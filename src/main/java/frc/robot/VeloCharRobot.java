package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

public class VeloCharRobot extends TimedRobot {
    private static final TalonSRX leftMotorA = new TalonSRX(3), leftMotorB = new TalonSRX(4),
            rightMotorA = new TalonSRX(2), rightMotorB = new TalonSRX(1);

    private static final TalonSRX[] motors = { leftMotorA, leftMotorB, rightMotorB, rightMotorA };
    private static final TalonSRX[] leftMotors = { leftMotorA, leftMotorB };
    private static final TalonSRX[] rightMotors = { rightMotorA, rightMotorB };

    private PrintWriter csvWriter = null;
    private Joystick joystick = new Joystick(0);
    public double appliedOutput;
    public double rampRate = 0.020833333;
    public long lastTime;

    @Override
    public void robotInit() {
        // Setting master and follower talons
        leftMotorB.follow(leftMotorA);
        rightMotorB.follow(rightMotorA);

        // Negation settings
        Arrays.stream(leftMotors).forEach(motor -> motor.setInverted(true));
        Arrays.stream(rightMotors).forEach(motor -> motor.setInverted(false));

        for (TalonSRX motor : motors) {
            // Current and voltage settings
            motor.configPeakCurrentLimit(30, 10);
            motor.configPeakCurrentDuration(500, 10);
            motor.configContinuousCurrentLimit(35, 10);
            motor.configVoltageCompSaturation(12, 10);
            motor.enableVoltageCompensation(true);
            motor.enableCurrentLimit(true);
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

    @Override
    public void robotPeriodic() {
        if (!isOperatorControl() && csvWriter != null) {
            csvWriter.close();
            csvWriter = null;
        }
        Scheduler.getInstance().run();
    }

    public void teleopPeriodic() {
        if (joystick.getRawButton(1)) { // if button A is pressed
            if (csvWriter == null) {
                // create a new CSV writer, reset everything
                try {
                    csvWriter = new PrintWriter(
                            new File("/home/lvuser/dtmeasure/measure_velocity-" + System.currentTimeMillis() + ".csv"));
                    csvWriter.println("lvoltage,lvelocity,rvoltage,rvelocity");
                } catch (FileNotFoundException e) {
                    throw new RuntimeException(e);
                }
                appliedOutput = 0;
                leftMotorA.set(ControlMode.PercentOutput, 0);
                rightMotorA.set(ControlMode.PercentOutput, 0);
            } else {

                csvWriter.println(leftMotorA.getMotorOutputVoltage() + "," + leftMotorA.getSelectedSensorVelocity(0)
                        + "," + rightMotorA.getMotorOutputVoltage() + "," + rightMotorA.getSelectedSensorVelocity(0));

                appliedOutput += rampRate * ((System.currentTimeMillis() - lastTime) / 1000.0);
                lastTime = System.currentTimeMillis();
                leftMotorA.set(ControlMode.PercentOutput, appliedOutput);
                rightMotorA.set(ControlMode.PercentOutput, appliedOutput);
            }
        } else {
            appliedOutput = 0;
            if (csvWriter != null) {
                csvWriter.close();
                csvWriter = null;
            }
            leftMotorA.set(ControlMode.PercentOutput, 0);
            rightMotorA.set(ControlMode.PercentOutput, 0);
        }
        lastTime = System.currentTimeMillis();
    }

    private static double toFeet(double ticksPerDecisecond) {
        return (ticksPerDecisecond / 4517.0) * 6 * Math.PI * 10;
    }
}