// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  WPI_TalonSRX m_motor = new WPI_TalonSRX(0);
  Joystick m_joystick = new Joystick(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_motor.config_kP(0, 1);
    m_motor.config_kD(0, 10);
    m_motor.config_kI(0, 0);

    m_motor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Position", m_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Setpoint", m_motor.getClosedLoopTarget());
    SmartDashboard.putNumber("Position Error", m_motor.getClosedLoopError());
    SmartDashboard.putNumber("Velocity", m_motor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Position Error Change", m_motor.getErrorDerivative());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_motor.set(ControlMode.Position, 1024);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  // Only necessary for sim
  TalonSRXSimCollection m_simCollection = new TalonSRXSimCollection(m_motor);
  double m_simPosition = 0;
  double m_simVelocity = 0;

  public double getAccel(double voltage) {
    // dw/dt = Kt/JR(V - w/Kv).
    // Let us make max velocity be half rot / second
    // 12 - 524/Kv = 0; Kv ~= 44
    // Let J = R = 1, Kt = 25
    // dw/dt = 25(V - w/44)
    return 25 * (voltage - m_simVelocity / 44);
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    // This is really bad code
    m_simVelocity += getAccel(m_motor.getMotorOutputVoltage()) * 0.020;
    m_simPosition += m_simVelocity * 0.020;

    m_simCollection.setAnalogPosition((int)m_simPosition);
    m_simCollection.setAnalogVelocity((int)m_simVelocity);
  }
}
