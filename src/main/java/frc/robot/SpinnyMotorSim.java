package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;


/** Represents a simulated spinny motor mechanism. */
public class SpinnyMotorSim extends LinearSystemSim<N2, N1, N2> {
  // Gearbox for the spinny motor.
  private final DCMotor m_gearbox;

  // The gearing from the motors to the output.
  private final double m_gearing;

  /**
   * Creates a simulated spinny motor mechanism.
   *
   * @param plant The linear system that represents the spinny motor.
   * @param gearbox The type of and number of motors in the spinny motor gearbox.
   * @param gearing The gearing of the spinny motor (numbers greater than 1 represent reductions).
   */
  public SpinnyMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing) {
    super(plant);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated spinny motor mechanism.
   *
   * @param plant The linear system that represents the spinny motor.
   * @param gearbox The type of and number of motors in the spinny motor gearbox.
   * @param gearing The gearing of the spinny motor (numbers greater than 1 represent reductions).
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public SpinnyMotorSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      double gearing,
      Matrix<N2, N1> measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated spinny motor mechanism.
   *
   * @param gearbox The type of and number of motors in the spinny motor gearbox.
   * @param gearing The gearing of the spinny motor (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the spinny motor. If this is unknown, use the
   *     {@link #spinny motorSim(LinearSystem, DCMotor, double, Matrix)} constructor.
   */
  @SuppressWarnings("ParameterName")
  public SpinnyMotorSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
    super(LinearSystemIdNew.createSpinnyMotorSystem(gearbox, jKgMetersSquared, gearing));
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated spinny motor mechanism.
   *
   * @param gearbox The type of and number of motors in the spinny motor gearbox.
   * @param gearing The gearing of the spinny motor (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the spinny motor. If this is unknown, use the
   *     {@link #spinny motorSim(LinearSystem, DCMotor, double, Matrix)} constructor.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  @SuppressWarnings("ParameterName")
  public SpinnyMotorSim(
      DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N2, N1> measurementStdDevs) {
    super(
        LinearSystemIdNew.createSpinnyMotorSystem(gearbox, jKgMetersSquared, gearing),
        measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Returns the spinny motor position.
   *
   * @return The spinny motor position.
   */
  public double getAngularPositionRad() {
    return getOutput(0);
  }

  /**
   * Returns the spinny motor position in rotations.
   *
   * @return The spinny motor position in rotations.
   */
  public double getAngularPositionRotations() {
    return Units.radiansToRotations(getAngularPositionRad());
  }

  /**
   * Returns the spinny motor velocity.
   *
   * @return The spinny motor velocity.
   */
  public double getAngularVelocityRadPerSec() {
    return getOutput(1);
  }

  /**
   * Returns the spinny motor velocity in RPM.
   *
   * @return The spinny motor velocity in RPM.
   */
  public double getAngularVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(getAngularVelocityRadPerSec());
  }

  /**
   * Returns the spinny motor current draw.
   *
   * @return The spinny motor current draw.
   */
  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is spinning
    // 2x faster than the output
    return m_gearbox.getCurrent(getAngularVelocityRadPerSec() * m_gearing, m_u.get(0, 0))
        * Math.signum(m_u.get(0, 0));
  }

  /**
   * Sets the input voltage for the spinny motor.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
  }
}
