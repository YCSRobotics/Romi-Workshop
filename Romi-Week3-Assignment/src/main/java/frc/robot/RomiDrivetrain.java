// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class RomiDrivetrain {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the gyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Initialize our open loop based objects
  private boolean m_hasInitTimer = false;
  private Timer m_moveTimer = new Timer();

  // Initialize the pid controller for proper closed loop control
  private PIDController m_forwardController = new PIDController(0.2, 0, 0);
  private PIDController m_turnController = new PIDController(0.05, 0, 0);

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void initDrivetrain() {
    m_hasInitTimer = false;
    m_moveTimer.reset();

    // Only return a non-zero value if error is outside of tolerance
    m_forwardController.setTolerance(0.1);
    m_turnController.setTolerance(0.1);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  /**
   * Moves the robot forward for 5 seconds
   */
  public void timedBasedAutonomous() {
    // start the timer
    if (!m_hasInitTimer) {
      m_moveTimer.start();
      m_hasInitTimer = true;
    }

    // if the timer has not passed 5 seconds, move forward at half speed
    if (!m_moveTimer.hasElapsed(5)) {
      arcadeDrive(0.5, 0.0);
    } else {
      arcadeDrive(0.0, 0.0);
    }
  }

  /**
   * A rudementary closed-loop based autonomous that stops the robot after 2 feet has passed
   * On a real robot, it's better to use a PID based controller to ensure you reach the target distance
   * as accurately as possible
   */
  public void bangBangAutonomous() {
    if (getLeftDistanceInch() < 24) {
      m_diffDrive.arcadeDrive(0.5, 0);
    } else {
      m_diffDrive.arcadeDrive(0.0, 0);
    }
  }

  /**
   * A more complex go-straight closed-loop autonomous that uses two pid controllers (forward and turn)
   * Each pid controller is broken out into it's own calculateXOutput() function that returns suitable values
   */
  public void pidBasedGoStraightAutonomous() {
    var forwardPercOut = calculateForwardOutput(24);
    var turnPercOut = calculateTurnOutput(0);

    m_diffDrive.arcadeDrive(forwardPercOut, turnPercOut);
  }

  /**
   * Calulates the forward closed loop output
   */
  public double calculateForwardOutput(double distanceInInches) {
    return MathUtil.clamp(m_forwardController.calculate(getLeftDistanceInch(), distanceInInches), 0, 1.0);
  }

  /**
   * Calculates the turn closed loop output
   */
  public double calculateTurnOutput(double angleInDegrees) {
    return MathUtil.clamp(m_turnController.calculate(-m_gyro.getAngleZ(), angleInDegrees), 0, 1.0);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }
}
