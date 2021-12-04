// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  PWMTalonSRX leftRearMotor = new PWMTalonSRX(Constants.DriveConstants.kLeftRearMotor);
  PWMTalonSRX leftFrontMotor = new PWMTalonSRX(Constants.DriveConstants.kLeftFrontMotor);
  SpeedControllerGroup m_left = new SpeedControllerGroup(leftFrontMotor, leftRearMotor);

  // The motors on the right side of the drive.
  PWMTalonSRX rightRearMotor = new PWMTalonSRX(Constants.DriveConstants.kRightRearMotor);
  PWMTalonSRX rightFrontMotor = new PWMTalonSRX(Constants.DriveConstants.kRightFrontMotor);
  SpeedControllerGroup m_right = new SpeedControllerGroup(rightFrontMotor, rightRearMotor);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() { 
    m_right.setInverted(true);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot, true);
  }

  public void TankDrive(double left, double right){
    m_drive.tankDrive(left*1, right*1, true);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }
  
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
