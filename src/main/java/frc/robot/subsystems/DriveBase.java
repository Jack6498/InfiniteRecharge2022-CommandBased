// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.surpriselib.DriveSignal;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DriveBase extends SubsystemBase implements Loggable {
  // motors
  private WPI_TalonFX leftLeader, rightLeader;
  private WPI_TalonFX leftFollower, rightFollower;
  private MotorControllerGroup leftMotors, rightMotors;
  private DifferentialDriveOdometry odometry;
  private DifferentialDrive diffDrive;
  // 
  @Log
  private DoubleSolenoid  shifter; // gear shifter
  private Compressor compressor;
  // imu
  private AHRS gyro;
  
  private boolean isHighGear = false;
  private DriveControlMode driveControlMode;
  private final SimpleMotorFeedforward driveFeedforward =
    new SimpleMotorFeedforward(
      kS,
      kVLinear,
      kALinear
    );
  /**
   * Driving Mode<p>
   * Available Modes are Open Loop (Manual), Base Locked, and Trajectory Following.<p>
   * WARNING: ONLY OPEN LOOP IS CURRENTLY IMPLEMENTED
   */
  public enum DriveControlMode
  {
    OPEN_LOOP, 
    BASE_LOCKED,
    TRAJECTORY_FOLLOWING
  }

  public DriveBase()
  {
    leftLeader = new WPI_TalonFX(LeftLeaderId);
    leftFollower = new WPI_TalonFX(LeftFollowerId);
    rightLeader = new WPI_TalonFX(RightLeaderId);
    rightFollower = new WPI_TalonFX(RightFollowerId);

    leftLeader.configOpenloopRamp(DriveRampRate);
    leftFollower.configOpenloopRamp(DriveRampRate);
    rightLeader.configOpenloopRamp(DriveRampRate);
    rightFollower.configOpenloopRamp(DriveRampRate);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftMotors = new MotorControllerGroup(leftLeader, leftFollower);
    rightMotors = new MotorControllerGroup(rightLeader, rightFollower);
    rightMotors.setInverted(true);
    diffDrive = new DifferentialDrive(leftMotors, rightMotors);
    diffDrive.setSafetyEnabled(false);

    gyro = new AHRS(Port.kMXP);
    gyro.reset();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ShifterForwardSolenoidId, ShifterReverseSolenoidId);

    // engage brakes when neutral input
    setBrakeMode(NeutralMode.Coast);

    // setup encoders
    leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  // hardware methods

  public void resetSensors()
  {
    gyro.reset();
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
  }

  public Rotation2d getGyroAngle()
  {
    // negative angle because of the direction the gyro is mounted
    return Rotation2d.fromDegrees(gyro.getAngle());
  }


  @Log
  public double getHeading() {
    return getGyroAngle().getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public void setBrakeMode(NeutralMode brakeMode)
  {
    leftLeader.setNeutralMode(brakeMode);
    rightLeader.setNeutralMode(brakeMode);
  }
  
  // driving modes
  public void setTankDrive(final DriveSignal signal)
  {
    if(driveControlMode != DriveControlMode.OPEN_LOOP)
    {
      driveControlMode = DriveControlMode.OPEN_LOOP;
    }
    diffDrive.tankDrive(signal.leftMotor, signal.rightMotor);
    setBrakeMode(signal.brakeMode);
  }

  public void arcadeDriveIK(double throttle, double turn)
  {
    throttle = MathUtil.applyDeadband(throttle, 0.05);
    turn = MathUtil.applyDeadband(turn, 0.05);

    DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(throttle, turn, true);
    leftLeader.set(ControlMode.PercentOutput, speeds.left);
    leftLeader.set(ControlMode.PercentOutput, speeds.right);
  }

  public void arcadeDrive(double throttle, double turn) {
    diffDrive.arcadeDrive(throttle, turn, true);
  }

  public void toggleGear()
  {
    if (isHighGear) {
      shifter.set(Value.kReverse);
      isHighGear = false;
    } else {
      shifter.set(Value.kForward);
      isHighGear = true;
    }
  }

  @Log(name="Gear")
  public boolean getGear()
  {
    return isHighGear;
  }

  public void stop()
  {
    setTankDrive(DriveSignal.NEUTRAL);
  }

  public double getEncoderPosition()
  {
    return ((leftLeader.getSelectedSensorPosition() + leftFollower.getSelectedSensorPosition()) / 2) 
      + ((rightLeader.getSelectedSensorPosition() + rightFollower.getSelectedSensorPosition()) / 2);
  }

  @Log(name = "Compressor Running")
  public boolean getPressure() {
    return compressor.enabled();
  }

  @Log(name = "Yaw (deg.)")
  public double getGyroAngleDegrees() {
    return getGyroAngle().getDegrees();
  }

  @Override
  public void periodic() {
    odometry.update(
      gyro.getRotation2d(), 
      leftLeader.getSelectedSensorPosition(), 
      rightLeader.getSelectedSensorPosition()
    );
  }

  @Log.Graph
  public double getLeftVelocity() {
    return leftLeader.getSelectedSensorVelocity();
  }

  @Log.Graph
  public double getRightVelocity() {
    return rightLeader.getSelectedSensorVelocity();
  }
  
  @Log
  public double getSpeedRatio() {
    // 1.0 => both sides equal, 0.0 => left 0, right 1, 100.0 => right 0, left 1
    return MathUtil.applyDeadband(leftLeader.getSelectedSensorVelocity(), 0.01) / MathUtil.applyDeadband(rightLeader.getSelectedSensorVelocity(), 0.01);
  }

  public void consumeTrapezoidState(TrapezoidProfile.State leftProfileState, TrapezoidProfile.State rightProfileState) {
    leftLeader.set(
      ControlMode.Position,
      leftProfileState.position,
      DemandType.ArbitraryFeedForward,
      driveFeedforward.calculate(leftProfileState.velocity)
    );

    rightLeader.set(
      ControlMode.Position,
      rightProfileState.position,
      DemandType.ArbitraryFeedForward,
      driveFeedforward.calculate(rightProfileState.velocity)
    );
  }
}
