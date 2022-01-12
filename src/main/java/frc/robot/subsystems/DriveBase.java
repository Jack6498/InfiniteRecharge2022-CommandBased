// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.DriveSignal;

public class DriveBase extends SubsystemBase {
  // hardware
  // motors
  private TalonFX leftLeader, rightLeader;
  private TalonFX leftFollower, rightFollower;
  // pneumatics
  private Solenoid shifter; // gear shifter
  // imu
  private AHRS gyro;
  // rio
  private DigitalInput leftBarSensor, rightBarSensor;
  private static DriveBase instance;

  /**
   * Driving Mode<p>
   * Available Modes are Open Loop (Manual), Base Locked, Velocity Setpoint, and Trajectory Following.<p>
   * WARNING: ONLY OPEN LOOP IS CURRENTLY IMPLEMENTED
   */
  public enum DriveControlMode
  {
    OPEN_LOOP, 
    BASE_LOCKED, 
    VELOCITY_SETPOINT, 
    TRAJECTORY_FOLLOWING
  }

  public static DriveBase getInstance()
  {
    if (instance == null) {
      instance = new DriveBase();
    }
    return instance;
  }

  private DriveBase()
  {
    leftLeader = new TalonFX(Constants.Drive.LeftLeaderId);
    leftFollower = new TalonFX(Constants.Drive.LeftFollowerId);
    rightLeader = new TalonFX(Constants.Drive.RightLeaderId);
    rightFollower = new TalonFX(Constants.Drive.RightFollowerId);

    leftLeader.configOpenloopRamp(Constants.Drive.driveRampRate);
    leftFollower.configOpenloopRamp(Constants.Drive.driveRampRate);
    rightLeader.configOpenloopRamp(Constants.Drive.driveRampRate);
    rightFollower.configOpenloopRamp(Constants.Drive.driveRampRate);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    gyro = new AHRS(Port.kMXP);

    leftBarSensor = new DigitalInput(Constants.Drive.LeftPhotoeyePort);
    rightBarSensor = new DigitalInput(Constants.Drive.RightPhotoeyePort);
    
    // set motor status update rate at 100Hz (every 10ms) and contents to primary PID selected sensor feedback
    leftLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
    rightLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

    shifter = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Drive.ShifterSolenoidId);

    // engage brakes when neutral input
    setBrakeMode(true);

    // setup encoders
    leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    leftLeader.setSensorPhase(false);
    leftLeader.setInverted(true);
    leftFollower.setInverted(true);

    rightLeader.setSensorPhase(false);
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);

    // load velocity PID gains
    leftLeader = tunePID(leftLeader, Constants.Drive.VelocityControlSlot, Constants.Drive.VelocityControlkP, Constants.Drive.VelocityControlkI, Constants.Drive.VelocityControlkD, Constants.Drive.VelocityControlkF, Constants.Drive.VelocityControlIZone, Constants.Drive.VelocityControlRampRate);
    rightLeader = tunePID(rightLeader, Constants.Drive.VelocityControlSlot, Constants.Drive.VelocityControlkP, Constants.Drive.VelocityControlkI, Constants.Drive.VelocityControlkD, Constants.Drive.VelocityControlkF, Constants.Drive.VelocityControlIZone, Constants.Drive.VelocityControlRampRate);
    // load base lock PID gains
    leftLeader = tunePID(leftLeader, Constants.Drive.BaseLockSlot, Constants.Drive.BaseLock_kP, Constants.Drive.BaseLock_kI, Constants.Drive.BaseLock_kD, Constants.Drive.BaseLock_kF, Constants.Drive.BaseLockIZone, Constants.Drive.BaseLockRampRate);
    rightLeader = tunePID(rightLeader, Constants.Drive.BaseLockSlot, Constants.Drive.BaseLock_kP, Constants.Drive.BaseLock_kI, Constants.Drive.BaseLock_kD, Constants.Drive.BaseLock_kF, Constants.Drive.BaseLockIZone, Constants.Drive.BaseLockRampRate);
  }

  private DriveControlMode driveControlMode;
  private boolean brakeEngaged = true; //  we are braking by default

  public static TalonFX tunePID(final TalonFX talon, final int slotId, final double P, final double I, final double D, final double F, final int iZone, final double rampRate)
  {
    talon.config_kP(slotId, P);
    talon.config_kI(slotId, I);
    talon.config_kD(slotId, D);
    talon.config_kF(slotId, F);
    talon.config_IntegralZone(slotId, iZone);
    talon.configClosedloopRamp(rampRate);
    return talon;
  }

  boolean setBrakeMode(boolean brake_on)
  {
    return false;
  }
  
  public void setTankDrive(final DriveSignal signal)
  {
    if(driveControlMode != DriveControlMode.OPEN_LOOP)
    {
      driveControlMode = DriveControlMode.OPEN_LOOP;
    }
    leftLeader.set(ControlMode.PercentOutput, signal.leftMotor);
    rightLeader.set(ControlMode.PercentOutput, signal.rightMotor);
  }

  public void setArcadeDrive(double throttle, double turn)
  {

  }
}
