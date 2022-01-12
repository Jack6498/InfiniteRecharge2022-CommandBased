// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI.Port;
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
  private boolean isHighGear = false;
  private DriveControlMode driveControlMode;
  private boolean brakeEngaged = true; //  we are braking by default

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

  // hardware methods
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

  public AHRS getGyro()
  {
    return gyro;
  }

  public void resetSensors()
  {
    gyro.reset();
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
  }

  public Rotation2d getGyroAngle()
  {
    // negative angle because of the direction the gyro is mounted
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public void setBrakeMode(boolean brake_on)
  {
    if (brakeEngaged != brake_on) {
      if (brake_on) {
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
      } else {
        leftLeader.setNeutralMode(NeutralMode.Coast);
        rightLeader.setNeutralMode(NeutralMode.Coast);
      }
    }
    brakeEngaged = brake_on;
  }
  
  // driving modes
  public void setTankDrive(final DriveSignal signal)
  {
    if(driveControlMode != DriveControlMode.OPEN_LOOP)
    {
      driveControlMode = DriveControlMode.OPEN_LOOP;
    }
    leftLeader.set(ControlMode.PercentOutput, signal.leftMotor);
    rightLeader.set(ControlMode.PercentOutput, signal.rightMotor);
  }

  // blatantly stolen from Team 3494 (The Quadrangles) (love yall)
  public void setArcadeDrive(double throttle, double turn, boolean squareInputs)
  {
    throttle = applyDeadband(clamp(throttle), 0.05);
    turn = applyDeadband(clamp(turn), 0.05);

    // square inputs (but keep the sign) to increase fine control
    if (squareInputs) {
      throttle = Math.copySign(throttle * throttle, throttle);
      turn = Math.copySign(turn * turn, turn);
    }

    double leftMotorDemand, rightMotorDemand;
    double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(turn)), throttle);

    // are we going to move forward?
    if (throttle >= 0.0) 
    {
      // are we turning left?
      if (turn >= 0.0) 
      {
        leftMotorDemand = maxInput;
        rightMotorDemand = throttle - turn;
      }  
      else // we must be turning right
      {
        leftMotorDemand = throttle + turn;
        rightMotorDemand = maxInput;
      }
    }
    else // we must be going backwards
    {
      if (turn >= 0.0) // are we turning left?
      {
        leftMotorDemand = throttle + turn;
        rightMotorDemand = maxInput;
      }
      else // we must be turning right
      {
        leftMotorDemand = maxInput;
        rightMotorDemand = throttle - turn;
      }
    }
    double[] stickSpeeds = normalize(new double[]{leftMotorDemand, rightMotorDemand});
    setTankDrive(new DriveSignal(stickSpeeds[0], stickSpeeds[1]));
  }

  private double clamp(double value)
  {
    if (value > 1.0) 
    {
      return 1.0;
    } 
    else if (value < -1.0)
    {
      return -1.0;
    }
    else
    {
      return value;
    }

  }

  private double applyDeadband(double value, double deadband)
  {
    return value;
  }

  private double[] normalize(double[] motorSpeeds)
  {
    double max = Math.abs(motorSpeeds[0]);
    boolean normFlag = max > 1; // do we normalize or just pass it back?
    for (int i = 1; i < motorSpeeds.length; i++) {
      if (Math.abs(motorSpeeds[i]) > max) { // set max to biggest magnitude motor speed
        max = Math.abs(motorSpeeds[i]);
        normFlag = max > 1;
      }
    }
    if (normFlag) { // normalize speeds
      for (int i = 0; i < motorSpeeds.length; i++) {
        motorSpeeds[i] /= max;
      }
    }
    return motorSpeeds;
  }

  public void setGear(boolean highGearOn)
  {
    isHighGear = highGearOn;
    shifter.set(isHighGear);
  }

  public boolean getGear()
  {
    return isHighGear;
  }

  public DriveControlMode getControlMode()
  {
    return driveControlMode;
  }

  
}
