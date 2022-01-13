// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.MathUtils.applyDeadband;
import static frc.robot.util.MathUtils.clamp;
import static frc.robot.util.MathUtils.normalize;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DriveSignal;

public class DriveBase extends PIDSubsystem {
  // hardware
  // motors
  private TalonFX leftLeader, rightLeader;
  private TalonFX leftFollower, rightFollower;
  MotorControllerGroup leftMotors, rightMotors;
  // pneumatics
  private Solenoid shifter; // gear shifter
  // imu
  private AHRS gyro;
  // rio
  //private DigitalInput leftBarSensor, rightBarSensor;
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
    super(new PIDController(Constants.Drive.VelocityControlkP, 0, 0));
    leftLeader = new TalonFX(Constants.Drive.LeftLeaderId);
    leftFollower = new TalonFX(Constants.Drive.LeftFollowerId);
    rightLeader = new TalonFX(Constants.Drive.RightLeaderId);
    rightFollower = new TalonFX(Constants.Drive.RightFollowerId);

    leftLeader.configOpenloopRamp(Constants.Drive.DriveRampRate);
    leftFollower.configOpenloopRamp(Constants.Drive.DriveRampRate);
    rightLeader.configOpenloopRamp(Constants.Drive.DriveRampRate);
    rightFollower.configOpenloopRamp(Constants.Drive.DriveRampRate);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    gyro = new AHRS(Port.kMXP);
    gyro.reset();

    //leftBarSensor = new DigitalInput(Constants.Drive.LeftPhotoeyePort);
    //rightBarSensor = new DigitalInput(Constants.Drive.RightPhotoeyePort);
    
    // set motor status update rate at 100Hz (every 10ms) and contents to primary PID selected sensor feedback
    leftLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
    rightLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

    //shifter = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Drive.ShifterSolenoidId);

    // engage brakes when neutral input
    setBrakeMode(false);

    // setup encoders
    leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    leftLeader.setSensorPhase(false);
    leftLeader.setInverted(true);
    leftFollower.setInverted(true);

    rightLeader.setSensorPhase(false);
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);
    
    //leftMotors = new MotorControllerGroup((WPI_TalonFX)leftLeader, (WPI_TalonFX)leftFollower);
    //rightMotors = new MotorControllerGroup((WPI_TalonFX)rightLeader, (WPI_TalonFX)rightFollower);
  }

  // hardware methods
  public static TalonFX tunePID(final TalonFX talon, final int slotId, final double P, final double I, final double D, final double F, final double iZone, final double rampRate)
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
      throttle = Math.abs(throttle) * throttle;
      turn = Math.abs(turn) * turn;
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

  

  public void toggleGear()
  {
    isHighGear = !isHighGear;
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

  public void stop()
  {
    setTankDrive(DriveSignal.NEUTRAL);
  }

  public double getEncoderPosition()
  {
    return ((leftLeader.getSelectedSensorPosition() + leftFollower.getSelectedSensorPosition()) / 2) 
      + ((rightLeader.getSelectedSensorPosition() + rightFollower.getSelectedSensorPosition()) / 2);
  }

  public WPI_TalonFX[] getWPIMotors()
  {
    WPI_TalonFX[] motors = new WPI_TalonFX[4];
    motors[0] = (WPI_TalonFX)leftLeader;
    motors[1] = (WPI_TalonFX)leftFollower;
    motors[2] = (WPI_TalonFX)rightLeader;
    motors[3] = (WPI_TalonFX)rightFollower;
    return motors;
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    setTankDrive(new DriveSignal(output, output));
  }

  @Override
  protected double getMeasurement() {
    return leftLeader.getSelectedSensorVelocity();
  }
}
