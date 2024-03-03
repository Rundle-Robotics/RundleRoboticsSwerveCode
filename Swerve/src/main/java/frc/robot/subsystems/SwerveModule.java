// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SwerveModule extends SubsystemBase {

  private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
  /** Creates a new SwerveModule. */

  // four identical swerve modules can use identical base code 
  public SwerveModule(int driveMotorId, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, 
  int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    turnMotor.setInverted(turningMotorReversed);
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0 , 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getDrivePosition(){
    return driveEncoder.getPosition();
  
  }
  public double getTurningPosition(){
    return turningEncoder.getPosition();
  
  }
  public double getDriveVelocity(){
    return driveEncoder.getVelocity();

  }
  public double getTurningVelocity(){
    return turningEncoder.getVelocity();

  }
  public double getAbsolutEncoderRad(){
    double angle = absoluteEncoder.getVoltage()/RobotContainer.getVoltage5V();
    // need to change by voltage supplied 
    angle *=2.0*Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed? -1.0:1);
  }

    public void resetEncoders(){
      driveEncoder.setPosition(0);
      turningEncoder.setPosition(getAbsolutEncoderRad());
    }

    public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }
    public void setDesiredState(SwerveModuleState state){
      if (Math.abs(state.speedMetersPerSecond)<0.001){
        stop();
        return;

      }
      state = SwerveModuleState.optimize(state,getState().angle);
      driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
      turnMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
      SmartDashboard.putString("Swerve[" + absoluteEncoder.getVoltage() + "] state", state.toString());

      //getChannel()
      //????????
    }
    public void stop(){
      driveMotor.set(0);
      turnMotor.set(0);
      
    }
  }

