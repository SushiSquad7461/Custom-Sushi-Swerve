// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerveModule;
import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Math.Rotation2;
import SushiFrcLib.Math.Vector2;
import SushiFrcLib.Motor.MotorHelper;

public class SwerveModule extends SubsystemBase {
  private final WPI_TalonFX drive;
  private final WPI_TalonFX turn;
  private final CANCoder canCoder;
  private final double angleOffest;

  private double newAngle;
  private boolean turnToNewAngle = false;
  private int turnInversion = 1;

  public SwerveModule(int driveCanId, TalonFXInvertType driveInversion,int turnCanId, int canCoderPort, double angleOffest) {
    drive = MotorHelper.createFalconMotor(driveCanId, kSwerveModule.DRIVE_CURRENT_LIMIT, driveInversion);
    turn = MotorHelper.createFalconMotor(turnCanId, kSwerveModule.TURN_CURRENT_LIMIT, TalonFXInvertType.CounterClockwise);
    canCoder = new CANCoder(canCoderPort);
    this.angleOffest = angleOffest;

    drive.config_kP(0, kSwerveModule.kPDrive);
    drive.config_kI(0, kSwerveModule.kIDrive);
    drive.config_kD(0, kSwerveModule.kDDrive);
    drive.config_kF(0, kSwerveModule.kFDrive);
  }

  public double getModuleAngle() {
    return Conversion.normalizeAngle(canCoder.getPosition() - angleOffest);
  }

  public void updateModule(Vector2 newPos) {
    if (Math.abs(newPos.getAngle().toDegrees() - getModuleAngle()) >= kSwerveModule.ERROR_BOUND) {
        turnToNewAngle = true;
        newAngle = newPos.getAngle().toDegrees();

        // Optimization
        if (Math.abs(newAngle - getModuleAngle()) > 180) {
            turnInversion = -1;
        } else {
            turnInversion = 1;
        }
    } else {
        turnToNewAngle = false;
    }

    drive.set(ControlMode.Velocity, newPos.length);
  }

  @Override
  public void periodic() {
    if (turnToNewAngle) {
        if (Math.abs(newAngle - getModuleAngle()) <= kSwerveModule.ERROR_BOUND) {
            turnToNewAngle = false;
        } else if ((newAngle - canCoder.getPosition()) > 0) {
            turn.set(ControlMode.PercentOutput, kSwerveModule.TURN_MOTOR_SPEED * turnInversion);
        } else {
            turn.set(ControlMode.PercentOutput, -kSwerveModule.TURN_MOTOR_SPEED * turnInversion);
        }
    }
  }

  @Override
  public void simulationPeriodic() { }
}
