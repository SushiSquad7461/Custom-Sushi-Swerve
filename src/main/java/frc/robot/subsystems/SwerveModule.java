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
import SushiFrcLib.Math.Rotation2;
import SushiFrcLib.Math.Vector2;
import SushiFrcLib.Motor.MotorHelper;

public class SwerveModule extends SubsystemBase {
  private final WPI_TalonFX drive;
  private final WPI_TalonFX turn;
  private final CANCoder canCoder;
  
  private double newAngle;
  private boolean turnToNewAngle = false;

  public SwerveModule(int driveCanId, TalonFXInvertType driveInversion,int turnCanId, int canCoderPort) {
    drive = MotorHelper.createFalconMotor(driveCanId, kSwerveModule.DRIVE_CURRENT_LIMIT, driveInversion);
    turn = MotorHelper.createFalconMotor(turnCanId, kSwerveModule.TURN_CURRENT_LIMIT, TalonFXInvertType.CounterClockwise);
    canCoder = new CANCoder(canCoderPort);
  }

  public void updateModule(Vector2 newPos) {
    if (Math.abs(newPos.getAngle().toDegrees() - canCoder.getPosition()) >= kSwerveModule.ERROR_BOUND) {
        turnToNewAngle = true;
        newAngle = newPos.getAngle().toDegrees();
    } else {
        turnToNewAngle = false;
    }

    drive.set(ControlMode.Velocity, newPos.length);
  }

  @Override
  public void periodic() {
    if (turnToNewAngle) {
        if (Math.abs(newAngle - canCoder.getPosition()) <= kSwerveModule.ERROR_BOUND) {
            turnToNewAngle = false;
        } else if ((newAngle - canCoder.getPosition()) > 0) {
            turn.set(ControlMode.PercentOutput, kSwerveModule.TURN_MOTOR_SPEED);
        } else {
            turn.set(ControlMode.PercentOutput, -kSwerveModule.TURN_MOTOR_SPEED);
        }
    }
  }

  @Override
  public void simulationPeriodic() { }
}
