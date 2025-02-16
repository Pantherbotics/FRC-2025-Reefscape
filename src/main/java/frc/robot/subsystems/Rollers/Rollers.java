// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Rollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import au.grapplerobotics.LaserCan;

public class Rollers extends SubsystemBase {
  private final LaserCan m_laserCan = new LaserCan(RollerConstants.kLaserCANID);
  private final SparkFlex m_rollersMotor = new SparkFlex(RollerConstants.kRollersMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_rollersMotor.getClosedLoopController();

  public Rollers() {
    m_rollersMotor.configure(RollerConstants.kMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public boolean hasCoral() {
    LaserCan.Measurement measurement = m_laserCan.getMeasurement();
    // return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT ? measurement.distance_mm < RollerConstants.kThreshold : false;
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm < RollerConstants.kThreshold;
    } else {
      return false;
    }
  }

  public Command setRollerSpeed(Voltage voltage) {
    return this.runOnce(() -> m_rollersMotor.setVoltage(voltage));
  }

  public Command setRollerPosition(Angle wheelAngle) {
    return this.run(() -> m_controller.setReference(wheelAngle.in(Rotations) * RollerConstants.kMotorToWheelRatio, ControlType.kPosition));
  }

  public Command seatCoral(){
    return setRollerSpeed(RollerConstants.kSeatVoltage)
    .until(()->!hasCoral())
    .andThen(setRollerSpeed(RollerConstants.kBackVoltage))
    .until(this::hasCoral)
    .andThen(setRollerSpeed(Volts.zero()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has coral?", hasCoral());
    
  } 
}
