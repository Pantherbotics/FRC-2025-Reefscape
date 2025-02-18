// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

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
  private final Debouncer m_debouncer = new Debouncer(RollerConstants.kDebounceTime, DebounceType.kBoth);
  private final Alert m_lasercanAlert = new Alert("Lasercan Bad reading", AlertType.kError);

  public Rollers() {
    m_rollersMotor.configure(RollerConstants.kMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  private boolean laserCANReading(){
    LaserCan.Measurement measurement = m_laserCan.getMeasurement();
    // return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT ? measurement.distance_mm < RollerConstants.kThreshold : false;
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      m_lasercanAlert.set(false);
      return measurement.distance_mm < RollerConstants.kThreshold;
    } else {
      m_lasercanAlert.set(true);
      return false;
    }
  }

  public boolean hasCoral() {
    return m_debouncer.calculate(laserCANReading());
  }

  public Command setRollerSpeed(Voltage voltage) {
    return this.run(() ->{ m_rollersMotor.setVoltage(voltage); SmartDashboard.putNumber(
      "roller speed", voltage.in(Volts));});
  }

  public Command setRollerPosition(Supplier<Angle> wheelAngle) {
    return this.startRun(
      ()->m_rollersMotor.getEncoder().setPosition(wheelAngle.get().plus(Degrees.of(90)).in(Rotations) * RollerConstants.kMotorToWheelRatio),
      () -> m_controller.setReference(wheelAngle.get().plus(Degrees.of(90)).in(Rotations) * RollerConstants.kMotorToWheelRatio, ControlType.kPosition)
    );
  }

  // public Command seatCoral(){
  //   return setRollerSpeed(RollerConstants.kSeatVoltage)
  //   .until(()->!hasCoral())

  //   .andThen(Commands.runOnce(()->SmartDashboard.putString("status debug", "No coral")))

  //   .andThen(Commands.waitSeconds(1))

  //   .andThen(setRollerSpeed(RollerConstants.kBackVoltage))

  //   .until(this::hasCoral)

  //   .andThen(Commands.runOnce(()->SmartDashboard.putString("status debug", "Has coral")))
    
  //   .andThen(Commands.waitSeconds(0.5))
  //   .andThen(setRollerSpeed(Volts.zero()).until(()->true))
    
  //   .andThen(Commands.runOnce(()->SmartDashboard.putString("status debug", "Stopped")));
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("roller pos", Units.rotationsToDegrees(m_rollersMotor.getEncoder().getPosition()));
    SmartDashboard.putBoolean("Has coral?", hasCoral());
    
  } 
}
