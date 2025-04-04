// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.RollerConstants;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
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
  private boolean isSeated = false;
  private boolean isSeating = false;

  public Rollers() {
    m_rollersMotor.configure(RollerConstants.kMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putData(this);
  }

  private boolean laserCANReading(){
    LaserCan.Measurement measurement = m_laserCan.getMeasurement();
    // return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT ? measurement.distance_mm < RollerConstants.kThreshold : false;
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm < RollerConstants.kThreshold;
    } else {
      return false;
    }
  }

  public boolean hasCoral() {
    return Utils.isSimulation()?true:laserCANReading();
  }

  public Command setRollerSpeed(Voltage voltage) {
    return this.run(() ->{
      if (voltage.in(Volts) != 0){isSeated = false;}
      m_rollersMotor.setVoltage(voltage); 
      SmartDashboard.putNumber("roller speed", voltage.in(Volts));});
  }
  
  public Command stopRollers(){
    return this.runOnce(()->m_rollersMotor.setVoltage(0));
  }

  public Command setSpeedForSeconds(Voltage voltage, Time time){
    return setRollerSpeed(voltage).raceWith(Commands.waitSeconds(time.in(Seconds)));
  }

  public Command setRollerPosition(Supplier<Angle> wheelAngle) {
    return this.startRun(
      ()->m_rollersMotor.getEncoder().setPosition(wheelAngle.get().plus(Degrees.of(90)).in(Rotations) * RollerConstants.kMotorToWheelRatio),
      () -> m_controller.setReference(wheelAngle.get().plus(Degrees.of(90)).in(Rotations) * RollerConstants.kMotorToWheelRatio, ControlType.kPosition)
    );
  }

  public Command smartIntake(){
    return
      Commands.runOnce(()->isSeating=true)
        .andThen(this.setRollerSpeed(RollerConstants.kSeatVoltage).until(this::hasCoral))
        .andThen(Commands.waitUntil(()-> m_rollersMotor.getOutputCurrent() > 30))
        .andThen(this.stopRollers().alongWith(Commands.runOnce(()->isSeated = true)))
        .andThen(Commands.waitSeconds(0.2))
        .andThen(Commands.runOnce(()->isSeating=false))
      ;
  }

  public Command smartOuttake(Voltage outtakeVoltage, Time outtakeSeconds){
    return
      this.setSpeedForSeconds(outtakeVoltage, outtakeSeconds)
      .andThen(
        new ConditionalCommand(
          Commands.sequence(
            this.setRollerSpeed(RollerConstants.kBackVoltage).until(this::hasCoral),
            this.setRollerSpeed(RollerConstants.kSeatVoltage).until(()->!hasCoral()),
            this.stopRollers().alongWith(Commands.runOnce(()->isSeated = true))
          ), 
         this.stopRollers(), 
          ()->m_rollersMotor.getOutputCurrent()>RollerConstants.kMaxCurrent));
  }

  public Command seatCoral(){
    return Commands.runOnce(()->isSeating=true)
    .alongWith(this.setRollerSpeed(RollerConstants.kSeatVoltage).until(()->!this.hasCoral()))
    .andThen(this.setRollerSpeed(RollerConstants.kBackVoltage).until(this::hasCoral))
    .andThen(Commands.waitSeconds(0.1))
    .andThen(this.stopRollers().alongWith(Commands.runOnce(()->isSeated = true)))
    .andThen(Commands.runOnce(()->isSeating=false));
  }

  public boolean isSeated(){
    return Utils.isSimulation()?true:isSeated;
  }

  public boolean isSeating(){
    return isSeating;
  }


  public boolean isCurrentSpike(){
    return m_rollersMotor.getOutputCurrent() > 40;
  }
  // Algae stuff


  @Override
  public void periodic() {
    SmartDashboard.putNumber("roller pos", Units.rotationsToDegrees(m_rollersMotor.getEncoder().getPosition()));
    SmartDashboard.putBoolean("Has coral?", hasCoral());
    SmartDashboard.putBoolean("isSeated", isSeated);
    SignalLogger.writeDouble("roller current", m_rollersMotor.getOutputCurrent());
    SmartDashboard.putNumber("roller current", m_rollersMotor.getOutputCurrent());
    SmartDashboard.putBoolean("isCurrentHit", isCurrentSpike());
  } 


}
