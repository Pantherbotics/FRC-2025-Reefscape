// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntake extends SubsystemBase {
  public final Servo m_pivotServo = new Servo(CoralIntakeConstants.kServoID);
  private final TalonFXS m_RollersMotor = new TalonFXS(CoralIntakeConstants.kRollersMotorID); // change the ID once wired

  private double goal = 0.0;
  public CoralIntake() {
    m_pivotServo.setBoundsMicroseconds(1950, 1750, 1500, 1250, 1050);
    m_RollersMotor.getConfigurator().apply(CoralIntakeConstants.kMotorConfig);
    // m_pivotServo.setZeroLatch();
    SmartDashboard.putData(this);
  }

  public Command setServoPosition(double position) {
    return this.runOnce(() -> {
      goal = MathUtil.clamp(position, 0.0, 1.0);
      m_pivotServo.set(goal);
    });
  }

  public Command disableServo(){
    return this.runOnce(()->m_pivotServo.setDisabled());
  }

  public Command setSpeed(double speed){
    return this.runOnce(()->m_pivotServo.setSpeed(speed));
  }

  public Command setPulseWidth(int width){
    return this.runOnce(()-> m_pivotServo.setPulseTimeMicroseconds(width));
  }

  public Command setServoHigh() {
    return this.runOnce(() -> m_pivotServo.setAlwaysHighMode());
  }

  public Command setRollersVoltage(Voltage volts) {
    return this.runOnce(() -> m_RollersMotor.setVoltage(volts.in(Volts)));
  }

  public boolean pivotAtGoal(){
    return MathUtil.isNear(goal, m_pivotServo.get(), CoralIntakeConstants.kPositionTolerance);
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake pivot at goal", pivotAtGoal());
    SmartDashboard.putNumber("Commanded servo position", m_pivotServo.get());
    SmartDashboard.putNumber("Actual servo position", m_pivotServo.getPosition());
  }
}
