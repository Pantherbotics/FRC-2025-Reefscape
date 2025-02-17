// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntake extends SubsystemBase {
  private final Servo m_pivotServo = new Servo(CoralIntakeConstants.kServoID);
  private double goal = 0.0;
  public CoralIntake() {}

  public Command setServoPosition(double position){
    return this.runOnce(()->{
      goal = MathUtil.clamp(position, 0.0, 1.0);
      m_pivotServo.set(goal);
    });
  }

  public boolean pivotAtGoal(){
    return MathUtil.isNear(goal, m_pivotServo.get(), CoralIntakeConstants.kPositionTolerance);
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake pivot at goal", pivotAtGoal());
  }
}
