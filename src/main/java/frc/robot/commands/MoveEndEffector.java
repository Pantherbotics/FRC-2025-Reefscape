// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.RobotStates.EEState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.EndEffector.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveEndEffector extends SequentialCommandGroup {
  /** Creates a new MoveEndEffector. */
  public MoveEndEffector(Elevator elevator, Pivot pivot, EEState endState) {
    addCommands(
      new ConditionalCommand(
        pivot.setAngleCommand(Rotations.of(Math.min(endState.angle.in(Rotations), RobotStates.kMoveAngle.in(Rotations))))
          .andThen(elevator.setHeightCommand(endState.distance))
          .andThen(pivot.setAngleCommand(endState.angle)),
        pivot.setAngleCommand(endState.angle)
          .alongWith(elevator.setHeightCommand(endState.distance)), 
        ()->RobotStates.needsSafeMovement(elevator.elevatorPosition(), pivot.pivotAngle(), endState)
      )
    );
    addRequirements(elevator, pivot);
  }
}
