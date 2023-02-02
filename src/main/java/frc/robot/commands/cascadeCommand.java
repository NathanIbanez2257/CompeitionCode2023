package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.cascade;

public class cascadeCommand extends CommandBase {

  cascade cascadeSub;
  double speed;

  public cascadeCommand(cascade cascade, double cascadeSpeed) {

    cascadeSub = cascade;
    speed = cascadeSpeed;
    addRequirements(cascadeSub);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    cascadeSub.move(speed);
    
    //cascadeSub.cascadeLimit(true);
  }

  @Override
  public void end(boolean interrupted) {
    cascadeSub.move(0);

  }

  @Override
  public boolean isFinished() {
    //cascadeSub.cascadeLimit(false);
    return false;
  }

}
