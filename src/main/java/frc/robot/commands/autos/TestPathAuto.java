package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;

public class TestPathAuto extends Command {
  private final String pathName;

  public TestPathAuto(String pathName) {
    this.pathName = pathName;
  }

  @Override
  public void initialize() {
    try {
      Command pathCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));

      pathCommand.schedule();
    } catch (Exception e) {
      System.out.println("Unable to run auto");
      e.printStackTrace();
    }
  }
}
