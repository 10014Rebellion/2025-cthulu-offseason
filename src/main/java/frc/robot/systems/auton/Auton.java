package frc.robot.systems.auton;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class Auton {
    public Auton() {
        initNamedCommands();
    }

    public void initNamedCommands() {
        NamedCommands.registerCommand("ReadyAlgaeL3", readyAlgaeCmd(true));
        NamedCommands.registerCommand("ReadyAlgaeL2", readyAlgaeCmd(false));
        NamedCommands.registerCommand("ReadyCoral", readyCoralCmd());
        NamedCommands.registerCommand("ScoreCoral", scoreCoralCmd());
        NamedCommands.registerCommand("ShootAlgae", shootAlgaeCmd());
        NamedCommands.registerCommand("ScoreAlgae", shootAlgaeCmd()); // Just in case we miswrite it.
    }

    public Command readyAlgaeCmd(boolean isL3){
        return null;
    }

    public Command readyCoralCmd(){
        return null;
    }

    public Command scoreCoralCmd(){
        return null;
    }

    public Command shootAlgaeCmd(){
        return null;
    }
}
