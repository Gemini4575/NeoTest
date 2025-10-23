package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeoDiagnostics;

public class NeoTest extends Command {

    NeoDiagnostics n;

    public NeoTest(NeoDiagnostics tn) {
        n = tn;
        addRequirements(n);
    }

    @Override
    public void execute() {
        n.runTest();
    }
}
