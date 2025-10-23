package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.time.Instant;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class NeoDiagnostics extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder enc;
    // Config
    private final double TEST_OUTPUT = 0.10; // 10%
    private final int CURRENT_LIMIT_A = 35;
    private final double STEADY_WINDOW_S = 1.0; // 1 second
    private final double SAMPLING_DT = 0.02; // 20 ms

    public NeoDiagnostics(int canId) {
        motor = new SparkMax(canId, MotorType.kBrushless);
        enc = motor.getEncoder();
        SparkBaseConfig driveMotorConfig = new SparkMaxConfig();
        driveMotorConfig.smartCurrentLimit(CURRENT_LIMIT_A, CURRENT_LIMIT_A);
        driveMotorConfig.disableFollowerMode();
        driveMotorConfig.inverted(true);
        driveMotorConfig.idleMode(IdleMode.kBrake);
        motor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void runTest() {
        // Run three windows: idle, forward, reverse
        Window idle = sampleWindow(0.0);
        Window fwd = sampleWindow(TEST_OUTPUT);
        Window rev = sampleWindow(-TEST_OUTPUT);

        motor.set(0.0);

        // Compute inferred resistances (rough)
        double fwdI = Math.max(1e-6, fwd.avgI - idle.avgI);
        double revI = Math.max(1e-6, rev.avgI - idle.avgI);
        double fwdR = fwd.avgV / fwdI;
        double revR = rev.avgV / revI;

        // Determine issues
        List<String> issues = new ArrayList<>();
        if (Double.isNaN(fwdR) || Double.isInfinite(fwdR) || fwdR <= 0)
            issues.add("Bad forward resistance reading");
        if (Double.isNaN(revR) || Double.isInfinite(revR) || revR <= 0)
            issues.add("Bad reverse resistance reading");
        double relDiff = Math.abs(fwdR - revR) / Math.max(fwdR, revR);
        if (relDiff > 0.5)
            issues.add("Forward/reverse R differ >50%");
        if (idle.avgI > 5.0)
            issues.add("Idle current >5A (possible drag)");
        if (fwd.avgI > CURRENT_LIMIT_A * 0.9 || rev.avgI > CURRENT_LIMIT_A * 0.9)
            issues.add("Current near limit");

        boolean pass = issues.isEmpty();

        // Push results to SmartDashboard
        SmartDashboard.putBoolean("neoTestPass", pass);
        SmartDashboard.putNumber("neoIdleAvgCurrentA", idle.avgI);
        SmartDashboard.putNumber("neoFwdAvgCurrentA", fwd.avgI);
        SmartDashboard.putNumber("neoRevAvgCurrentA", rev.avgI);
        SmartDashboard.putNumber("neoFwdAvgVoltageV", fwd.avgV);
        SmartDashboard.putNumber("neoRevAvgVoltageV", rev.avgV);
        SmartDashboard.putNumber("neoFwdInferredROhm", fwdR);
        SmartDashboard.putNumber("neoRevInferredROhm", revR);
        SmartDashboard.putNumber("neoIdleVel", idle.avgVel);
        SmartDashboard.putNumber("neoFwdVel", fwd.avgVel);
        SmartDashboard.putNumber("neoRevVel", rev.avgVel);
        SmartDashboard.putString("neoTestIssues", String.join(", ", issues));

        System.out.println("NEO test complete. Pass: " + pass);
        if (!issues.isEmpty())
            System.out.println("Issues: " + String.join("; ", issues));
    }

    private static class Window {
        double avgV, avgI, avgVel;
    }

    private Window sampleWindow(double output) {
        motor.set(output);
        double start = Timer.getFPGATimestamp();
        double end = start + STEADY_WINDOW_S;
        Window w = new Window();
        int count = 0;

        while (Timer.getFPGATimestamp() < end) {
            w.avgV += motor.getBusVoltage();
            w.avgI += motor.getOutputCurrent();
            w.avgVel += enc.getVelocity();
            count++;
            try {
                Thread.sleep((long) (SAMPLING_DT * 1000));
            } catch (InterruptedException e) {
            }
        }
        motor.set(0.0);
        if (count > 0) {
            w.avgV /= count;
            w.avgI /= count;
            w.avgVel /= count;
        }
        return w;
    }
}
