package org.firstinspires.ftc.teamcode.subsystems;

import static org.opencv.core.Core.mean;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;

public class Camera implements Subsystem{
    public class DetermineMotifAction implements Action {
        private boolean hasStarted = false;
        private boolean isFinished = false;
        private Motif[] motifWrapper;
        private double timeUntilBail;
        private double startTime;
        private DetermineMotifAction(double timeUntilBail, Motif[] motifWrapper) {
            this.motifWrapper = motifWrapper;
            this.timeUntilBail = timeUntilBail;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasStarted) {
                hasStarted = true;

                startTime = System.currentTimeMillis();
                limelight3A.pipelineSwitch(2);
            }

            LLResult result = limelight3A.getLatestResult();
            if (result.isValid()) {
                if (result.getFiducialResults().get(0) != null) {
                    int discoveredId = result.getFiducialResults().get(0).getFiducialId();
                    motifWrapper[0] = Motif.values()[discoveredId - 21];
                    isFinished = true;
                }
            }

            isFinished = (System.currentTimeMillis() - startTime) > timeUntilBail || isFinished;

            return !isFinished;
        }
    }

    public class FindLocationAction implements Action{
        private double[] location;
        private Pose3D[] locations;
        private int index = 0;
        private boolean hasStarted = false;

        // location is a list of size 3 [x,y,heading]
        public FindLocationAction(double[] location, int amount) {
            locations = new Pose3D[amount];
            this.location = location;
            location[0] = 0.0;
            location[1] = 0.0;
            location[2] = 0.0;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasStarted) {
                hasStarted = true;

                limelight3A.pipelineSwitch(1);
            }
            LLResult result = limelight3A.getLatestResult();
            if (result.isValid()) {
                locations[index] = result.getBotpose();
                index++;
            }

            if (index >= locations.length) {
                for (Pose3D pose3D : locations) {
                    location[0] += pose3D.getPosition().toUnit(DistanceUnit.INCH).x / locations.length;
                    location[1] += pose3D.getPosition().toUnit(DistanceUnit.INCH).y / locations.length;
                }
                location[2] += locations[locations.length-1].getOrientation().getYaw(AngleUnit.RADIANS);
                return false;
            }
            return true;
        }
    }

    private Limelight3A limelight3A;

    public Camera(HardwareMap hardwareMap) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void start() {
        limelight3A.start();
    }

    public DetermineMotifAction getDetermineMotifAction(Motif[] motifWrapper) {
        return new DetermineMotifAction(500.0, motifWrapper);
    }

    public FindLocationAction getFindLocationAction(double[] location, int amount) {
        return new FindLocationAction(location, amount);
    }

    @Override
    public void update(int iterationCount) {

    }

    @Override
    public String status() {
        return "";
    }
}