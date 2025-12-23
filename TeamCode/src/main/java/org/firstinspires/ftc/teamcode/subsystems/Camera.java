package org.firstinspires.ftc.teamcode.subsystems;

import static org.opencv.core.Core.mean;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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
                for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
                    if (fiducialResult.getFiducialId() == 21 || fiducialResult.getFiducialId() == 22 || fiducialResult.getFiducialId() == 23) {
                        motifWrapper[0] = Motif.values()[fiducialResult.getFiducialId() - 21];
                        isFinished = true;
                    }
                }
            }

            if ((System.currentTimeMillis() - startTime) > timeUntilBail && motifWrapper[0] == null) {
                motifWrapper[0] = Motif.NONE;
            }

            return !isFinished && (System.currentTimeMillis() - startTime) <= timeUntilBail;
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
                Pose3D currPose = null;

                for (int i = 0; i < result.getFiducialResults().size(); i++) {
                    currPose = result.getFiducialResults().get(i).getRobotPoseFieldSpace();
                }

                if (currPose != null) {
                    locations[index] = currPose;
                }
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

    public void setPipeline(int pipeline) {
        limelight3A.pipelineSwitch(pipeline);
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
        LLResult currResult = limelight3A.getLatestResult();
        if (currResult.isValid()) {
            return "result valid";
        }
        return "result not valid";
    }
}