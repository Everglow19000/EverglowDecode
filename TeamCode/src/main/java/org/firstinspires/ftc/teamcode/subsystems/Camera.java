package org.firstinspires.ftc.teamcode.subsystems;

import static org.opencv.core.Core.magnitude;
import static org.opencv.core.Core.mean;

import android.util.Log;

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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;

import java.util.List;

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
        private boolean isMT2;

        // location is a list of size 3 [x,y,heading]
        public FindLocationAction(double[] location, int amount, boolean isMT2) {
            this.isMT2 = isMT2;
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
            if (isMT2) {
                limelight3A.updateRobotOrientation(Math.toDegrees(localizer.getPose().heading.toDouble()));
            }
            LLResult result = limelight3A.getLatestResult();
            if (result.isValid()) {
                if (isMT2) {
                    locations[index] = result.getBotpose_MT2();
                }
                else {
                    locations[index] = result.getBotpose();
                }
                index++;
            }

            if (index >= locations.length) {
                for (Pose3D pose3D : locations) {
                    location[0] += pose3D.getPosition().toUnit(DistanceUnit.INCH).x / locations.length;
                    location[1] += pose3D.getPosition().toUnit(DistanceUnit.INCH).y / locations.length;
                    if (!isMT2) {
                        location[2] += pose3D.getOrientation().getYaw(AngleUnit.RADIANS) / locations.length;
                    }
                }
                if (isMT2) {
                    location[2] += localizer.getPose().heading.toDouble();
                }
//                else {
//                    location[2] /= locations.length;
//                }
                return false;
            }
            return true;
        }
    }

    public Limelight3A limelight3A;
    private Localizer localizer;

    public Camera(HardwareMap hardwareMap, Localizer localizer) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        this.localizer = localizer;
    }

    public void start() {
        limelight3A.start();
    }

    public void setPipeline(int pipeline) {
        limelight3A.pipelineSwitch(pipeline);
    }
    // returns -1 if result is invalid
    public double getDistanceFromAprilTag(boolean isBlue) {
        LLResult result = limelight3A.getLatestResult();
        if (result.isValid()) {
            if (result.getPipelineIndex() != 1) {
                limelight3A.pipelineSwitch(1);
            }
            else {
                int wantedTagID = isBlue ? 20 : 24;

                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                Pose3D pose = null;

                for (int i = 0; i < fiducialResults.size(); i++) {
                    if (fiducialResults.get(i).getFiducialId() == wantedTagID) {
                        pose = fiducialResults.get(i).getRobotPoseTargetSpace();
                    }
                }

                if (pose == null) {
                    return -1;
                }

                return Math.sqrt(
                        0
//                                + Math.pow(pose.getPosition().toUnit(DistanceUnit.INCH).x, 2)
//                            + Math.pow(pose.getPosition().toUnit(DistanceUnit.INCH).y, 2)
                                + Math.pow(pose.getPosition().toUnit(DistanceUnit.INCH).z, 2)
                );
            }
        }
        return -1;
    }

    public DetermineMotifAction getDetermineMotifAction(Motif[] motifWrapper) {
        return new DetermineMotifAction(500.0, motifWrapper);
    }

    public FindLocationAction getFindLocationAction(double[] location, int amount) {
        return new FindLocationAction(location, amount, true);
    }

    public FindLocationAction getFindLocationAction(double[] location, int amount, boolean isMT2) {
        return new FindLocationAction(location, amount, isMT2);
    }

    @Override
    public void update(int iterationCount) {
        limelight3A.updateRobotOrientation(Math.toDegrees(localizer.getPose().heading.toDouble()));
        LLResult result = limelight3A.getLatestResult();
        if (result.isValid()) {
            if (result.getPipelineIndex() != 1) {
                limelight3A.pipelineSwitch(1);
            }
            else {
                double x = result.getBotpose_MT2().getPosition().toUnit(DistanceUnit.INCH).x;
                double y = result.getBotpose_MT2().getPosition().toUnit(DistanceUnit.INCH).y;

                localizer.setPose(new Pose2d(x, y, localizer.getPose().heading.toDouble()));
            }
        }
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