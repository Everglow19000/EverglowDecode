package org.firstinspires.ftc.teamcode.subsystems;

import static org.opencv.core.Core.mean;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;

public class Camera implements Subsystem{

    private class FindLocationAction implements Action{
        private double[] location;
        private Pose3D[] locations;
        private int index = 0;

        // location is a list of size 3 [x,y,heading]
        public FindLocationAction(double[] location, int amount) {
            locations = new Pose3D[amount];
            this.location = location;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            LLResult result = limelight3A.getLatestResult();
            locations[index] = result.getBotpose();
            index++;

            if (index >= locations.length) {
                for (Pose3D pose3D : locations) {
                    location[0] += pose3D.getPosition().x / locations.length;
                    location[1] += pose3D.getPosition().y / locations.length;
                    location[2] += pose3D.getOrientation().getYaw() / locations.length;
                }
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
        limelight3A.pipelineSwitch(1);
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