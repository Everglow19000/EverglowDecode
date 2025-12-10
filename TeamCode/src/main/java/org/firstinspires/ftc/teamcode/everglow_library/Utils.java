package org.firstinspires.ftc.teamcode.everglow_library;

import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class Utils {
    public static Vector2d rotateByAngle(Vector2d vector, double angle) {
        double newX = vector.x*Math.cos(angle) - vector.y*Math.sin(angle);
        double newY = vector.x*Math.sin(angle) + vector.y*Math.cos(angle);

        return new Vector2d(newX, newY);
    }

    public static InterpLUT interpLUTFromArrays(double[] inputs, double[] outputs) {
        if (inputs.length != outputs.length) {
            throw new IllegalArgumentException("Inputs and Outputs must be the same length!");
        }

        InterpLUT lookUpTable = new InterpLUT();
        for (int i = 0; i < inputs.length; i++) {
            lookUpTable.add(inputs[i], outputs[i]);
        }

        lookUpTable.createLUT();
        return lookUpTable;
    }

    public static void setServoPWMRange(Servo servo, double usPulseLower, double usPulseUpper) {
        LynxServoController lynxServoController = (LynxServoController) servo.getController();
        lynxServoController.setServoPwmRange(servo.getPortNumber(), new PwmControl.PwmRange(usPulseLower, usPulseUpper));
    }

    public static Rotation2d getOptimalAngleToShoot(Vector2d goalEdgeLowerX, Vector2d goalEdgeHigherX, Vector2d botPose) {
        if (goalEdgeLowerX.x > goalEdgeHigherX.x) {
            return getOptimalAngleToShoot(goalEdgeHigherX, goalEdgeLowerX, botPose);
        }

        double goalLineSlope = (goalEdgeLowerX.y-goalEdgeHigherX.y)/(goalEdgeLowerX.x-goalEdgeHigherX.x);
        double goalLineOffset = (-goalLineSlope*goalEdgeLowerX.x) + goalEdgeLowerX.y;

        double goalLinePerpendicularSlope = -1.0/goalLineSlope;
        double goalLinePerpendicularOffset = (-goalLinePerpendicularSlope*botPose.x) + botPose.y;

        double closestPointX = (goalLinePerpendicularOffset - goalLineOffset)/(goalLineSlope - goalLinePerpendicularSlope);

        if (closestPointX < goalEdgeLowerX.x) {
            closestPointX = goalEdgeLowerX.x;
        }
        else if (closestPointX > goalEdgeHigherX.x) {
            closestPointX = goalEdgeHigherX.x;
        }

        double closestPointY = (goalLineSlope*closestPointX) + goalLineOffset;

        return Rotation2d.exp(Math.atan2(closestPointY - botPose.y, closestPointX - botPose.x));
    }
}
