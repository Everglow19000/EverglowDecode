package org.firstinspires.ftc.teamcode.everglow_library;

import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class Utils {
    public static Vector2d rotateByAngle(Vector2d vector, double angle) {
        double newX = vector.x*Math.cos(angle) - vector.y*Math.sin(angle);
        double newY = vector.x*Math.sin(angle) + vector.y*Math.cos(angle);

        return new Vector2d(newX, newY);
    }

    public static InterpLUT interpLUTFromArrays(int[] inputs, int[] outputs) {
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
}
