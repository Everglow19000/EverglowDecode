package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.everglow_library.Utils;

public enum ArtifactColor {
    GREEN  (new double[] {0.1862348178137652, 0.46963562753036436, 0.3441295546558704}),
    PURPLE (new double[] {0.24472573839662448, 0.350210970464135, 0.4050632911392405}),
    NONE   (new double[] {-1,-1,-1});

    public final double[] color;
    ArtifactColor(double[] color) {
        this.color = Utils.normalizeArray(color, true);
    }
    public static double[] NormalizedRGBAToArray(NormalizedRGBA NRGBA) {
        return Utils.normalizeArray(new double[] {
                (double)NRGBA.red,
                (double)NRGBA.green,
                (double)NRGBA.blue},
                true);
    }
}
