package org.firstinspires.ftc.teamcode.subsystems;

public enum Motif {
    PPG(ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN),
    PGP(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE),
    GPP(ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE),
    NONE(ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE);
    public ArtifactColor[] sequence;

    private Motif(ArtifactColor color0, ArtifactColor color1, ArtifactColor color2) {
        sequence = new ArtifactColor[3];
        sequence[0] = color0;
        sequence[1] = color1;
        sequence[2] = color2;
    }
}
