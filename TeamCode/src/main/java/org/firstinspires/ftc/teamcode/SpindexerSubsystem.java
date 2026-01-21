package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

public class SpindexerSubsystem {
    private final Servo spindexerServo;
    Timer timer = new Timer();

    private int spindexerState;
    private int spindexerShooterState;
    private double offset = 0.1;

    public SpindexerSubsystem(Servo spindexerServo) {
        this.spindexerServo = spindexerServo;
        spindexerServo.setPosition(0);
        spindexerState = 0;
        spindexerShooterState = 0;
    }

    public void rotateSpindexerIntake() {
        switch (spindexerState) {
            case 0:
                spindexerServo.setPosition(-0.05);
                spindexerState = 1;
                break;
            case 1:
                spindexerServo.setPosition(0.35);
                spindexerState = 2;
                break;
            case 2:
                spindexerServo.setPosition(0.75);
                spindexerState = 0;
                break;

        }
    }

    public void rotateSpindexerShooter() {
        switch (spindexerShooterState) {
            case 0:
                spindexerServo.setPosition(0.15);
                spindexerShooterState = 1;
                break;
            case 1:
                spindexerServo.setPosition(0.55);
                spindexerShooterState = 2;
                break;
            case 2:
                spindexerServo.setPosition(0.95);
                spindexerShooterState = 0;
                break;
        }
    }
}
