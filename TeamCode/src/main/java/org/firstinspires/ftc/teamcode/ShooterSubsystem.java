package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterSubsystem {
    private final DcMotorEx shooterMotor;
    private final double MotorTicksPerRev = 28;
    private final double MotorGearRatio = (double) 1/19.2;

    private static final PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
            3,
            0,
            0,
            0
    );
    public ShooterSubsystem(DcMotorEx shooterMotor) {
        this.shooterMotor = shooterMotor;
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //this.shooterMotor.setMode(Dc);
    }

    public void revToRPM(double RPM) {
        this.shooterMotor.setVelocity(rpmToTicksPerSecond(RPM));
    }
    public void stop() {
        this.shooterMotor.setVelocity(0);
    }

    public double rpmToTicksPerSecond(double rpm) {
        return rpm * MotorTicksPerRev / MotorGearRatio / 60;
    }
}
