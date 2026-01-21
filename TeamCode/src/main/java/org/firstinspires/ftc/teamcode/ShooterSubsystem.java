package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterSubsystem {
    private final DcMotorEx shooterMotor;
    private final double MotorTicksPerRev = 28;
    private final double MotorGearRatio = (double) 1/19.2;

    private static final PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
            5,
            0,
            0,
            0
    );
    public ShooterSubsystem(DcMotorEx shooterMotor) {
        this.shooterMotor = shooterMotor;
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //this.shooterMotor.setMode(Dc);
    }

    public void revToRPM(double RPM) {
        this.shooterMotor.setVelocity(rpmToTicksPerSecond(RPM));
    }
    public void stop() {
        this.shooterMotor.setVelocity(0);
        //try set voltage
    }

    public double rpmToTicksPerSecond(double rpm) {
        return rpm * MotorTicksPerRev / MotorGearRatio / 60;
    }
}
