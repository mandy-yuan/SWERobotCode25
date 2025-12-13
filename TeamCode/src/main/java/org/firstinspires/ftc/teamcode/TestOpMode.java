package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestOpMode extends OpMode {
    private Servo spindexerServo;
    private DcMotorEx intakeMotor;
    private SpindexerSubsystem spindexerSubsystem;
    private DcMotorEx shooterMotor;
    private Servo serializerServo;

    private ShooterSubsystem shooterSubsystem;
    public void init() {
        spindexerServo = hardwareMap.get(Servo.class, "spindexer");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        spindexerSubsystem = new SpindexerSubsystem(spindexerServo);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterSubsystem = new ShooterSubsystem(shooterMotor);
        serializerServo = hardwareMap.get(Servo.class, "serializer");
    }

    public void start() {

    }
    public void loop() {
        if (gamepad2.yWasPressed()) {
            spindexerSubsystem.rotateSpindexer();
        }
        if (gamepad2.xWasPressed()) {
            spindexerSubsystem.rotateSpindexerShooter();
        }
        //if (gamepad2.rightBumperWasPressed()) {
        //    spindexerSubsystem.rotateSpindexerIntake();
        //}

        if (gamepad2.right_trigger > 0) {
            shooterSubsystem.revToRPM(6000);
        }
        else {
            shooterSubsystem.stop();
        }

        if (gamepad2.a) {
            serializerServo.setPosition(0.7);
        }
        else {
            serializerServo.setPosition(0.48);
        }

        if (gamepad2.b) {
            intakeMotor.setPower(-0.5);
        }
        else {
            intakeMotor.setPower(0);
        }
    }
    public void stop() {

    }
}
