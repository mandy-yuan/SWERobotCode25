package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class ShooterTest extends OpMode{
    public DcMotorEx shooter;
    private static final double P = 0.0;
    private static final double F = 0.0;

    public double highVelocity = 5000;
    public double lowVelocity = 3000;
    double currentTarget = highVelocity;

    @Override
    public void init(){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init completed");
    }

    @Override
    public void loop(){
        //switch from far to near
        if (gamepad1.yWasPressed()) {
            if (currentTarget == highVelocity) {
                currentTarget = lowVelocity;
            } else {
                currentTarget = highVelocity;
            }
        }

        //shooter test
        if(gamepad2.right_trigger>0){
           shooter.setVelocity(currentTarget);
        }
        else{
            shooter.setPower(0);
        }
        telemetry.addData("Target", currentTarget);
        telemetry.addData("Velocity", shooter.getVelocity());
        telemetry.update();
    }
}
