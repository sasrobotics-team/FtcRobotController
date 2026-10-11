package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RobotTeleOp", group = "Main")
public class RobotTeleOp extends OpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shooter;
    private Servo s1;

    // Servo presets (0.0 - 1.0 range).
    private static final double SERVO_POS_A = 0.3333; // "3722" from your telemetry reading
    private static final double SERVO_POS_B = 0.1000;

    // Shooter spins at half power, clockwise (looking at shaft head-on).
    private static final double SHOOTER_POWER = 0.5;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean servoAtA = false; // false -> at B (0.0)
    private boolean shooterRunning = false;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        s1 = hardwareMap.get(Servo.class, "S1");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        // FORWARD on the motor used to give CCW at the shaft. Clockwise is the
        // opposite, so REVERSE here. If it spins the wrong way, flip to FORWARD.
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Park the servo at B (0.0) on init so first A press goes to 0.3722.
        s1.setPosition(SERVO_POS_B);

        telemetry.addLine("Ready. A = toggle servo, B = toggle shooter (CW, half power).");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Tank drive.
        double leftStick = -gamepad1.left_stick_y;
        double rightStick = -gamepad1.right_stick_y;
        backLeft.setPower(leftStick);
        backRight.setPower(rightStick);

        // A: toggle servo between the two presets.
        boolean aPressed = gamepad1.a;
        if (aPressed && !prevA) {
            servoAtA = !servoAtA;
            s1.setPosition(servoAtA ? SERVO_POS_A : SERVO_POS_B);
        }
        prevA = aPressed;

        // B: toggle shooter on/off, half power, clockwise.
        boolean bPressed = gamepad1.b;
        if (bPressed && !prevB) {
            shooterRunning = !shooterRunning;
            shooter.setPower(shooterRunning ? SHOOTER_POWER : 0.0);
        }
        prevB = bPressed;

        telemetry.addData("backLeft pwr", "%.2f", leftStick);
        telemetry.addData("backRight pwr", "%.2f", rightStick);
        telemetry.addData("S1 position", "%.4f", s1.getPosition());
        telemetry.addData("servo at A", servoAtA);
        telemetry.addData("shooter running", shooterRunning);
        telemetry.addData("shooter pwr", "%.2f", shooter.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        backLeft.setPower(0);
        backRight.setPower(0);
        shooter.setPower(0);
    }
}
