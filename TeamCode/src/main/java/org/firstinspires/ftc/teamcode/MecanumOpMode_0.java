package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MecanumOpMode_0", group = "Drive")
public class MecanumOpMode_0 extends OpMode {

    DcMotor front_left, front_right, back_left, back_right;
    IMU imu;

    // 5 speed levels — left bumper = slower, right bumper = faster
    private static final double[] SPEED_MODES  = { 0.25, 0.40, 0.60, 0.80, 1.00 };
    private static final String[] SPEED_LABELS = { "25%", "40%", "60%", "80%", "100%" };
    private int speedIndex = 2; // default 60%

    // Front-right motor runs faster than the others on this chassis.
    // Reduce its output by this factor to keep the robot tracking straight.
    // Tune FR_SCALE down (e.g. 0.90) until driving straight feels true.
    private static final double FR_SCALE = 0.93;

    // Headless (field-centric) mode — toggled by B, heading reset by Y
    private boolean headlessMode    = false;
    private boolean prevB           = false;
    private boolean prevY           = false;

    // Edge-detection state for bumpers
    private boolean prevLeftBumper  = false;
    private boolean prevRightBumper = false;

    @Override
    public void init() {
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        // Motor directions
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // Brake on zero power — hard stop when sticks are released
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Encoder-regulated velocity (RUN_USING_ENCODER uses the PID controller
        // built into the motor driver hub so all four wheels hold their set speed
        // independently — this also helps compensate for the FR motor variance)
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU — built into the REV Control Hub.
        // Adjust logoFacingDirection / usbFacingDirection to match how your
        // Control Hub is physically mounted on the robot.
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);
        imu.resetYaw();

        telemetry.addData("Status", "Initialized — waiting for start");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ── HEADLESS TOGGLE (B) & HEADING RESET (Y) ───────────────────────────
        boolean bPressed = gamepad1.b;
        boolean yPressed = gamepad1.y;

        if (bPressed && !prevB) {
            headlessMode = !headlessMode;
        }
        if (yPressed && !prevY) {
            imu.resetYaw(); // re-zero "forward" to current robot facing
        }

        prevB = bPressed;
        prevY = yPressed;

        // ── SPEED CONTROL ─────────────────────────────────────────────────────
        boolean leftBumper  = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        if (leftBumper && !prevLeftBumper) {
            speedIndex = Math.max(0, speedIndex - 1);
        }
        if (rightBumper && !prevRightBumper) {
            speedIndex = Math.min(SPEED_MODES.length - 1, speedIndex + 1);
        }

        prevLeftBumper  = leftBumper;
        prevRightBumper = rightBumper;

        double speedMultiplier = SPEED_MODES[speedIndex];

        // ── INPUTS ────────────────────────────────────────────────────────────
        double x  =  gamepad1.left_stick_x;   // strafe
        double y  = -gamepad1.left_stick_y;   // forward/back (inverted stick)
        double rx =  gamepad1.right_stick_x;  // rotation

        // Deadzone — prevents motor chatter when sticks sit slightly off-center
        if (Math.abs(x)  < 0.05) x  = 0;
        if (Math.abs(y)  < 0.05) y  = 0;
        if (Math.abs(rx) < 0.05) rx = 0;

        // ── FIELD-CENTRIC ROTATION ─────────────────────────────────────────────
        // When headless mode is active, rotate the stick vector by the robot's
        // current yaw so pushing "up" always means field-forward regardless of
        // which way the robot is facing.
        double driveX = x;
        double driveY = y;

        if (headlessMode) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate input vector opposite to robot heading
            driveX = x * Math.cos(-heading) - y * Math.sin(-heading);
            driveY = x * Math.sin(-heading) + y * Math.cos(-heading);
        }

        // ── MECANUM KINEMATICS ────────────────────────────────────────────────
        // Standard axial / lateral / yaw mixing:
        //   front_left  = +y + x + rx
        //   front_right = +y - x - rx
        //   back_left   = +y - x + rx
        //   back_right  = +y + x - rx
        double fl = driveY + driveX + rx;
        double fr = driveY - driveX - rx;
        double bl = driveY - driveX + rx;
        double br = driveY + driveX - rx;

        // Normalize so no value exceeds 1.0
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        // Apply front-right hardware correction before scaling
        fr *= FR_SCALE;

        // Apply speed multiplier
        fl *= speedMultiplier;
        fr *= speedMultiplier;
        bl *= speedMultiplier;
        br *= speedMultiplier;

        // ── DRIVE OUTPUT ──────────────────────────────────────────────────────
        if (x == 0 && y == 0 && rx == 0) {
            // Explicit zero lets BRAKE behavior take over immediately
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);
        } else {
            front_left.setPower(fl);
            front_right.setPower(fr);
            back_left.setPower(bl);
            back_right.setPower(br);
        }

        // ── TELEMETRY ─────────────────────────────────────────────────────────
        double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Mode", headlessMode ? "HEADLESS (field-centric)" : "ROBOT-centric");
        telemetry.addData("Heading", "%.1f°  (Y = reset)", headingDeg);
        telemetry.addData("Speed", "[ LB ] " + SPEED_LABELS[speedIndex]
                + " [ RB ]  (" + (speedIndex + 1) + "/5)");
        telemetry.addData("FL / FR", "%.2f  /  %.2f", fl, fr);
        telemetry.addData("BL / BR", "%.2f  /  %.2f", bl, br);
        telemetry.addData("Enc FL/FR", "%d  /  %d",
                front_left.getCurrentPosition(), front_right.getCurrentPosition());
        telemetry.addData("Enc BL/BR", "%d  /  %d",
                back_left.getCurrentPosition(), back_right.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
}
