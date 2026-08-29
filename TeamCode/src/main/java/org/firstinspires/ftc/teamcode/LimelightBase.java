package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LimelightBase", group = "Main")
public class LimelightBase extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addLine("Initializing Limelight...");
        telemetry.update();

        // Start Limelight communication
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            telemetry.addData("Result Null?", result == null);

            if (result != null) {

                telemetry.addData("Valid?", result.isValid());

                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());

                int fiducialCount = result.getFiducialResults().size();
                telemetry.addData("Fiducial Count", fiducialCount);

                if (fiducialCount > 0) {
                    LLResultTypes.FiducialResult tag =
                            result.getFiducialResults().get(0);

                    telemetry.addData("First Tag ID", tag.getFiducialId());
                    telemetry.addData("X deg", tag.getTargetXDegrees());
                    telemetry.addData("Y deg", tag.getTargetYDegrees());
                } else {
                    telemetry.addLine("NO FIDUCIALS RECEIVED");
                }
            }

            telemetry.update();
        }

        limelight.stop();
    }
}