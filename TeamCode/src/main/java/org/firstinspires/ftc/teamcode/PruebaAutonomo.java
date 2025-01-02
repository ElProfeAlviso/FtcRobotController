package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="PruebaAutonomo", group="Linear OpMode")
public class PruebaAutonomo extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // Define the target distance in inches
    // Define the robot's wheel circumference (in inches)
    private static final double WHEEL_CIRCUMFERENCE = 3.54331* Math.PI; // Example: 4-inch diameter wheels

    // Define the encoder ticks per revolution (TPR) of your motor
    private static final int TICKS_PER_REV =  420;//yMark NeveRest 40

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
driveDistance(15);
sleep(500);
        driveDistance(-15);
        sleep(500);
        // Reset encoders

    }  private void driveDistance(double TARGET_DISTANCE) {
        // Reset encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate target encoder ticks
        int targetTicks = (int) ((TARGET_DISTANCE / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV);

        // Set target position and run to position
        leftDrive.setTargetPosition(targetTicks);
        rightDrive.setTargetPosition(targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motors
        leftDrive.setPower(0.4);
        rightDrive.setPower(0.4);

        right

        // Wait for motors to reach target position
        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            // Display telemetry data (optional)
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Left Position", leftDrive.getCurrentPosition());
            telemetry.addData("Right Position", rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}




