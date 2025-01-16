package org.firstinspires.ftc.teamcode;

//Importacion de librerias

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Propiedades de visualizacion Driver Station.
@Autonomous(name="Auto Moves", group="Autos",preselectTeleOp="TitaniumRamsRegional")

//Clase principal del programa (IMU utilizado modelo:
public class MovimientosAutos extends LinearOpMode {

    // Declarar los objetos de motores
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private DcMotor wrist = null;
    private Servo claw = null;
    private CRServo intake = null;

    private BHI260IMU imu;
    private Orientation angulos;
    private DistanceSensor distanceSensor;
    private double distancia;

    // Definir el objetivo en pulgadas
    // Definir la circunferencia del rueda en pulgadas
    private static final double WHEEL_CIRCUMFERENCE = 3.54331* Math.PI; // Diametro de las ruedas 3.54331 pulgadas

    // Define el encoder ticks por revolution (TPR) del motor
    private static final int TICKS_PER_REV =  420;//yMark NeveRest 40

    // Arm and Wrist target positions for each state
    //Posiciones de Arm en diferentes configuraciones.
    private static final int ARM_POSITION_INIT = 170;
    private static final int ARM_POSITION_INTAKE = 300;
    private static final int ARM_POSITION_WALL_GRAB = 920;
    private static final int ARM_POSITION_WALL_UNHOOK = 1270;
    private static final int ARM_POSITION_HOVER_HIGH = 1950;
    private static final int ARM_POSITION_CLIP_HIGH = 1500;
    private static final int ARM_POSITION_LOW_BASKET = 1870;

    //Posiciones de WRIST en diferentes configuraciones.
    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_SAMPLE = 230;
    private static final int WRIST_POSITION_SPEC = 10;
    private static final int WRIST_POSITION_BASKET = 285;



    // Posiciones de CLAW en diferentes configuraciones.
    private static final double CLAW_OPEN_POSITION = 0.50;
    private static final double CLAW_CLOSED_POSITION = 0.65;

    // ENUM para asignación de los diferentes estados del robot.
    private enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
    }

    // Enviar robot a posición inicial.
    private RobotState currentState = RobotState.INIT;

    // CLAW toggle state.
    private boolean clawOpen = true;
    private boolean lastBump = false;
    private boolean lastHook = false;
    private boolean lastGrab = false;

    //target position de arm y Wrist.
    private int targetArm = 0;
    private int targetWrist = 0;

    @Override
    public void runOpMode() {
        // inicializa las variables de hardware, los strings son los nombres de parametros en la driver station.
        // La funcion get obtiene el mapeo de la configuracion de hardware en el driver station.
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");


        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to use encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Se establece la direccion de los motores.Debido a que los motores estan invertidos uno respeco al otro,
        // se invierte el motor derecho.

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Se establece el modo de freno al estar en neutral.
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Inicialización del IMU BHI260AP

        // Los parámetros son limitados en BHI260AP,
        // generalmente no se necesitan configuraciones adicionales.

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancia");
        distancia = distanceSensor.getDistance(DistanceUnit.CM);

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters parametrosIMU = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));


        imu.initialize(parametrosIMU);


        // Esperar a que el IMU esté listo (puede no ser necesario para BHI260AP)
        sleep(100); // Pausa corta para asegurar la inicialización

        telemetry.addData("IMU", "Inicializado");
        telemetry.addData("Distancia Init", distancia);
        telemetry.update();

        // La funcion waitForStart() espera a que el boton de play sea presionado.

        waitForStart();


        //===================SECUENCIA DE COMANDOS AUTONOMOS====================================
        CerrarGarra();
        RobotState(RobotState.INIT);
        Adelante(30, 0.6, 100);
        Atras(10, 0.6, 100);
        girarIzquierda(90, 0.6, 100);
        Adelante(40, 0.6, 100);
        girarDerecha(90, 0.5, 100);
        Adelante(55, 0.6, 100);
        girarIzquierda(90, 0.6, 100);
        Adelante(5, 0.6, 100);
        girarDerecha(100, 0.6, 100);
        Atras(70, 0.6, 100);
        Adelante(70, 0.6, 100);
        girarIzquierda(100, 0.6, 100);
        Adelante(10, 0.6, 100);
        girarDerecha(100, 0.6, 100);
        Atras(70, 0.6, 100);
        Adelante(70, 0.6, 100);
        girarIzquierda(100, 0.6, 100);
        Adelante(10, 0.6, 100);
        girarDerecha(90, 0.6, 100);
        Atras(70, 0.6, 100);




        }



        //================================FUNCIONES DE INTAKE=================================










        //================================FUNCIONES DE MOVIMIENTOS=================================

      private void Adelante(double TARGET_DISTANCE ,double Power, long SLEEPTIME) {
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
        leftDrive.setPower(Power);
        rightDrive.setPower(Power);

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

        sleep(SLEEPTIME);
    }


    private void Atras(double TARGET_DISTANCE ,double Power, long SLEEPTIME) {
        TARGET_DISTANCE = -TARGET_DISTANCE;
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
        leftDrive.setPower(Power);
        rightDrive.setPower(Power);

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

        sleep(SLEEPTIME);
    }


    // Función para girar a la izquierda (grados positivos)
    private void girarIzquierda(double grados , double Power, long SLEEPTIME) { girar((grados - 5),Power,SLEEPTIME);}

    // Función para girar a la derecha (grados negativos)
    private void girarDerecha(double grados , double Power, long SLEEPTIME) { girar(-(grados -5) ,Power,SLEEPTIME); }

    // Función principal para realizar el giro
    private void girar(double grados, double Power, long SLEEPTIME) {

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();
        angulos = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double anguloInicial = angulos.firstAngle;
        double anguloObjetivo = anguloInicial + grados;

        // Ajustar el ángulo objetivo para que esté entre -180 y 180 grados
        anguloObjetivo = anguloObjetivo % 360;
        if (anguloObjetivo > 180) {
            anguloObjetivo -= 360;
        } else if (anguloObjetivo < -180) {
            anguloObjetivo += 360;
        }

        while (opModeIsActive() && Math.abs(angulos.firstAngle - anguloObjetivo) > 2) {
            angulos = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double potencia = (grados > 0 ? Power : -Power); // Ajustar la potencia según la dirección

            leftDrive.setPower(-potencia);
            rightDrive.setPower(potencia);

            telemetry.addLine("==Angulo del IMU==");
            telemetry.addData("Ángulo actual", angulos.firstAngle);
            telemetry.addData("Ángulo objetivo", anguloObjetivo);
            telemetry.addLine("**Potencias del Drive**");
            telemetry.addData("LeftDrive", leftDrive.getPower());
            telemetry.addData("RightDrive", rightDrive.getPower());
            telemetry.update();
        }


        leftDrive.setPower(0);
        rightDrive.setPower(0);

        sleep(SLEEPTIME);
    }

    private void AbrirGarra(){

        claw.setPosition(CLAW_OPEN_POSITION);

    }

    private void CerrarGarra() {

        claw.setPosition(CLAW_CLOSED_POSITION);
    }

    private void IntakeMeter (){
        intake.setPower(1.0);

    }

    private void IntakeSacar(){

        intake.setPower(-1.0);
    }

    private void IntakeStop (){
        intake.setPower(0);
    }

    private void RobotState(RobotState currentState) {

        while (opModeIsActive()) {

            switch (currentState) {
                case INIT:
                    targetArm = ARM_POSITION_INIT;
                    targetWrist = WRIST_POSITION_INIT;
                    telemetry.addData("State", "INIT");
                    break;
                case INTAKE:
                    targetArm = ARM_POSITION_INTAKE;
                    targetWrist = WRIST_POSITION_SAMPLE;
                    telemetry.addData("State", "INTAKE");
                    break;

                case WALL_GRAB:
                    targetArm = ARM_POSITION_WALL_GRAB;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "WALL_GRAB");
                    break;

                case WALL_UNHOOK:
                    targetArm = ARM_POSITION_WALL_UNHOOK;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "WALL_UNHOOK");
                    break;

                case HOVER_HIGH:
                    targetArm = ARM_POSITION_HOVER_HIGH;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "HOVER_HIGH");
                    break;

                case CLIP_HIGH:
                    targetArm = ARM_POSITION_CLIP_HIGH;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "CLIP_HIGH");
                    break;
                case LOW_BASKET:
                    targetArm = ARM_POSITION_LOW_BASKET;
                    targetWrist = WRIST_POSITION_BASKET;
                    telemetry.addData("State", "LOW_BASKET");
                    break;
                case MANUAL:
                    telemetry.addData("State", "MANUAL");
                    break;
            }


            arm.setTargetPosition(targetArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setTargetPosition(targetWrist);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            wrist.setPower(1);

            // Send telemetry data to the driver station
            telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Wrist Position", wrist.getCurrentPosition());
            telemetry.addData("Wrist Power", wrist.getPower());
            telemetry.update();
        }

    }
}




