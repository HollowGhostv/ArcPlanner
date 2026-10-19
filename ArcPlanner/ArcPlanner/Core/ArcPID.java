package org.firstinspires.ftc.teamcode.ArcPlanner.Core;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Clase que implementa un controlador PID con términos de ganancia proporcional,
 * integral, derivativa, feedforward (kS), limitación integral y tolerancia.
 */
public class ArcPID
{
    // Constantes y variables de estado del controlador PID
    private double kP, kI, kD;
    private double kS = 0.0;
    private double lastError = 0;
    private double integralSum = 0;
    private double lastDerivative = 0;

    // Límites y tolerancias configurables
    private double maxI = 1.0;
    private double tolerance = 0.0;

    // Temporizador para medir el tiempo transcurrido (delta time) entre ciclos
    private ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor para inicializar las ganancias básicas P, I y D.
     */
    public ArcPID(double p, double i, double d)
    {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    /**
     * Configura el parámetro de feedforward (kS).
     */
    public void setFeedforward(double kS)
    {
        this.kS = kS;
    }

    /**
     * Configura el límite máximo para la acumulación integral (anti-windup).
     */
    public void setIntegralLimit(double maxI) {
        this.maxI = maxI;
    }

    /**
     * Configura la tolerancia dentro de la cual el error se considera cero.
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Calcula la salida del controlador PID en función del valor objetivo y el valor actual.
     */
    public double calculate(double target, double current)
    {
        // Cálculo del error actual
        double error = target - current;

        // Obtención del tiempo transcurrido y protección contra división por cero
        double deltaTime = timer.seconds();
        if (deltaTime <= 0) deltaTime = 1e-6;

        // Si el error está dentro de la tolerancia establecida, se resetea la integral y se retorna 0
        if (Math.abs(error) < tolerance)
        {
            integralSum = 0;
            lastError = error;
            return 0;
        }

        // Cálculo del término proporcional
        double p = kP * error;

        // Acumulación y limitación del término integral
        integralSum += error * deltaTime;
        integralSum = Math.max(-maxI, Math.min(maxI, integralSum));
        double i = kI * integralSum;

        // Cálculo de la derivada basada en el cambio de error respecto al tiempo y filtrado
        double derivative = (error - lastError) / deltaTime;
        derivative = 0.8 * derivative + 0.2 * lastDerivative;
        double d = kD * derivative;

        // Cálculo del término feedforward basado en la dirección del error
        double ff = Math.signum(error) * kS;

        // Actualización de estados anteriores y reinicio del temporizador
        lastDerivative = derivative;
        lastError = error;
        timer.reset();

        // Suma de todos los componentes del controlador
        double output = p + i + d + ff;

        // Retorno de la salida limitada entre -1 y 1
        return Math.max(-1, Math.min(1, output));
    }
}