package org.tahomarobotics.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.units.measure.Frequency;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Hertz;

public class RobustConfigurator {
    private static final Logger logger = LoggerFactory.getLogger(RobustConfigurator.class);

    /**
     * Times configurations are attempted to be applied
     */
    private static final int DEFAULT_RETRIES = 5;

    // Retrying Configurators

    /**
     * Attempts to run the configuration function until success or RETRIES.
     *
     * @param specifier Specifier for the device(s)
     * @param retries   Amount of times to retry
     * @param config    Configuration function
     * @return Resulting status code
     */
    @SuppressWarnings("SameParameterValue")
    private static StatusCode tryConfigure(String specifier, int retries, Supplier<StatusCode> config) {
        StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < retries; i++) {
            statusCode = config.get();
            if (statusCode.isOK()) {
                logger.info("Successfully configured {} in {} attempt{}!", specifier, i + 1, i == 0 ? "" : "s");
                break;
            } else if (statusCode.isWarning()) {
                logger.warn("[{}/{}] Configuring {} returned warning status code: {}, retrying...", i + 1, retries, specifier, statusCode);
            } else {
                logger.error("[{}/{}] Configuring {} returned error status code: {}, retrying...", i + 1, retries, specifier, statusCode);
            }
        }
        return statusCode;
    }

    // Device Configurators

    /**
     * Attempts to configure a TalonFX.
     *
     * @param deviceName    Name of the device
     * @param configurator  Configurator of the motor
     * @param configuration Configuration to apply
     * @return The resulting status code
     */
    public static StatusCode tryConfigureTalonFX(String deviceName, TalonFXConfigurator configurator, TalonFXConfiguration configuration) {
        return tryConfigure("TalonFX '" + deviceName + "'", DEFAULT_RETRIES, () -> configurator.apply(configuration));
    }

    /**
     * Attempts to modify the configuration of a TalonFX.
     *
     * @param deviceName   Name of the device
     * @param configurator Configurator of the motor
     * @param modification Modification to apply
     * @return The resulting status code
     */
    public static StatusCode tryModifyTalonFX(String deviceName, TalonFXConfigurator configurator, Consumer<TalonFXConfiguration> modification) {
        var config = new TalonFXConfiguration();
        configurator.refresh(config);
        modification.accept(config);

        return tryConfigureTalonFX(deviceName, configurator, config);
    }

    /**
     * Attempts to configure a CANcoder.
     *
     * @param deviceName    Name of the device
     * @param configurator  Configurator of the CANcoder
     * @param configuration Configuration to apply
     * @return The resulting status code
     */
    public static StatusCode tryConfigureCANcoder(String deviceName, CANcoderConfigurator configurator, CANcoderConfiguration configuration) {
        return tryConfigure("CANcoder '" + deviceName + "'", DEFAULT_RETRIES, () -> configurator.apply(configuration));
    }

    /**
     * Attempts to modify the configuration of a CANcoder.
     *
     * @param deviceName   Name of the device
     * @param configurator Configurator of the CANcoder
     * @param modification Modification to apply
     * @return The resulting status code
     */
    public static StatusCode tryModifyCANcoder(String deviceName, CANcoderConfigurator configurator, Consumer<CANcoderConfiguration> modification) {
        var config = new CANcoderConfiguration();
        configurator.refresh(config);
        modification.accept(config);

        return tryConfigureCANcoder(deviceName, configurator, config);
    }

    // Status Signals

    /**
     * Attempts to set the update frequency for all supplied signals.
     *
     * @param specifier Specifier for the signals' device(s)
     * @param frequency Frequency of updating [4Hz..1000Hz]
     * @param signals   Signals to update
     * @return The resulting status code
     */
    public static StatusCode trySetUpdateFrequencyForAll(String specifier, Frequency frequency, BaseStatusSignal... signals) {
        return tryConfigure("status signals for '" + specifier + "'", DEFAULT_RETRIES,
                () -> BaseStatusSignal.setUpdateFrequencyForAll(frequency.in(Hertz), signals));
    }

    /**
     * Attempts to optimize all unset signals on the devices.
     *
     * @param subsystem          Specifier for the devices' subsystem
     * @param optimizedFrequency The optimized frequency
     * @param devices            Devices to optimize
     * @return The resulting status code
     */
    public static StatusCode tryOptimizeBusUsageForAll(String subsystem, Frequency optimizedFrequency, ParentDevice... devices) {
        return tryConfigure("devices for '" + subsystem + "'", DEFAULT_RETRIES, () -> ParentDevice.optimizeBusUtilizationForAll(optimizedFrequency.in(Hertz), devices));
    }

    /**
     * Attempts to optimize all unset signals on the devices by disabling them.
     *
     * @param subsystem Specifier for the devices' subsystem
     * @param devices   Devices to optimize
     * @return The resulting status code
     */
    public static StatusCode tryOptimizeBusUsageForAll(String subsystem, ParentDevice... devices) {
        return tryConfigure("devices for '" + subsystem + "'", DEFAULT_RETRIES, () -> ParentDevice.optimizeBusUtilizationForAll(devices));
    }
}
