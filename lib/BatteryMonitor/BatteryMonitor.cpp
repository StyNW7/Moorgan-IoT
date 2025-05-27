#include "esp_adc_cal.h" // For ADC calibration
#include <Tools.h>

// Voltage Divider Resistors (Update if you ever change them)
const float R1_VALUE = 100000.0; // 100k Ohms
const float R2_VALUE = 39000.0;  // 39k Ohms

// ADC Voltage Range AT THE ADC PIN (after diode drop and voltage division)
// These values are based on V_battery_max = 8.4V, V_battery_min = 6.0V, V_diode_fwd = 0.35V
const float ADC_MAX_VOLTAGE_AT_PIN = 2.258; // Corresponds to ~8.05V after diode (8.4V battery)
const float ADC_MIN_VOLTAGE_AT_PIN = 1.585; // Corresponds to ~5.65V after diode (6.0V battery)

// ADC Calibration (ESP32 specific)
static esp_adc_cal_characteristics_t adc_chars;
// ADC Attenuation - 11dB gives a range up to ~2.5V-3.1V, suitable for our max ADC pin voltage
const adc_atten_t ATTENUATION = ADC_ATTEN_DB_11;
const adc_attenuation_t ATTEN = ADC_11db;
const adc_unit_t ADC_UNIT = ADC_UNIT_1; // Use ADC1

// Number of samples to average for ADC reading
const int ADC_SAMPLES = 32;

// --- End Configuration ---

// Function to map voltage at ADC pin to battery percentage
float mapVoltageToPercentage(float voltageAtAdcPin) {
    float percentage;

    // Clamp voltage to the defined min/max range for percentage calculation
    if (voltageAtAdcPin >= ADC_MAX_VOLTAGE_AT_PIN) {
        percentage = 100.0;
    } else if (voltageAtAdcPin <= ADC_MIN_VOLTAGE_AT_PIN) {
        percentage = 0.0;
    } else {
        // Linear mapping
        percentage = (voltageAtAdcPin - ADC_MIN_VOLTAGE_AT_PIN) * 100.0 / (ADC_MAX_VOLTAGE_AT_PIN - ADC_MIN_VOLTAGE_AT_PIN);
    }
    
    // For more accuracy, consider a multi-point lookup table or a non-linear function
    // based on your battery's specific discharge curve.
    
    return percentage;
}

void setupBatteryMon() {
    analogReadResolution(12); // 12-bit ADC
    analogSetPinAttenuation(BATTERYADCPIN, ATTEN); // Set attenuation for the pin

    // Characterize ADC for Vref calibration
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT, ATTENUATION, ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_DEFAULT_VREF, &adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        xprintln("ADC: Using eFuse Vref");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        xprintln("ADC: Using Two Point eFuse Vref");
    } else {
        xprintln("ADC: Using Default Vref (Consider calibrating for more accuracy)");
    }
}

float getBatteryCapacity() {
    uint32_t adc_reading_raw = 0;
    // Take multiple samples
    for (int i = 0; i < ADC_SAMPLES; i++) {
        adc_reading_raw += analogRead(BATTERYADCPIN);
    }
    adc_reading_raw /= ADC_SAMPLES;

    // Convert raw ADC reading to millivolts using calibration data
    uint32_t voltage_mv_at_adc = esp_adc_cal_raw_to_voltage(adc_reading_raw, &adc_chars);
    float voltage_at_adc_pin = voltage_mv_at_adc / 1000.0;

    // Calculate the voltage before the divider (after the diode)
    float voltage_after_diode = voltage_at_adc_pin * (R1_VALUE + R2_VALUE) / R2_VALUE;

    // Estimate battery voltage before the diode (optional, less precise due to variable Vf)
    const float DIODE_FORWARD_VOLTAGE_ESTIMATE = 0.35; // Adjust if you know your diode's Vf better
    float estimated_battery_voltage_before_diode = voltage_after_diode + DIODE_FORWARD_VOLTAGE_ESTIMATE;

    // Get battery percentage based on voltage at the ADC pin
    float battery_percentage = mapVoltageToPercentage(voltage_at_adc_pin);

    xprint("Raw ADC: ");
    xprint(adc_reading_raw);
    xprint("  |  V@ADC Pin: ");
    xprintf("%.3f", voltage_at_adc_pin);
    xprint("V");
    xprint("  |  V Post-Diode: ");
    xprintf("%.2f", voltage_after_diode);
    xprint("V");
    xprint("  |  Est. V Batt: ");
    xprintf("%.2f", estimated_battery_voltage_before_diode);
    xprint("V");
    xprint("  |  Percentage: ");
    xprintf("%.1f", battery_percentage);
    xprintln("%");

    return battery_percentage;
}