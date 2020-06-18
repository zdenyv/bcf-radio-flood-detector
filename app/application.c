#include <application.h>
#include <bc_ds18b20.h>

#define SERVICE_INTERVAL_INTERVAL (60 * 60 * 1000)
#define BATTERY_UPDATE_INTERVAL (60 * 60 * 1000)
#define TEMPERATURE_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define TEMPERATURE_PUB_VALUE_CHANGE 0.2f
#define TEMPERATURE_UPDATE_SERVICE_INTERVAL (5 * 1000)
#define TEMPERATURE_UPDATE_NORMAL_INTERVAL (1 * 60 * 1000)

#define FLOOD_DETECTOR_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define FLOOD_DETECTOR_UPDATE_SERVICE_INTERVAL (1 * 1000)
#define FLOOD_DETECTOR_UPDATE_NORMAL_INTERVAL  (5 * 1000)

#define TEMPERATURE_DS18B20_PUB_NO_CHANGE_INTEVAL (5  * 60* 1000)
#define TEMPERATURE_DS18B20_PUB_VALUE_CHANGE 0.5f

#define RADIO_FLOOD_DETECTOR        0x0d

// LED instance
bc_led_t led;

// Button instance
bc_button_t button;

// Thermometer instance
bc_tmp112_t tmp112;
static event_param_t temperature_event_param = { .next_pub = 0 };

// Flood detector instance
bc_flood_detector_t flood_detector;
event_param_t flood_detector_event_param = { .next_pub = 0 };

static bc_ds18b20_t ds18b20;

struct {
    event_param_t temperature;
    event_param_t temperature_ds18b20;
    event_param_t humidity;
    event_param_t illuminance;
    event_param_t pressure;

} params;

void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    (void) self;
    (void) event_param;

    if (event == BC_BUTTON_EVENT_PRESS)
    {
        bc_led_pulse(&led, 100);
    }
}

void battery_event_handler(bc_module_battery_event_t event, void *event_param)
{
    (void) event_param;

    float voltage;

    if (event == BC_MODULE_BATTERY_EVENT_UPDATE)
    {
        if (bc_module_battery_get_voltage(&voltage))
        {
            bc_radio_pub_battery(&voltage);
        }
    }
}

void tmp112_event_handler(bc_tmp112_t *self, bc_tmp112_event_t event, void *event_param)
{
    float value;
    event_param_t *param = (event_param_t *)event_param;

    if (event == BC_TMP112_EVENT_UPDATE)
    {
        if (bc_tmp112_get_temperature_celsius(self, &value))
        {
            if ((fabsf(value - param->value) >= TEMPERATURE_PUB_VALUE_CHANGE) || (param->next_pub < bc_scheduler_get_spin_tick()))
            {
                bc_radio_pub_temperature(param->channel, &value);

                param->value = value;
                param->next_pub = bc_scheduler_get_spin_tick() + TEMPERATURE_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
}

void flood_detector_event_handler(bc_flood_detector_t *self, bc_flood_detector_event_t event, void *event_param)
{
    bool is_alarm;
    event_param_t *param = (event_param_t *)event_param;


    if (event == BC_FLOOD_DETECTOR_EVENT_UPDATE)
    {
       is_alarm = bc_flood_detector_is_alarm(self);

       if ((is_alarm != param->value) || (param->next_pub < bc_scheduler_get_spin_tick()))
       {
           bc_radio_pub_bool("flood-detector/a/alarm", &is_alarm);

           param->value = is_alarm;
           param->next_pub = bc_scheduler_get_spin_tick() + FLOOD_DETECTOR_NO_CHANGE_INTEVAL;
       }
    }
}

void ds18b20_event_handler(bc_ds18b20_t *self, uint64_t device_address, bc_ds18b20_event_t e, void *p)
{
    (void) p;

    float value = NAN;

    if (e == bc_ds18b20_EVENT_UPDATE)
    {
        bc_ds18b20_get_temperature_celsius(self, device_address, &value);

        //bc_log_debug("UPDATE %" PRIx64 "(%d) = %f", device_address, device_index, value);

        if ((fabs(value - params.temperature_ds18b20.value) >= TEMPERATURE_DS18B20_PUB_VALUE_CHANGE) || (params.temperature_ds18b20.next_pub < bc_scheduler_get_spin_tick()))
        {
            static char topic[64];
            snprintf(topic, sizeof(topic), "ext-thermometer/%" PRIx64 "/temperature", device_address);
            bc_radio_pub_float(topic, &value);
            params.temperature_ds18b20.value = value;
            params.temperature_ds18b20.next_pub = bc_scheduler_get_spin_tick() + TEMPERATURE_DS18B20_PUB_NO_CHANGE_INTEVAL;
            bc_scheduler_plan_from_now(0, 300);
        }
    }
}

void switch_to_normal_mode_task(void *param)
{
    bc_tmp112_set_update_interval(&tmp112, TEMPERATURE_UPDATE_NORMAL_INTERVAL);

    bc_flood_detector_set_update_interval(&flood_detector, FLOOD_DETECTOR_UPDATE_NORMAL_INTERVAL);

    bc_ds18b20_set_update_interval(&ds18b20, TEMPERATURE_UPDATE_NORMAL_INTERVAL);

    bc_scheduler_unregister(bc_scheduler_get_current_task_id());
}

void application_init(void)
{
    // Initialize LED
    bc_led_init(&led, BC_GPIO_LED, false, false);
    bc_led_set_mode(&led, BC_LED_MODE_OFF);

    bc_radio_init(BC_RADIO_MODE_NODE_SLEEPING);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_scan_interval(&button, 20);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize battery
    bc_module_battery_init();
    bc_module_battery_set_event_handler(battery_event_handler, NULL);
    bc_module_battery_set_update_interval(BATTERY_UPDATE_INTERVAL);

    // Initialize thermometer sensor on core module
    temperature_event_param.channel = BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE;
    bc_tmp112_init(&tmp112, BC_I2C_I2C0, 0x49);
    bc_tmp112_set_event_handler(&tmp112, tmp112_event_handler, &temperature_event_param);
    bc_tmp112_set_update_interval(&tmp112, TEMPERATURE_UPDATE_SERVICE_INTERVAL);

    // Initialize flood detector
    bc_flood_detector_init(&flood_detector, BC_FLOOD_DETECTOR_TYPE_LD_81_SENSOR_MODULE_CHANNEL_A);
    bc_flood_detector_set_event_handler(&flood_detector, flood_detector_event_handler, &flood_detector_event_param);
    bc_flood_detector_set_update_interval(&flood_detector, FLOOD_DETECTOR_UPDATE_SERVICE_INTERVAL);

    // For single sensor you can call bc_ds18b20_init()
    bc_ds18b20_init(&ds18b20, BC_DS18B20_RESOLUTION_BITS_12);
    bc_ds18b20_set_event_handler(&ds18b20, ds18b20_event_handler, NULL);
    bc_ds18b20_set_update_interval(&ds18b20, TEMPERATURE_UPDATE_SERVICE_INTERVAL);

    bc_radio_pairing_request("zdeny-flood-detector-with-ext-temp", VERSION);

    bc_scheduler_register(switch_to_normal_mode_task, NULL, SERVICE_INTERVAL_INTERVAL);

    bc_led_pulse(&led, 2000);
}
