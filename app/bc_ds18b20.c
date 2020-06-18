#include <bc_ds18b20.h>
#include <bc_onewire.h>

#include <bc_gpio.h>
#include <bc_i2c.h>

#define _BC_DS18B20_SCRATCHPAD_SIZE 9
#define _BC_DS18B20_1WIRE_PIN BC_GPIO_P5

static bc_tick_t _bc_ds18b20_lut_delay[] = {
    [BC_DS18B20_RESOLUTION_BITS_9] = 100,
    [BC_DS18B20_RESOLUTION_BITS_10] = 190,
    [BC_DS18B20_RESOLUTION_BITS_11] = 380,
    [BC_DS18B20_RESOLUTION_BITS_12] = 760
};

static void _bc_ds18b20_task_interval(void *param);

static void _bc_ds18b20_task_measure(void *param);

static int _bc_ds18b20_power_semaphore = 0;

void bc_ds18b20_init(bc_ds18b20_t *self, bc_ds18b20_resolution_bits_t resolution)
{
    static bc_ds18b20_sensor_t sensor;
    bc_ds18b20_init_multiple(self, &sensor, 1, resolution);
}

void bc_ds18b20_init_multiple(bc_ds18b20_t *self, bc_ds18b20_sensor_t *sensors, int sensor_count, bc_ds18b20_resolution_bits_t resolution)
{
    memset(self, 0, sizeof(*self));

    bc_onewire_init(_BC_DS18B20_1WIRE_PIN);

    self->_resolution = resolution;
    self->sensor = sensors;
    self->sensor_count = sensor_count;

    self->_task_id_interval = bc_scheduler_register(_bc_ds18b20_task_interval, self, BC_TICK_INFINITY);
    self->_task_id_measure = bc_scheduler_register(_bc_ds18b20_task_measure, self, 10);
}

void bc_ds18b20_set_event_handler(bc_ds18b20_t *self,
        void (*event_handler)(bc_ds18b20_t *, uint64_t _device_address, bc_ds18b20_event_t, void *), void *event_param)
{
    self->_event_handler = event_handler;
    self->_event_param = event_param;
}

void bc_ds18b20_set_update_interval(bc_ds18b20_t *self, bc_tick_t interval)
{
    self->_update_interval = interval;

    if (self->_update_interval == BC_TICK_INFINITY)
    {
        bc_scheduler_plan_absolute(self->_task_id_interval, BC_TICK_INFINITY);
    }
    else
    {
        bc_scheduler_plan_relative(self->_task_id_interval, self->_update_interval);

        bc_ds18b20_measure(self);
    }
}

bool bc_ds18b20_measure(bc_ds18b20_t *self)
{
    if (self->_measurement_active)
    {
        return false;
    }

    self->_measurement_active = true;

    bc_scheduler_plan_now(self->_task_id_measure);

    return true;
}

int bc_ds18b20_get_index_by_device_address(bc_ds18b20_t *self, uint64_t device_address)
{
    for (int i = 0; i < self->sensor_found; i++)
    {
        if (self->sensor[i]._device_address == device_address)
        {
            return i;
        }
    }

    return -1;
}

bool bc_ds18b20_get_temperature_raw(bc_ds18b20_t *self, uint64_t device_address, int16_t *raw)
{
    int sensor_index = bc_ds18b20_get_index_by_device_address(self, device_address);

    if (sensor_index == -1)
    {
        return false;
    }

	if (!self->sensor[sensor_index]._temperature_valid)
	{
		return false;
	}

    *raw = self->sensor[sensor_index]._temperature_raw;

	return true;
}

bool bc_ds18b20_get_temperature_celsius(bc_ds18b20_t *self, uint64_t device_address, float *celsius)
{
    int sensor_index = bc_ds18b20_get_index_by_device_address(self, device_address);

    if (sensor_index == -1)
    {
        return false;
    }

    if (!self->sensor[sensor_index]._temperature_valid)
    {
        return false;
    }

    if ((device_address & 0xFF) == 0x10)
    {
        // If sensor is DS18S20 with family code 0x10, divide by 2 because it is only 9bit sensor
        *celsius = (float) self->sensor[sensor_index]._temperature_raw / 2;    
    }
    else
    {
        *celsius = (float) self->sensor[sensor_index]._temperature_raw / 16;    
    }
    
    return true;
}

static void _bc_ds18b20_task_interval(void *param)
{
    bc_ds18b20_t *self = param;

    bc_ds18b20_measure(self);

    bc_scheduler_plan_current_relative(self->_update_interval);
}

static bool _bc_ds18b20_power_up(bc_ds18b20_t *self)
{
    if (self->_power)
    {
        return true;
    }

    if (_bc_ds18b20_power_semaphore == 0)
    {

        bc_module_sensor_init();

        if (bc_module_sensor_get_revision() == BC_MODULE_SENSOR_REVISION_R1_1)
        {
            bc_module_sensor_set_vdd(1);
        }
        else
        {
            bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_A, BC_MODULE_SENSOR_PULL_UP_56R);
        }

        if (!bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_PULL_UP_4K7))
        {
            return false;
        }
    }

    _bc_ds18b20_power_semaphore++;

    self->_power = true;

    return true;
}

static bool _bc_ds18b20_power_down(bc_ds18b20_t *self)
{
    if (!self->_power)
    {
        return true;
    }

    _bc_ds18b20_power_semaphore--;

    if (_bc_ds18b20_power_semaphore == 0)
    {
        bc_module_sensor_init();

        if (bc_module_sensor_get_revision() == BC_MODULE_SENSOR_REVISION_R1_1)
        {
            bc_module_sensor_set_vdd(0);
        }
        else
        {
            bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_A, BC_MODULE_SENSOR_PULL_NONE);
        }

        if (!bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_PULL_NONE))
        {
            return false;
        }
    }

    self->_power = false;

    return true;
}

static bool _bc_ds18b20_is_scratchpad_valid(uint8_t *scratchpad)
{
    if (scratchpad[5] != 0xff)
    {
        return false;
    }

    if (scratchpad[7] != 0x10)
    {
        return false;
    }

    if (bc_onewire_crc8(scratchpad, _BC_DS18B20_SCRATCHPAD_SIZE, 0) != 0)
    {
        return false;
    }

    return true;
}

static void _bc_ds18b20_task_measure(void *param)
{
    bc_ds18b20_t *self = param;

    start:

    switch (self->_state)
    {
        case BC_DS18B20_STATE_ERROR:
        {
            for (int i = 0; i < self->sensor_found; i++)
            {
                self->sensor[i]._temperature_valid = false;
            }

            self->_measurement_active = false;

            _bc_ds18b20_power_down(self);

            if (self->_event_handler != NULL)
            {
                self->_event_handler(self, 0, bc_ds18b20_EVENT_ERROR, self->_event_param);
            }

            self->_state = BC_DS18B20_STATE_PREINITIALIZE;

            return;
        }
        case BC_DS18B20_STATE_PREINITIALIZE:
        {
            self->_state = BC_DS18B20_STATE_ERROR;

            if (!bc_module_sensor_init())
            {
                goto start;
            }

            bc_module_sensor_set_mode(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_MODE_INPUT);

            if (!_bc_ds18b20_power_up(self))
            {
                goto start;
            }

            self->_state = BC_DS18B20_STATE_INITIALIZE;

            bc_scheduler_plan_current_from_now(10);

            break;
        }
        case BC_DS18B20_STATE_INITIALIZE:
        {
            self->_state = BC_DS18B20_STATE_ERROR;
           
            uint64_t _device_address = 0;
            self->sensor_found = 0;

            bc_onewire_search_start(0);
            while ((self->sensor_found < self->sensor_count) && bc_onewire_search_next(_BC_DS18B20_1WIRE_PIN, &_device_address))
            {
                self->sensor[self->sensor_found]._device_address = _device_address;

                _device_address++;
                self->sensor_found++;
            }

            if (self->sensor_found == 0)
            {
                goto start;
            }

            bc_onewire_transaction_start(_BC_DS18B20_1WIRE_PIN);

            // Write Scratchpad
            if (!bc_onewire_reset(_BC_DS18B20_1WIRE_PIN))
            {
                bc_onewire_transaction_stop(_BC_DS18B20_1WIRE_PIN);

                goto start;
            }

            bc_onewire_skip_rom(_BC_DS18B20_1WIRE_PIN);

            uint8_t buffer[] = {0x4e, 0x75, 0x70, self->_resolution << 5 | 0x1f};

            bc_onewire_write(_BC_DS18B20_1WIRE_PIN, buffer, sizeof(buffer));

            bc_onewire_transaction_stop(_BC_DS18B20_1WIRE_PIN);

            if (self->_measurement_active)
            {
                bc_scheduler_plan_current_now();
            }
            else
            {
                if (!_bc_ds18b20_power_down(self))
                {
                    goto start;
                }
            }

            self->_state = BC_DS18B20_STATE_READY;

            return;
        }
        case BC_DS18B20_STATE_READY:
        {
            self->_state = BC_DS18B20_STATE_ERROR;

            if (!_bc_ds18b20_power_up(self))
            {
                goto start;
            }

            self->_state = BC_DS18B20_STATE_MEASURE;

            bc_scheduler_plan_current_from_now(10);

            return;
        }
        case BC_DS18B20_STATE_MEASURE:
        {
            self->_state = BC_DS18B20_STATE_ERROR;

            bc_onewire_transaction_start(_BC_DS18B20_1WIRE_PIN);

            if (!bc_onewire_reset(_BC_DS18B20_1WIRE_PIN))
            {
            	bc_onewire_transaction_stop(_BC_DS18B20_1WIRE_PIN);
                goto start;
            }

            //bc_onewire_select(_BC_DS18B20_1WIRE_PIN, &self->_device_address);
            bc_onewire_skip_rom(_BC_DS18B20_1WIRE_PIN);

            bc_onewire_write_8b(_BC_DS18B20_1WIRE_PIN, 0x44);

            bc_onewire_transaction_stop(_BC_DS18B20_1WIRE_PIN);

            self->_state = BC_DS18B20_STATE_READ;

            bc_scheduler_plan_current_from_now(_bc_ds18b20_lut_delay[self->_resolution]);

            return;
        }
        case BC_DS18B20_STATE_READ:
        {
            self->_state = BC_DS18B20_STATE_ERROR;

            uint8_t scratchpad[_BC_DS18B20_SCRATCHPAD_SIZE];

            for (int i = 0; i < self->sensor_found; i++)
            {
                bc_onewire_transaction_start(_BC_DS18B20_1WIRE_PIN);

                if (!bc_onewire_reset(_BC_DS18B20_1WIRE_PIN))
                {
                    bc_onewire_transaction_stop(_BC_DS18B20_1WIRE_PIN);

                    goto start;
                }

                bc_onewire_select(_BC_DS18B20_1WIRE_PIN, &self->sensor[i]._device_address);

                bc_onewire_write_8b(_BC_DS18B20_1WIRE_PIN, 0xBE);

                bc_onewire_read(_BC_DS18B20_1WIRE_PIN, scratchpad, sizeof(scratchpad));

                bc_onewire_transaction_stop(_BC_DS18B20_1WIRE_PIN);
              

                if (!_bc_ds18b20_is_scratchpad_valid(scratchpad))
                {
                    goto start;
                }

                self->sensor[i]._temperature_raw = ((int16_t) scratchpad[1]) << 8 | ((int16_t) scratchpad[0]);
                self->sensor[i]._temperature_valid = true;
            }

            if (!_bc_ds18b20_power_down(self))
            {
                goto start;
            }

            self->_state = BC_DS18B20_STATE_UPDATE;

            goto start;
        }
        case BC_DS18B20_STATE_UPDATE:
        {
            self->_measurement_active = false;

            for (int i = 0; i < self->sensor_found; i++)
            {

                if (self->_event_handler != NULL)
                {
                    self->_event_handler(self, self->sensor[i]._device_address, bc_ds18b20_EVENT_UPDATE, self->_event_param);
                }
            }

            self->_state = BC_DS18B20_STATE_READY;

            return;
        }
        default:
        {
            self->_state = BC_DS18B20_STATE_ERROR;

            goto start;
        }
    }
}

