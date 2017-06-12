/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/HeartRateService.h"
#include "MAX30102.h"
#include "MAX30205.h"
#include "algorithm.h"
#include "filterdata.h"
#include "Timer.h"
#include "MAX17055.h"
#include "MAX14690.h"

// Define hardware objects here

// Uncomment this if not working with virtual serial port (more stable if this is commented OUT)
// #define DEBUG       1

// DFU.  Must build with mbed-cli on the DFU branch (feature-nrf5_dfu_s13x_v2)
// #define USE_DFU     1

// Power Management in the system
#define HAS_PM      1

// Power guage
// #define HAS_GAUGE   1   

#if USE_DFU
#include "ble/services/DFUService.h"
#endif

#if DEBUG
Serial pc(USBTX, USBRX);
#endif

DigitalOut  led3(LED3);

InterruptIn INT(P0_20);     // SpO2 interrupt

MAX30205 temp_sensor(P0_17, P0_16, 0x90);

#if HAS_GAUGE
MAX17055    gauge(P0_17, P0_16);
#endif

#if HAS_PM
MAX14690 pm(P0_17, P0_16);
#endif

static Timer    timer;

const static char       DEVICE_NAME[] = "ESBSW1";
static const uint16_t   uuid16_list[] = {GattService::UUID_HEART_RATE_SERVICE };
static HeartRateService *hrServicePtr = NULL;

static float    currentTemperature = 0.0;

Thread t;
static EventQueue   eventQueue (/* event count */ 16 * /* event size */ 32);

#define BUFFER_MAX  500
static uint32_t aun_ir_buffer[BUFFER_MAX]; //IR LED sensor data
static uint32_t aun_red_buffer[BUFFER_MAX];    //Red LED sensor data
static int16_t  n_spo2 = 100; //%SpO2 value
static int8_t   ch_spo2_valid = 0;   //indicator to show if the SP02 calculation is valid
static int16_t  n_heart_rate = 50;   //heart rate value
static int8_t   ch_hr_valid = 0;    //indicator to show if the heart rate calculation is valid
static bool connected = false;

FilterData *sp_filter = new FilterData(10, 8, 1.0, 50, 95.0, 5.0, 5, 1.0);
FilterData *hr_filter = new FilterData(10, 8, 1.5, 50, 50.0, 20.0, 5, 2.0);

static void log_str(const char *str)
{
#if DEBUG
    pc.printf(str);
#endif
}

static void log_line(const char *str)
{
#if DEBUG
    pc.printf("%s\r\n", str);
#endif
}

// flag 0: write as is, flag 1: append newline at the end
static void vlog(int flag, const char * format, ... )
{
#ifndef _DEBUG
    return;
#endif

    if (flag < 0 || flag > 1) return;
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer,format, args);
  
    if (flag == 0)
        log_str(buffer);
    else if (flag == 1)
        log_line(buffer);
 
  va_end(args);
}

static void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    connected = false;
    BLE::Instance().gap().startAdvertising(); // Restart advertising
}

static void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{
    connected = true;
}

static void updateHRMSensorValue()
{
    if (!hrServicePtr || !connected) return;

     led3 = 1;

    uint16_t hr = (uint16_t) (hr_filter->GetValue() + 0.5);

    // Make sure it's 12 bits.  Header is 0x0000
    hr = hr & 0xFFF;

    hrServicePtr->updateHeartRate(hr);
}

static void updateSPO2SensorValue()
{
    if (!hrServicePtr || !connected) return;

    uint16_t spo2 = (uint16_t) (sp_filter->GetValue() + 0.5);

    // Header is 0x1000
    spo2 = 0x1000 | (spo2 & 0xFFF);

    hrServicePtr->updateHeartRate(spo2);
}

static void updateTempSensorValue()
{
    if (!hrServicePtr) return;

     led3 = 1;

    // Unit is in 1/10th of Farenhait
    uint16_t temp = (uint16_t) (currentTemperature * 10 + 0.5);

    // Header is 0x2000
    temp = 0x2000 | (temp & 0xFFF);

    hrServicePtr->updateHeartRate(temp);
}

static void onBleInitError(BLE &ble, ble_error_t error)
{
    (void) ble;

    vlog(1, "ble init error number = %d", (int) error);
}

static void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE)
    {
        led3 = !led3;
        onBleInitError(ble, error);
        return;
    }

    if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE)
    {
        vlog(1, "ble not default instance");
        led3 = !led3;
        return;
    }

    ble.gap().onConnection(connectionCallback);
    ble.gap().onDisconnection(disconnectionCallback);

    /* Set up advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_HEART_RATE_SENSOR);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));

    /* Set up primary service */
    hrServicePtr = new HeartRateService(ble, (uint16_t) 50, HeartRateService::LOCATION_OTHER);

    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms */
    ble.gap().startAdvertising();

#if USE_DFU
    static DFUService   dfu(ble);
#endif
}

static void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context)
{
    BLE &ble = BLE::Instance();

    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

static void InitMaxim30102()
{
    bool ret;
    uint8_t uch_dummy;
    
    ret = maxim_max30102_reset();

    if (ret == false)
        vlog(1, "30102 reset failed");

    wait(0.1);
    
    //read and clear status register
    ret = maxim_max30102_read_reg(0,&uch_dummy); 
    wait(0.1);
    
    ret = maxim_max30102_init();
    wait(0.1);
    
    if (ret == false)
        vlog(1, "30102 init failed");

    currentTemperature = readTemp(true);
    vlog(1, "30102 T=%.1fF", currentTemperature);
    updateTempSensorValue();
    
    // Read and clear status register
    maxim_max30102_read_reg(0, &uch_dummy);

    timer.start();

    // Read the first 500 samples, and determine the signal range
    for (int i = 0; i < BUFFER_MAX; ++i)
    {
        while (INT.read() == 1);   //wait until the interrupt pin asserts
        
        maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));  //read from MAX30102 FIFO
        vlog(2, "time=%d, red=%d, ir=%d", timer.read_ms(), aun_red_buffer[i], aun_ir_buffer[i]);
    }
    
    // Calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_MAX, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
    vlog(1, "Raw TS=%d, HR=%d (valid=%d), SP=%d (valid=%d)", timer.read_ms(), n_heart_rate, ch_hr_valid, n_spo2, ch_spo2_valid);
}

static int  buffer_count = 0;
static int  calc_count = 0;

static void UpdateMaxim30102()
{
    if (buffer_count == 0)
    {
      //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for (int i = 100; i < BUFFER_MAX; ++i)
        {
            aun_red_buffer[i - 100] = aun_red_buffer[i];
            aun_ir_buffer[i - 100] = aun_ir_buffer[i];
        }

        buffer_count = 400;
    }

    maxim_max30102_read_fifo((aun_red_buffer + buffer_count), (aun_ir_buffer + buffer_count));  //read from MAX30102 FIFO
    vlog(2, "time=%d, red=%d, ir=%d", timer.read_ms(), aun_red_buffer[buffer_count], aun_ir_buffer[buffer_count]);

    if (++buffer_count == BUFFER_MAX)
    {
        buffer_count = 0;

        // Calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_MAX, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 

         if (ch_hr_valid && n_heart_rate >= 10 && n_heart_rate <= 300)     // Basic assumptions
            hr_filter->AddPoint((double) n_heart_rate);
        else if (!ch_hr_valid)
            n_heart_rate = -1;
        
        if (ch_spo2_valid && n_spo2 >= 50 && n_spo2 <= 100)
            sp_filter->AddPoint((double) n_spo2);
        else if (!ch_spo2_valid)
            n_spo2 = -1;
        
        if (++calc_count == 10000)
            calc_count = 0;

        if (calc_count % 15 == 0)        // Every 30 seconds report the temperature
        {
            // currentTemperature = readTemp(true);

            uint16_t int_temp;

            temp_sensor.readTemperature(&int_temp);
            currentTemperature = temp_sensor.toCelsius(int_temp);
            currentTemperature = temp_sensor.toFahrenheit(currentTemperature);

            updateTempSensorValue();
            vlog(1, "T=%.1fF", currentTemperature);
        }

        // Write out these every 2 seconds
        vlog(1, "Raw TS=%d, HR=%d (valid=%d), SP=%d (valid=%d)", timer.read_ms(), n_heart_rate, ch_hr_valid, n_spo2, ch_spo2_valid);
        vlog(1, "Calculated HR=%d, SP=%d", (int) (hr_filter->GetValue() + 0.5), (int) (sp_filter->GetValue() + 0.5));
        updateHRMSensorValue();
        updateSPO2SensorValue();

#if HAS_GAUGE
        // Print the current state of charge
        vlog(1, "SOC = %f%%", gauge.soc());
#endif
    }
}

void periodicCallback()
{
    UpdateMaxim30102();
}

int main()
{
    t.start(callback(&eventQueue, &EventQueue::dispatch_forever));
    
#if HAS_PM
    pm.ldo3Mode = MAX14690::LDO_ENABLED;
    pm.ldo2Millivolts = 3200;
    pm.ldo3Millivolts = 3100;
    pm.ldo2Mode = MAX14690::LDO_ENABLED;
    pm.monCfg = MAX14690::MON_HI_Z;
#endif

#if HAS_GAUGE
    if (gauge.open())
    {
        vlog(1, "Gauge device detected!");

        // Load the default compensation value
        gauge.compensation(MAX17055::RCOMP0);
    }
#endif
    
#if HAS_PM
    if (pm.init() == MAX14690_ERROR)
    {
        vlog(1, "Error initializing MAX14690");
         led3 = 1;
    }
#endif

    BLE &ble = BLE::Instance();
    
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    InitMaxim30102();
    
    // INT.fall(eventQueue.event(&UpdateMaxim30102));
    // eventQueue.call_every(1, periodicCallback);
    
    while (1)
    {
          while (INT.read() == 1);

          UpdateMaxim30102();
    }

    return 0;
}
