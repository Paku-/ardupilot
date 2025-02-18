#include "AP_Frsky_D.h"

#if AP_FRSKY_D_TELEM_ENABLED

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>


/*
  send 1 byte and do byte stuffing
*/
void AP_Frsky_D::send_byte(uint8_t byte)
{
    if (byte == START_STOP_D) {
        _port->write(0x5D);
        _port->write(0x3E);
    } else if (byte == BYTESTUFF_D) {
        _port->write(0x5D);
        _port->write(0x3D);
    } else {
        _port->write(byte);
    }
}

/*
 * send one uint16 frame of FrSky data - for FrSky D protocol (D-receivers)
 */
void AP_Frsky_D::send_uint16(uint16_t id, uint16_t data)
{
    _port->write(START_STOP_D);    // send a 0x5E start byte
    uint8_t *bytes = (uint8_t*)&id;
    send_byte(bytes[0]);
    bytes = (uint8_t*)&data;
    send_byte(bytes[0]); // LSB
    send_byte(bytes[1]); // MSB
}

/*
 * send frame1/2/3 telemetry data
 * for FrSky D protocol (D-receivers)
 */
void AP_Frsky_D::send(void)
{
    const AP_BattMonitor &_battery = AP::battery();
    uint32_t now = AP_HAL::millis();
    
    // send fast FRAME1
    if (now - _D.last_fast_frame >= FAST_FRAME_INTERVAL) {
        _D.last_fast_frame = now;

        // // ACC data 
        // send_uint16(DATA_ID_ACC_X, (uint16_t)(0x0001)); 
        // send_uint16(DATA_ID_ACC_Y, (uint16_t)(0x0002)); 
        // send_uint16(DATA_ID_ACC_Z, (uint16_t)(0x0003)); 

        calc_nav_alt();
        send_uint16(DATA_ID_BARO_ALT_BP, _SPort_data.alt_nav_meters);       // send nav altitude integer part
        send_uint16(DATA_ID_BARO_ALT_AP, _SPort_data.alt_nav_cm);           // send nav altitude decimal part

        send_uint16(DATA_ID_TEMP1, gcs().custom_mode());                    // send flight mode
        send_uint16(DATA_ID_TEMP2, (uint16_t)(AP::gps().num_sats() * 10 + AP::gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)

        //send per cell voltages
        uint8_t cells_count = 3;
        for (uint8_t i = 0; i < cells_count; i++) {
            uint16_t cell_id_volts = 0x1000*(i)+(_battery.voltage()/cells_count/4.2*0x0834); //magic number 0x0834 = 2100 
            cell_id_volts = (cell_id_volts >> 8) | (cell_id_volts << 8);    //swap bytes
            send_uint16(DATA_ID_VOLTS, (uint16_t)cell_id_volts);            // send hours & mins  
        }

        //send voltage/current sensor
        // float current;
        // if (!_battery.current_amps(current)) {
        //     current = 0;
        // }
        // send_uint16(DATA_ID_CURRENT, (uint16_t)roundf(current * 10.0f)); // send current consumption
        
        send_uint16(DATA_ID_CURRENT, (uint16_t)(10) );      
        send_uint16(DATA_ID_VOLTAGE_BP, (uint16_t)(5));   // 
        send_uint16(DATA_ID_VOLTAGE_AP, (uint16_t)(4));   // Has to be less 10 or will be added to BP
        
        //end FRAME1 data
        
    }
    // send mid FRAME2
    if (now - _D.last_mid_frame >= MID_FRAME_INTERVAL) {
        _D.last_mid_frame = now;

        AP_AHRS &_ahrs = AP::ahrs();        
        send_uint16(DATA_ID_GPS_COURS_BP, (uint16_t)((_ahrs.yaw_sensor / 100) % 360));  // send heading in degree based on AHRS and not GPS
        send_uint16(DATA_ID_GPS_COURS_AP, (uint16_t)(0));                              // .0 
        
        if (AP::gps().status() >= 3) {
            calc_gps_position();        
        
            send_uint16(DATA_ID_GPS_LAT_BP, _SPort_data.latdddmm); // send gps latitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LAT_AP, _SPort_data.latmmmm); // send gps latitude minutes decimal part
            send_uint16(DATA_ID_GPS_LAT_NS, _SPort_data.lat_ns); // send gps North / South information
            send_uint16(DATA_ID_GPS_LONG_BP, _SPort_data.londddmm); // send gps longitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LONG_AP, _SPort_data.lonmmmm); // send gps longitude minutes decimal part
            send_uint16(DATA_ID_GPS_LONG_EW, _SPort_data.lon_ew); // send gps East / West information
            send_uint16(DATA_ID_GPS_SPEED_BP, _SPort_data.speed_in_meter); // send gps speed integer part
            send_uint16(DATA_ID_GPS_SPEED_AP, _SPort_data.speed_in_centimeter); // send gps speed decimal part
            send_uint16(DATA_ID_GPS_ALT_BP, _SPort_data.alt_gps_meters); // send gps altitude integer part
            send_uint16(DATA_ID_GPS_ALT_AP, _SPort_data.alt_gps_cm); // send gps altitude decimal part
        }

        uint8_t percentage = 0;
        IGNORE_RETURN(_battery.capacity_remaining_pct(percentage));
        uint8_t battery_fuel = ((percentage / 25) +1) * 25 ; // 0-25, 26-50, 51-75, 76-100
        if (battery_fuel > 100) {
            battery_fuel = 100;
        }   
        send_uint16(DATA_ID_FUEL, (uint16_t) battery_fuel); // send battery remaining
        // send_uint16(DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage

    }
  
  
  
    // send slow FRAME3

    // const AP_Vehicle *vehicle = AP::vehicle();   
    // if ((vehicle != nullptr) && vehicle->get_time_flying_ms() && (now - _D.last_5000ms_frame >= 5000)) {
    if ((now - _D.last_slow_frame >= SLOW_FRAME_INTERVAL)) {

        _D.last_slow_frame = now;

        // send date - not used
        // send_uint16(DATA_ID_DAY_MONTH, (uint16_t)(0x0101));             // 01.01        
        // send_uint16(DATA_ID_YEAR_0, (uint16_t)(0x01));                  // 2001

        
        // uint32_t time_flying_s = vehicle->get_time_flying_ms()/1000;                
        // uint32_t time_flying_s = 4000;                
        uint32_t time_flying_s = now/1000;                              
        
        uint8_t time_flying_h = (uint8_t)time_flying_s/3600;            // time since start in hours
        uint8_t time_flying_m = (uint8_t)(time_flying_s % 3600)/60;     // minutes        
        time_flying_s = (uint8_t)time_flying_s % 60;                    // seconds

       
        //  send time
        send_uint16(DATA_ID_HOURS_MINUTE, (uint16_t)((time_flying_m << 8) | time_flying_h));    // send hours & mins       
        send_uint16(DATA_ID_SECONDS_0, (uint16_t)(time_flying_s));                              // seconds

       
        send_uint16(DATA_ID_RPM, (uint16_t)(1000));              
        
    }


        


}

#endif  // AP_FRSKY_D_TELEM_ENABLED
