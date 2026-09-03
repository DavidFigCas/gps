#include <stdio.h>

#include "common/mbuf.h"
#include "common/platform.h"
#include "mgos_app.h"
#include "mgos_gpio.h"
#include "mgos_timers.h"
#include "mgos_uart.h"
#include "common/json_utils.h"

#include "mgos.h"
#include "gps.h"
#include "minmea.h"
#include "mgos_sys_config.h"

static int gps_uart_no = 0;
static size_t gpsDataAvailable = 0;
static struct minmea_sentence_rmc lastFrame;
// static char *gps_data;
float last_lat;
float last_lon;
float last_speed;
static bool s_has_fix = false;
static int sat = 0;
static int qt = 0;
static double last_update_time = 0;
// false hasta el primer guardado con fix real (lat/lon validos, no
// NaN/>=999): permite que ESE guardado salte el throttle de
// gps.update_interval en vez de esperar a que se cumpla el intervalo desde
// el arranque (last_update_time nace en 0, asi que sin esto el primer fix
// tarda hasta gps.update_interval segundos en propagarse aunque el GPS lo
// haya conseguido casi de inmediato). Una vez guardado el primer fix real,
// las actualizaciones siguientes vuelven a respetar el intervalo normal.
static bool s_first_fix_saved = false;

bool mgos_gps_get_location(float *lat, float *lon, float *speed)
{
    if (lat != NULL) *lat = last_lat;
    if (lon != NULL) *lon = last_lon;
    if (speed != NULL) *speed = last_speed;
    return s_has_fix;
}

void mgos_save_location()
{

    float lat = minmea_tocoord(&lastFrame.latitude);
    float lon = minmea_tocoord(&lastFrame.longitude);
    float speed = minmea_tocoord(&lastFrame.speed);

    if ((isnan(lat)) || (lat >= 999))
    {
        // lat = 0.0f;
        lat = mgos_sys_config_get_device_location_lat();
    }

    if ((isnan(lon)) || (lon >= 999))
    {
        // lon = 0.0f;
        lon = mgos_sys_config_get_device_location_lon();
    }

    if (isnan(speed))
    {
        speed = 0.0f;
    }

    // if (lat != 0 && lon != 0)
    //{
    mgos_sys_config_set_device_location_lat(lat);
    mgos_sys_config_set_device_location_lon(lon);
    //}

    LOG(LL_INFO, ("GPS coordinates: Latitude = %f, Longitude = %f", lat, lon));

    // snprintf(gps_data, 80, "{lat: \"%f\", lon: \"%f\", sp: \"%f\", sat: \"%d\" , qt: \"%d\"}", lat, lon, speed, sat, qt);

    // return gps_data;
}

static void parseGpsData(char *line)
{
    char lineNmea[MINMEA_MAX_LENGTH];
    strncpy(lineNmea, line, sizeof(lineNmea) - 1);
    strcat(lineNmea, "\n");
    lineNmea[sizeof(lineNmea) - 1] = '\0';

    enum minmea_sentence_id id = minmea_sentence_id(lineNmea, false);
    // printf("sentence id = %d from line %s\n", (int) id, lineNmea);
    switch (id)
    {
    case MINMEA_SENTENCE_RMC:
    {
        struct minmea_sentence_rmc frame;
        if (minmea_parse_rmc(&frame, lineNmea))
        {
            lastFrame = frame;

            float lat = minmea_tocoord(&lastFrame.latitude);
            float lon = minmea_tocoord(&lastFrame.longitude);
            bool has_fix = !isnan(lat) && !isnan(lon) && lat < 999 && lon < 999;

            /* Actualiza la última posición conocida solo con fixes válidos */
            if (frame.valid && !isnan(lat) && !isnan(lon) && lat < 999 && lon < 999)
            {
                last_lat = lat;
                last_lon = lon;
                float sp = minmea_tofloat(&lastFrame.speed);
                last_speed = isnan(sp) ? 0.0f : sp;
                s_has_fix = true;
            }

            // Verifica el intervalo de actualización, salvo que este sea el
            // primer fix real: ese se guarda de inmediato (ver
            // s_first_fix_saved arriba) para no esperar hasta
            // gps.update_interval segundos de uptime solo porque
            // last_update_time arranca en 0.
            double current_time = mgos_uptime(); // Obtén el tiempo en segundos desde el inicio
            bool interval_due = (current_time - last_update_time) >=
                                 mgos_sys_config_get_gps_update_interval();
            bool first_fix = has_fix && !s_first_fix_saved;
            if (interval_due || first_fix)
            {
                last_update_time = current_time;
                if (has_fix)
                {
                    s_first_fix_saved = true;
                }

                // Llama a la función para obtener la ubicación
                mgos_save_location();
            }

            // Guardar la configuración en un archivo para que sea permanente
            // if (!mgos_sys_config_save(&mgos_sys_config, false /* no reconfigura la red */, NULL /* no callback */))
            //{
            //    LOG(LL_ERROR, ("Failed to save configuration"));
            //}
            // else
            //{
            //    pulsos_litro = mgos_sys_config_get_app_pulsos_litro();
            //    LOG(LL_INFO, ("Configuration saved: pulsos_litro = %.2f", pulsos_litro));
            //}

            /*
      printf("$RMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
             frame.latitude.value, frame.latitude.scale,
             frame.longitude.value, frame.longitude.scale,
             frame.speed.value, frame.speed.scale);
      printf("$RMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
             minmea_rescale(&frame.latitude, 1000),
             minmea_rescale(&frame.longitude, 1000),
             minmea_rescale(&frame.speed, 1000));
      printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\n",
             minmea_tocoord(&frame.latitude),
             minmea_tocoord(&frame.longitude),
             minmea_tofloat(&frame.speed));
      */
        }
    }
    break;

    case MINMEA_SENTENCE_GGA:
    {
        struct minmea_sentence_gga frame;
        if (minmea_parse_gga(&frame, lineNmea))
        {
            // printf("$GGA: fix quality: %d\n", frame.fix_quality);
            qt = frame.fix_quality;
        }
    }
    break;

    case MINMEA_SENTENCE_GSV:
    {
        struct minmea_sentence_gsv frame;
        if (minmea_parse_gsv(&frame, lineNmea))
        {
            sat = frame.total_sats;
            // printf("$GSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);

            // printf("$GSV: sattelites in view: %d\n", frame.total_sats);

            /*for (int i = 0; i < 4; i++)
        printf("$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
               frame.sats[i].nr,
               frame.sats[i].elevation,
               frame.sats[i].azimuth,
               frame.sats[i].snr);
      */
        }
    }
    break;
    case MINMEA_INVALID:
    {
        break;
    }
    case MINMEA_UNKNOWN:
    {
        break;
    }
    case MINMEA_SENTENCE_GSA:
    {
        break;
    }
    case MINMEA_SENTENCE_GLL:
    {
        break;
    }
    case MINMEA_SENTENCE_GST:
    {
        break;
    }
    case MINMEA_SENTENCE_VTG:
    {
        break;
    }
    case MINMEA_SENTENCE_ZDA:
    {
        break;
    }
    }
}

static void gps_read_cb(void *arg)
{

    // printf("Hello, GPS!\r\n");
    if (gpsDataAvailable > 0)
    {
        struct mbuf rxb;
        mbuf_init(&rxb, 0);
        mgos_uart_read_mbuf(gps_uart_no, &rxb, gpsDataAvailable);
        if (rxb.len > 0)
        {
            char *pch;
            // printf("%.*s", (int) rxb.len, rxb.buf);
            pch = strtok(rxb.buf, "\n");
            while (pch != NULL)
            {
                // printf("GPS lineNmea: %s\n", pch);
                parseGpsData(pch);
                pch = strtok(NULL, "\n");
            }
        }
        mbuf_free(&rxb);

        gpsDataAvailable = 0;
    }

    (void)arg;
}

int esp32_uart_rx_fifo_len(int uart_no);

static void uart_dispatcher(int uart_no, void *arg)
{
    assert(uart_no == gps_uart_no);
    size_t rx_av = mgos_uart_read_avail(uart_no);
    if (rx_av > 0)
    {
        gpsDataAvailable = rx_av;
    }
    (void)arg;
}

bool mgos_gps_init(void)
{
    if (!mgos_sys_config_get_gps_enable())
        return true;

    struct mgos_uart_config ucfg;
    gps_uart_no = mgos_sys_config_get_gps_uart_no();
    mgos_uart_config_set_defaults(gps_uart_no, &ucfg);

    ucfg.baud_rate = mgos_sys_config_get_gps_baud_rate();
    ucfg.num_data_bits = 8;
    // ucfg.dev.tx_gpio = 17;
    ucfg.dev.rx_gpio = mgos_sys_config_get_gps_rx_gpio();
    ucfg.dev.tx_gpio = mgos_sys_config_get_gps_tx_gpio();
    // ucfg.parity = MGOS_UART_PARITY_NONE;
    // ucfg.stop_bits = MGOS_UART_STOP_BITS_1;
    if (!mgos_uart_configure(gps_uart_no, &ucfg))
    {
        return false;
    }

    mgos_set_timer(mgos_sys_config_get_gps_update_interval_uart() /* ms */, true /* repeat */, gps_read_cb, NULL /* arg */);

    mgos_uart_set_dispatcher(gps_uart_no, uart_dispatcher, NULL /* arg */);
    mgos_uart_set_rx_enabled(gps_uart_no, true);

    // gps_data = calloc(1, 64);

    return true;
}
