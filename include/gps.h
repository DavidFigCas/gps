/*
 * High level GPS Functions
 *
 *
 */

#ifndef _mgos_gps_h_
#define _mgos_gps_h_

//extern float last_lat;
//extern float last_lon;
//extern float last_speed;

#include <stdbool.h>

/*
 * Get last location data
 * Returns:
 *  json object with format {lat: \"%f\", lon: \"%f\", sp: \"%f\"}
 */
void mgos_save_location();

/*
 * Última posición conocida del GPS.
 * Cualquier puntero puede ser NULL si no interesa ese dato.
 * Devuelve true si ya se recibió al menos un fix válido (RMC con valid=A);
 * false si aún no hay fix (los valores devueltos serán 0).
 */
bool mgos_gps_get_location(float *lat, float *lon, float *speed);

/**
 * @brief MGOS lib init
 */
bool mgos_gps_init(void);

#endif