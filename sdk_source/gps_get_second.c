#include "gps_get_second.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>



// ---------- checksum & string utils ----------
static uint8_t nmea_checksum_compute(const char *line){
    uint8_t cs=0; const char *p=line; if(*p=='$') ++p;
    while(*p && *p!='*') cs^=(uint8_t)(*p++);
    return cs;
}
static bool nmea_checksum_ok(const char *line){
    const char *star=strchr(line,'*'); if(!star) return false;
    unsigned rx=0; if(sscanf(star+1,"%2x",&rx)!=1) return false;
    return ((uint8_t)rx)==nmea_checksum_compute(line);
}


/* Parse "hhmmss" or "hhmmss.sss" and return seconds (0–59), or -1 if invalid. */
static int parse_hhmmss_seconds(const char *t)
{
    if (!t || strlen(t) < 6)
        return -1;

    // Ensure it’s digits
    if (!isdigit((unsigned char)t[0]) || !isdigit((unsigned char)t[5]))
        return -1;

    int sec = (t[4] - '0') * 10 + (t[5] - '0');
    if (sec < 0 || sec > 59)
        return -1;

    return sec;
}



// ---------- public entry ------------------------------
bool gps_extract_second(const char *block, uint8_t *sec_out)
{
    if (!block || !sec_out) return false;

    // Ensure the string ends cleanly
    char line[256];
    strncpy(line, block, sizeof(line)-1);
    line[sizeof(line)-1] = '\0';

    // Trim CR/LF in case UART appended them
    size_t n = strlen(line);
    while (n && (line[n-1] == '\r' || line[n-1] == '\n'))
        line[--n] = '\0';

    // Validation: must start with '$' and contain '*'
    if (line[0] != '$' || !strchr(line, '*'))
        return false;

    // Checksum validation and second extraction
    if (!strstr(line, "ZDA,")) return false;
    if (!nmea_checksum_ok(line)) return false;

    char *save = 0;
    char *tok = strtok_r(line, ",*", &save);
    int idx = -1;
    while (tok) {
        idx++;
        if (idx == 1) {
            int s = parse_hhmmss_seconds(tok);
            if (s >= 0) { *sec_out = (uint8_t)s; return true; }
            return false;
        }
        tok = strtok_r(NULL, ",*", &save);
    }
    return false;
}





