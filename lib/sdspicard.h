#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "hardware/adc.h"
#include "hardware/rtc.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"

#define ADC_PIN 26 // GPIO 26

extern bool logger_enabled;
extern const uint32_t period;
extern absolute_time_t next_log_time;
extern char filename[20];

sd_card_t *sd_get_by_name(const char *const name);

void run_setrtc();

void run_format();
void run_mount();
void run_unmount();
void run_getfree();
void run_ls();
void run_cat();


// Função para capturar dados do ADC e salvar no arquivo *.txt
void capture_adc_data_and_save(int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay, int16_t az);

// Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename);
