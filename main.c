#include "mpu6050.h" 
#include "sdspicard.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "pico/bootrom.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pioconfig.pio.h"

#define botaoB 6
#define FLAG_VALUE 0X22
#define GREEN 11
#define BLUE 12
#define RED 13
#define BUZZER_LEFT_GPIO 10
#define BUZZER_RIGHT_GPIO 21

// define uma referência para função sem argumento nem retorno (via de regra - callback)
typedef void (*p_fn_t)();

//define a estrutura para armazenar os comandos e callbacks
typedef struct
{
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

typedef struct pio_refs
{ // estrutura que representa a PIO
  PIO address;
  int state_machine;
  int offset;
  int pin;
} pio;

typedef struct rgb
{ // estrutura que armazena as cores para a matriz
  double red;
  double green;
  double blue;
} rgb;

typedef struct drawing
{ // estrutura que representa o desenho da matriz
  double figure[25];
  uint8_t index;
  rgb main_color;
  rgb background_color;
} sketch;

//função para colocar a placa em bootsel
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

//resquício de quando fui usar multicore
void core1_main();
static void run_help();
static void process_stdio(int cRxedChar);

static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <YY> <hh> <mm> <ss>: Set Real Time Clock"},
    {"format", run_format, "format [<drive#:>]: Formata o cartão SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cartão SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cartão SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Espaço livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <filename>: Mostra conteúdo do arquivo"},
    {"help", run_help, "help: Mostra comandos disponíveis"}};

//filas para o trânsito de informações do sensor para cada tarefa e periférico
QueueHandle_t xSensorDataQueue;
QueueHandle_t xAccelQueue;

// Configura o PIO, carrega programa e inicializa o state machine
void config_pio(pio *pio)
{
  pio->address = pio0;
  if (!set_sys_clock_khz(128000, false))
    printf("clock errado!");
  pio->offset = pio_add_program(pio->address, &pio_review_program);
  pio->state_machine = pio_claim_unused_sm(pio->address, true);

  pio_review_program_init(pio->address, pio->state_machine, pio->offset, pio->pin);
}

// Converte cor RGB para formato que vou enviar para a matriz
uint32_t rgb_matrix(rgb color)
{
  unsigned char r, g, b;
  r = color.red * 255;
  g = color.green * 255;
  b = color.blue * 255;
  return (g << 24) | (r << 16) | (b << 8);
}

// desenha na matriz os padrões
void draw_new(sketch sketch, uint32_t led_cfg, pio pio, const uint8_t vector_size)
{

  for (int16_t i = 0; i < vector_size; i++)
  {
    if (sketch.figure[i] == 1)
      led_cfg = rgb_matrix(sketch.main_color);
    else
      led_cfg = rgb_matrix(sketch.background_color);
    while (pio_sm_is_tx_fifo_full(pio.address, pio.state_machine)) {
        taskYIELD();  // espera liberar
    }
    pio_sm_put(pio.address, pio.state_machine, led_cfg);
  }
};

// tarefa relativa ao controle da matriz de LEDs
void vMatrixTask()
{

  pio my_pio = {
      .pin = 7,
      .address = 0,
      .offset = 0,
      .state_machine = 0};

  config_pio(&my_pio);

  sketch sketch1 = {
      .background_color = {
          .blue = 0.0, .green = 0.0, .red = 0.0},
      .index = 0,
      .main_color = {.blue = 0.01, .green = 0.05, .red = 0.00},
      .figure = {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0}};

  sketch sketch2 = {
      .background_color = {
          .blue = 0.0, .green = 0.0, .red = 0.0},
      .index = 0,
      .main_color = {.blue = 0.0, .green = 0.00, .red = 0.01},
      .figure = {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1}};

  while (true)
  {
    mpu6050_data data;
    // observa o estado atual e desenha na matriz de acordo
    if (xQueuePeek(xSensorDataQueue, &data, pdMS_TO_TICKS(20)) == pdPASS) {
        if (data.ay > 0){
            sketch1.main_color.blue = 0.05;
            draw_new(sketch1, 0, my_pio, 25);
        } else if (data.ay < 0){
            sketch1.main_color.blue = 0.01;
            draw_new(sketch1, 0, my_pio, 25);
        } else {
            draw_new(sketch2, 0, my_pio, 25);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

//Tarefa que gerencia as operações no cartão sd
void vSDTask()
{
    time_init();
    adc_init();

    printf("FatFS SPI example\n");
    printf("\033[2J\033[H"); // Limpa tela
    printf("\n> ");
    stdio_flush();
    //    printf("A tela foi limpa...\n");
    //    printf("Depois do Flush\n");
    run_help();

    while (true)
    {
        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar)
            process_stdio(cRxedChar);

        if (cRxedChar == 'a') // Monta o SD card se pressionar 'a'
        {
            printf("\nMontando o SD...\n");
            run_mount();
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'b') // Desmonta o SD card se pressionar 'b'
        {
            printf("\nDesmontando o SD. Aguarde...\n");
            run_unmount();
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'c') // Lista diretórios e os arquivos se pressionar 'c'
        {
            printf("\nListagem de arquivos no cartão SD.\n");
            run_ls();
            printf("\nListagem concluída.\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'd') // Exibe o conteúdo do arquivo se pressionar 'd'
        {
            read_file(filename);
            printf("Escolha o comando (h = help):  ");
        }
        if (cRxedChar == 'e') // Obtém o espaço livre no SD card se pressionar 'e'
        {
            printf("\nObtendo espaço livre no SD.\n\n");
            run_getfree();
            printf("\nEspaço livre obtido.\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'f') // Captura dados do ADC e salva no arquivo se pressionar 'f'
        {
            mpu6050_data data;
            for (uint8_t i = 0; i < 20; i++)
            {
                xQueuePeek(xSensorDataQueue, &data, 0);
                capture_adc_data_and_save(data.gx, data.gy, data.gz, data.ax, data.ay, data.az);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'g') // Formata o SD card se pressionar 'g'
        {
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
            run_format();
            printf("\nFormatação concluída.\n\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'h') // Exibe os comandos disponíveis se pressionar 'h'
        {
            run_help();
        }
        vTaskDelay(pdMS_TO_TICKS(200));
        tight_loop_contents();
    }
}

//tarefa que gerencia o led rgb
void vTaskLedFromAccel() {
    mpu_accel accel_data;

    // Inicializa GPIOs dos LEDs
    gpio_init(GREEN);     gpio_set_dir(GREEN, GPIO_OUT);
    gpio_init(BLUE);      gpio_set_dir(BLUE, GPIO_OUT);
    gpio_init(RED);  gpio_set_dir(RED, GPIO_OUT);

    while (true) {
        xQueuePeek(xAccelQueue, &accel_data, 0);
        // Normaliza (dividindo por 16384 para 2G de range)
        float ax = accel_data.x / 16384.0f;
        float ay = accel_data.y / 16384.0f;
        float az = accel_data.z / 16384.0f;

        // Calcula módulo da aceleração
        float total_accel = sqrtf(ax * ax + ay * ay + az * az);

        // Limiares de magnitude (ajustáveis)
        gpio_put(GREEN, 0);
        gpio_put(BLUE, 0);
        gpio_put(RED, 0);

        if (total_accel < 1.2f) {
            gpio_put(GREEN, 1);     // Aceleração baixa
        } else if (total_accel < 2.5f) {
            gpio_put(GREEN, 1);
            gpio_put(RED, 1);      // Aceleração moderada
        } else {
            gpio_put(RED, 1);  // Aceleração forte / impacto
        }

        // Pequeno delay para evitar sobrecarga da CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//tarefa que gerencia o buzzer, toca o buzzer respectivo seguindo a acel y (ax = dado do sensor.y)
void vTaskBuzzerFromAccel() {
    mpu_accel accel_data;

    gpio_set_function(BUZZER_LEFT_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(BUZZER_RIGHT_GPIO, GPIO_FUNC_PWM);

    uint slice_left = pwm_gpio_to_slice_num(BUZZER_LEFT_GPIO);
    uint slice_right = pwm_gpio_to_slice_num(BUZZER_RIGHT_GPIO);

    uint32_t pwm_wrap = 12500;

    // Configura PWM com clock mais lento para som mais suave e volume reduzido
    pwm_set_clkdiv(slice_left, 80.0f);
    pwm_set_wrap(slice_left, pwm_wrap);
    pwm_set_clkdiv(slice_right, 80.0f);
    pwm_set_wrap(slice_right, pwm_wrap);

    pwm_set_gpio_level(BUZZER_LEFT_GPIO, 0);
    pwm_set_gpio_level(BUZZER_RIGHT_GPIO, 0);

    pwm_set_enabled(slice_left, true);
    pwm_set_enabled(slice_right, true);

    const float min_threshold = 0.2f;     // abaixo disso, não emite som
    const float max_accel = 2.0f;
    const float max_duty_fraction = 0.4f; // volume mais baixo (até 40%)

    while (true) {
        if (xQueuePeek(xAccelQueue, &accel_data, pdMS_TO_TICKS(100)) == pdPASS) {
            float ax = (float)accel_data.y / 16384.0f;

            float duty = 0.0f;
            if (fabsf(ax) > min_threshold) {
                float norm = (fabsf(ax) - min_threshold) / (max_accel - min_threshold);
                if (norm > 1.0f) norm = 1.0f;
                duty = norm * max_duty_fraction;
            }

            uint16_t level = (uint16_t)(duty * pwm_wrap);

            if (ax > min_threshold) {
                pwm_set_gpio_level(BUZZER_RIGHT_GPIO, level);
                pwm_set_gpio_level(BUZZER_LEFT_GPIO, 0);
            } else if (ax < -min_threshold) {
                pwm_set_gpio_level(BUZZER_LEFT_GPIO, level);
                pwm_set_gpio_level(BUZZER_RIGHT_GPIO, 0);
            } else {
                pwm_set_gpio_level(BUZZER_LEFT_GPIO, 0);
                pwm_set_gpio_level(BUZZER_RIGHT_GPIO, 0);
            }

        } else {
            // Falha ao ler fila: silencia ambos
            pwm_set_gpio_level(BUZZER_LEFT_GPIO, 0);
            pwm_set_gpio_level(BUZZER_RIGHT_GPIO, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//tarefa que gerencia as leituras do sensor e envia para as filas
void vMPUTask()
{
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_init(&ssd, DISP_W, DISP_H, false, ENDERECO_DISP, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));

    mpu6050_reset();

    int16_t aceleracao[3], gyro[3], temp;

    // --- Variáveis para as Barras de Giroscópio (Parte Superior da Tela) ---
    int center_x_gyro_bar = DISP_W / 2;    // Centro horizontal para todas as barras de giro
    int bar_width_max = (DISP_W / 2) - 10; // Largura máxima de cada barra de giro

    // Posições Y para as barras de Giroscópio
    int y_pos_gx = 12; 
    int y_pos_gy = 23; 
    int y_pos_gz = 34; 

    // Escala para os valores do Giroscópio (mapear para pixels)
    float gyro_scale_factor = (float)bar_width_max / 32767.0f; // Mapeia o max. int16_t para bar_width_max

    // Variáveis de limiar e last_value para giroscópio
    float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;
    const float gyro_limiar = 100.0f; // Zona morta para giroscópio (em LSB)

    // --- Variáveis para as Linhas de Pitch/Roll (Parte Inferior da Tela) ---
    int center_x_accel_line = DISP_W / 2;
    // As linhas de Pitch/Roll serão desenhadas mais abaixo no display
    int y_pos_accel_ref = 55; 

    // Escala para Pitch/Roll: quantos pixels por grau
    float pixels_per_degree_roll = 0.8f;  // Ajuste conforme a sensibilidade desejada
    float pixels_per_degree_pitch = 0.8f; 

    // Comprimento da linha indicadora de Pitch/Roll
    int accel_line_length = 25;

    float last_roll = 0.0f, last_pitch = 0.0f;
    const float accel_limiar = 1.0f; // Zona morta para Pitch/Roll (em graus)

    while (1)
    {
        mpu6050_read_raw(aceleracao, gyro, &temp);

        // Giroscópio: valores brutos gx, gy, gz
        float gx = (float)gyro[0];
        float gy = (float)gyro[1];
        float gz = (float)gyro[2];
        float ax = (float)aceleracao[0];
        float ay = (float)aceleracao[1];
        float az = (float)aceleracao[2];

        mpu6050_data sensorData = {
                .gx = gx,
                .gy = gy,
                .gz = gz,
                .ax = ax,
                .ay = ay,
                .az = az
        };
        mpu_accel accelData = {
            .x = ax,
            .y = ay,
            .z = az
        };
        xQueueOverwrite(xSensorDataQueue,&sensorData);
        xQueueOverwrite(xAccelQueue, &accelData);

        // --- Cálculos para Acelerômetro (Pitch/Roll) ---
        ax = aceleracao[0] / 16384.0f; // Escala para 'g'
        ay = aceleracao[1] / 16384.0f;
        az = aceleracao[2] / 16384.0f;

        float roll = atan2(ay, az) * 180.0f / M_PI;
        float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

        // Atualiza o display apenas se houver mudança significativa em qualquer sensor
        if (fabs(gx - last_gx) > gyro_limiar || fabs(gy - last_gy) > gyro_limiar || fabs(gz - last_gz) > gyro_limiar ||
            fabs(roll - last_roll) > accel_limiar || fabs(pitch - last_pitch) > accel_limiar)
        {

            last_gx = gx;
            last_gy = gy;
            last_gz = gz;
            last_roll = roll;
            last_pitch = pitch;

            ssd1306_fill(&ssd, false); // Limpa o display a cada atualização

            // --- Seção do Giroscópio (Top da Tela) ---
            ssd1306_draw_string(&ssd, "GIRO:", 2, 0); // Título para Giro

            // Barra de GX
            int bar_end_x_gx = center_x_gyro_bar + (int)(gx * gyro_scale_factor);
            if (bar_end_x_gx < 0)
                bar_end_x_gx = 0;
            if (bar_end_x_gx > DISP_W - 1)
                bar_end_x_gx = DISP_W - 1;
            ssd1306_line(&ssd, center_x_gyro_bar, y_pos_gx, bar_end_x_gx, y_pos_gx, true);
            // ssd1306_draw_string(&ssd, "GX", 2, y_pos_gx - 4); // Rótulo do eixo

            // Barra de GY
            int bar_end_x_gy = center_x_gyro_bar + (int)(gy * gyro_scale_factor);
            if (bar_end_x_gy < 0)
                bar_end_x_gy = 0;
            if (bar_end_x_gy > DISP_W - 1)
                bar_end_x_gy = DISP_W - 1;
            ssd1306_line(&ssd, center_x_gyro_bar, y_pos_gy, bar_end_x_gy, y_pos_gy, true);
            // ssd1306_draw_string(&ssd, "GY", 2, y_pos_gy - 4);

            // Barra de GZ
            int bar_end_x_gz = center_x_gyro_bar + (int)(gz * gyro_scale_factor);
            if (bar_end_x_gz < 0)
                bar_end_x_gz = 0;
            if (bar_end_x_gz > DISP_W - 1)
                bar_end_x_gz = DISP_W - 1;
            ssd1306_line(&ssd, center_x_gyro_bar, y_pos_gz, bar_end_x_gz, y_pos_gz, true);
            // ssd1306_draw_string(&ssd, "GZ", 2, y_pos_gz - 4);

            // Linha de divisão entre Giro e Acelerômetro
            ssd1306_line(&ssd, 0, 42, DISP_W - 1, 42, true);

            // --- Seção do Acelerômetro (Bottom da Tela) ---
            ssd1306_draw_string(&ssd, "ACEL:", 2, 45); // Título para Acelerômetro

            // Mapeamento do Roll para a posição X da linha
            int offset_x_roll = (int)(roll * pixels_per_degree_roll);
            int line_x_roll = center_x_accel_line + offset_x_roll;

            if (line_x_roll < 0)
                line_x_roll = 0;
            if (line_x_roll > DISP_W - 1)
                line_x_roll = DISP_W - 1;

            // Mapeamento do Pitch para a posição Y da linha
            int offset_y_pitch = (int)(pitch * pixels_per_degree_pitch);
            int line_y_pitch = y_pos_accel_ref + offset_y_pitch;

            if (line_y_pitch < 43)
                line_y_pitch = 43; // Garante que a linha não invada a área do giro
            if (line_y_pitch > DISP_H - 1)
                line_y_pitch = DISP_H - 1;

            // Linha de referência central para Pitch/Roll
            ssd1306_line(&ssd, center_x_accel_line - (accel_line_length / 8), y_pos_accel_ref, center_x_accel_line + (accel_line_length / 8), y_pos_accel_ref, true); // Eixo X central
            ssd1306_line(&ssd, center_x_accel_line, y_pos_accel_ref - 2, center_x_accel_line, y_pos_accel_ref + 2, true);                                             // Eixo Y central

            // Desenha a linha indicadora do Roll (vertical que se move horizontalmente)
            ssd1306_line(&ssd, line_x_roll, y_pos_accel_ref - (accel_line_length / 2) + 5, line_x_roll, y_pos_accel_ref + (accel_line_length / 2) - 5, true);

            // Desenha a linha indicadora do Pitch (horizontal que se move verticalmente)
            ssd1306_line(&ssd, center_x_accel_line - (accel_line_length / 2) + 5, line_y_pitch, center_x_accel_line + (accel_line_length / 2) - 5, line_y_pitch, true);

            // Exibe os valores numéricos de Roll e Pitch
            char str_roll[20], str_pitch[20];
            snprintf(str_roll, sizeof(str_roll), "R:%5.1f", roll);
            snprintf(str_pitch, sizeof(str_pitch), "P:%5.1f", pitch);
            // ssd1306_draw_string(&ssd, str_roll, 5, 54);
            // ssd1306_draw_string(&ssd, str_pitch, 68, 54);

            ssd1306_send_data(&ssd);                   // Envia os dados para o display
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();

    xSensorDataQueue = xQueueCreate(1, sizeof(mpu6050_data));
    xAccelQueue = xQueueCreate(1, sizeof(mpu_accel));

    xTaskCreate(vSDTask, "SD Card Task", 1024, NULL, tskIDLE_PRIORITY, NULL); //inicia a tarefa de gerência do sd
    xTaskCreate(vMPUTask, "MPU Read Task", 1024, NULL, tskIDLE_PRIORITY, NULL); // inicia a tarefa de leitura do sensor
    xTaskCreate(vTaskLedFromAccel, "LED Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL); //inicia a tarefa de gerência do LED RGB
    xTaskCreate(vTaskBuzzerFromAccel, "LED Buzzer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL); //inicia a tarefa de gerência dos Buzzers
    xTaskCreate(vMatrixTask, "Matrix Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL); //inicia a tarefa de gerência da matriz de LEDs
    vTaskStartScheduler();
    panic_unsupported();

}

static void run_help()
{
    printf("\nComandos disponíveis:\n\n");
    printf("Digite 'a' para montar o cartão SD\n");
    printf("Digite 'b' para desmontar o cartão SD\n");
    printf("Digite 'c' para listar arquivos\n");
    printf("Digite 'd' para mostrar conteúdo do arquivo\n");
    printf("Digite 'e' para obter espaço livre no cartão SD\n");
    printf("Digite 'f' para capturar dados do ADC e salvar no arquivo\n");
    printf("Digite 'g' para formatar o cartão SD\n");
    printf("Digite 'h' para exibir os comandos disponíveis\n");
    printf("\nEscolha o comando:  ");
}

static void process_stdio(int cRxedChar)
{
    static char cmd[256];
    static size_t ix;

    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != (char)127)
        return;
    printf("%c", cRxedChar); // echo
    stdio_flush();
    if (cRxedChar == '\r')
    {
        printf("%c", '\n');
        stdio_flush();

        if (!strnlen(cmd, sizeof cmd))
        {
            printf("> ");
            stdio_flush();
            return;
        }
        char *cmdn = strtok(cmd, " ");
        if (cmdn)
        {
            size_t i;
            for (i = 0; i < count_of(cmds); ++i)
            {
                if (0 == strcmp(cmds[i].command, cmdn))
                {
                    (*cmds[i].function)();
                    break;
                }
            }
            if (count_of(cmds) == i)
                printf("Command \"%s\" not found\n", cmdn);
        }
        ix = 0;
        memset(cmd, 0, sizeof cmd);
        printf("\n> ");
        stdio_flush();
    }
    else
    {
        if (cRxedChar == '\b' || cRxedChar == (char)127)
        {
            if (ix > 0)
            {
                ix--;
                cmd[ix] = '\0';
            }
        }
        else
        {
            if (ix < sizeof cmd - 1)
            {
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}