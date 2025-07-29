#include "mpu6050.h" 
#include "sdspicard.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "pico/bootrom.h"
#define botaoB 6
#define FLAG_VALUE 0X22

typedef void (*p_fn_t)();
typedef struct
{
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

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
            printf("PASSOU DIRETO!\n");
            capture_adc_data_and_save();
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
    
    // Variáveis para as Barras de Giroscópio
    int center_x_gyro_bar = DISP_W / 2; 
    int bar_width_max = DISP_W / 2 - 10; 
    
    // Posições Y para as barras de Giroscópio
    int y_pos_gx = 18; 
    int y_pos_gy = 33; 
    int y_pos_gz = 48; 

    // Escala para os valores do Giroscópio para mapear para pixels
    float gyro_scale_factor = (float)bar_width_max / 32767.0f; 
                                                              
    // Variáveis de limiar e last_value para giroscópio
    float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;
    const float gyro_limiar = 100.0f; // Zona morta para giroscópio 

    while (1)
    {
        mpu6050_read_raw(aceleracao, gyro, &temp);

        // Giroscópio: valores brutos gx, gy, gz
        float gx = (float)gyro[0];
        float gy = (float)gyro[1];
        float gz = (float)gyro[2];

        // Atualiza apenas se houver mudança significativa no giroscópio
        if (fabs(gx - last_gx) > gyro_limiar || fabs(gy - last_gy) > gyro_limiar || fabs(gz - last_gz) > gyro_limiar) {
            last_gx = gx;
            last_gy = gy;
            last_gz = gz;

            char str_gx[20], str_gy[20], str_gz[20];
            snprintf(str_gx, sizeof(str_gx), "GX:%6d", (int)gx);
            snprintf(str_gy, sizeof(str_gy), "GY:%6d", (int)gy);
            snprintf(str_gz, sizeof(str_gz), "GZ:%6d", (int)gz);


            ssd1306_fill(&ssd, false); // Limpa o display a cada atualização

            // Título
            ssd1306_draw_string(&ssd, "Taxa Giro IMU", 10, 2);

            // --- Barra de GX ---
            // Linha de referência central para GX
            ssd1306_line(&ssd, center_x_gyro_bar, y_pos_gx, center_x_gyro_bar, y_pos_gx + 4, true); 
            
            int bar_end_x_gx = center_x_gyro_bar + (int)(gx * gyro_scale_factor);
            
            // Limitar a barra dentro dos limites do display
            if (bar_end_x_gx < 0) bar_end_x_gx = 0;
            if (bar_end_x_gx > DISP_W - 1) bar_end_x_gx = DISP_W - 1;

            // Desenha a barra de GX
            ssd1306_line(&ssd, center_x_gyro_bar, y_pos_gx + 2, bar_end_x_gx, y_pos_gx + 2, true);
            // Exibe o valor de GX
            ssd1306_draw_string(&ssd, "X", 5, y_pos_gx + 8); // Posiciona abaixo da barra

            // --- Barra de GY ---
            // Linha de referência central para GY
            ssd1306_line(&ssd, center_x_gyro_bar, y_pos_gy, center_x_gyro_bar, y_pos_gy + 4, true); 

            int bar_end_x_gy = center_x_gyro_bar + (int)(gy * gyro_scale_factor);
            
            if (bar_end_x_gy < 0) bar_end_x_gy = 0;
            if (bar_end_x_gy > DISP_W - 1) bar_end_x_gy = DISP_W - 1;

            // Desenha a barra de GY
            ssd1306_line(&ssd, center_x_gyro_bar, y_pos_gy + 2, bar_end_x_gy, y_pos_gy + 2, true);
            // Exibe o valor de GY
            ssd1306_draw_string(&ssd, "Y", 5, y_pos_gy + 8);

            // --- Barra de GZ ---
            // Linha de referência central para GZ
            ssd1306_line(&ssd, center_x_gyro_bar, y_pos_gz, center_x_gyro_bar, y_pos_gz + 4, true); 
            
            int bar_end_x_gz = center_x_gyro_bar + (int)(gz * gyro_scale_factor);
            
            if (bar_end_x_gz < 0) bar_end_x_gz = 0;
            if (bar_end_x_gz > DISP_W - 1) bar_end_x_gz = DISP_W - 1;

            // Desenha a barra de GZ
            ssd1306_line(&ssd, center_x_gyro_bar, y_pos_gz + 2, bar_end_x_gz, y_pos_gz + 2, true);
            // Exibe o valor de GZ
            ssd1306_draw_string(&ssd, "Z", 5, y_pos_gz + 8);

            ssd1306_send_data(&ssd); // Envia os dados para o display
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

    xTaskCreate(vSDTask, "SD Card Task", 1024, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vMPUTask, "MPU Read Task", 1024, NULL, tskIDLE_PRIORITY, NULL);
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