#include "sdspicard.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

bool logger_enabled;
const uint32_t period = 1000;
absolute_time_t next_log_time;
char filename[20] = "new_file.txt";

sd_card_t *sd_get_by_name(const char *const name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
FATFS *sd_get_fs_by_name(const char *name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

void run_setrtc()
{
    const char *dateStr = strtok(NULL, " ");
    if (!dateStr)
    {
        printf("Missing argument\n");
        return;
    }
    int date = atoi(dateStr);

    const char *monthStr = strtok(NULL, " ");
    if (!monthStr)
    {
        printf("Missing argument\n");
        return;
    }
    int month = atoi(monthStr);

    const char *yearStr = strtok(NULL, " ");
    if (!yearStr)
    {
        printf("Missing argument\n");
        return;
    }
    int year = atoi(yearStr) + 2000;

    const char *hourStr = strtok(NULL, " ");
    if (!hourStr)
    {
        printf("Missing argument\n");
        return;
    }
    int hour = atoi(hourStr);

    const char *minStr = strtok(NULL, " ");
    if (!minStr)
    {
        printf("Missing argument\n");
        return;
    }
    int min = atoi(minStr);

    const char *secStr = strtok(NULL, " ");
    if (!secStr)
    {
        printf("Missing argument\n");
        return;
    }
    int sec = atoi(secStr);

    datetime_t t = {
        .year = (int16_t)year,
        .month = (int8_t)month,
        .day = (int8_t)date,
        .dotw = 0, // 0 is Sunday
        .hour = (int8_t)hour,
        .min = (int8_t)min,
        .sec = (int8_t)sec};
    rtc_set_datetime(&t);
}

void run_format()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    /* Format the drive with default parameters */
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);
    if (FR_OK != fr)
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
}
void run_mount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr)
    {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
}
void run_unmount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr)
    {
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // in case medium is removed
    printf("SD ( %s ) desmontado\n", pSD->pcName);
}
void run_getfree()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    DWORD fre_clust, fre_sect, tot_sect;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);
    if (FR_OK != fr)
    {
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;
    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
}
void run_ls()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = "";
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;
    if (arg1[0])
    {
        p_dir = arg1;
    }
    else
    {
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr)
        {
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            return;
        }
        p_dir = cwdbuf;
    }
    printf("Directory Listing: %s\n", p_dir);
    DIR dj;
    FILINFO fno;
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, p_dir, "*");
    if (FR_OK != fr)
    {
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    while (fr == FR_OK && fno.fname[0])
    {
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        if (fno.fattrib & AM_DIR)
            pcAttrib = pcDirectory;
        else if (fno.fattrib & AM_RDO)
            pcAttrib = pcReadOnlyFile;
        else
            pcAttrib = pcWritableFile;
        printf("%s [%s] [size=%llu]\n", fno.fname, pcAttrib, fno.fsize);

        fr = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);
}
void run_cat()
{
    char *arg1 = strtok(NULL, " ");
    if (!arg1)
    {
        printf("Missing argument\n");
        return;
    }
    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);
    if (FR_OK != fr)
    {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil))
    {
        printf("%s", buf);
    }
    fr = f_close(&fil);
    if (FR_OK != fr)
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
}


float normalize_gyro(int16_t val) {
    return (float)val / 131.0f;
}

float normalize_accel(int16_t val) {
    return (float)val / 16384.0f;
}

// Função para capturar dados do ADC e salvar no arquivo *.txt
void capture_adc_data_and_save(int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay, int16_t az)
{
    //printf("%d, %d, %d, %d, %d, %d\n", gx, gy, gz, ax, ay, az);
    float ngx, ngy, ngz, nax, nay, naz;
    ngx = normalize_gyro(gx);
    ngy = normalize_gyro(gy);
    ngz = normalize_gyro(gz);
    nax = normalize_accel(ax);
    nay = normalize_accel(ay);
    naz = normalize_accel(az);
    //printf("%f, %f, %f, %f, %f, %f\n", ngx, ngy, ngz, nax, nay, naz);
    printf("\nCapturando dados do sensor. Aguarde finalização...\n");
    FIL file;
    FRESULT res = f_open(&file, filename, FA_WRITE | FA_OPEN_EXISTING);
    if (res == FR_NO_FILE)
    {
        res = f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
        if (res != FR_OK) {
            printf("\n[ERRO] Não foi possível criar o arquivo.\n");
            return;
        }

        const char *header = "G_x,G_y,G_z, A_X, A_Y, A_Z\n";
        UINT bw;
        f_write(&file, header, strlen(header), &bw);
    } else if (res == FR_OK)
    {
        f_lseek(&file, f_size(&file));
    }
    else
    {
        printf("\n[ERRO] Não foi possível abrir o arquivo para escrita. Monte o Cartao.\n");
        return;
    }
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", ngx, ngy, ngz, nax, nay, naz);

    UINT bw;
    res = f_write(&file, buffer, strlen(buffer), &bw);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível escrever no arquivo. Monte o Cartao.\n");
        f_close(&file);
        return;
    }
    f_close(&file);
    printf("\nDados do ADC salvos no arquivo %s.\n\n", filename);
}

// Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename)
{
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível abrir o arquivo para leitura. Verifique se o Cartão está montado ou se o arquivo existe.\n");

        return;
    }
    char buffer[128];
    UINT br;
    printf("Conteúdo do arquivo %s:\n", filename);
    while (f_read(&file, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0)
    {
        buffer[br] = '\0';
        printf("%s", buffer);
    }
    f_close(&file);
    printf("\nLeitura do arquivo %s concluída.\n\n", filename);
}
