# Projeto de Monitoramento com MPU6050, Matriz de LEDs RGB, Buzzer e Cartão SD no Raspberry Pi Pico usando FreeRTOS

Este projeto implementa a leitura do sensor inercial MPU6050 (acelerômetro e giroscópio), exibição de informações em display OLED SSD1306, controle de matriz de LEDs RGB via PIO, gerenciamento de buzzer PWM e operações em cartão SD, tudo rodando em multitarefa com FreeRTOS no Raspberry Pi Pico.

---

## Funcionalidades Principais

- **Leitura dos dados do MPU6050 (aceleração, giroscópio e temperatura) via I2C.**
- **Exibição gráfica e numérica dos dados no display OLED SSD1306 via I2C.**
- **Controle de matriz LED RGB (5x5) usando PIO para exibir padrões coloridos que respondem à aceleração no eixo Y.**
- **Controle de LEDs RGB separados, indicando níveis de aceleração total.**
- **Tocador de buzzer estéreo via PWM controlado pela aceleração no eixo Y, com intensidade e lado variáveis.**
- **Interface serial via USB para comandos de gerenciamento do cartão SD (montar, desmontar, listar, ler arquivos, formatar, salvar dados).**
- **Uso do FreeRTOS para multitarefa eficiente, com filas para comunicação entre tarefas.**
- **Modo BOOTSEL ativado via botão físico para fácil atualização de firmware.**

---

## Ligações e Pinos Usados

### MPU6050 (I2C0: GPIO 0 = SDA, GPIO 1 = SCL)

| MPU6050 | Pico GPIO |
|---------|-----------|
| VCC     | 3.3V      |
| GND     | GND       |
| SDA     | GP0       |
| SCL     | GP1       |

### Display SSD1306 (I2C1: GPIO 14 = SDA, GPIO 15 = SCL)

| SSD1306 | Pico GPIO |
|---------|-----------|
| VCC     | 3.3V      |
| GND     | GND       |
| SDA     | GP14      |
| SCL     | GP15      |

### LEDs e Buzzers

| Dispositivo          | GPIO      | Função                                      |
|---------------------|-----------|---------------------------------------------|
| Botão BOOTSEL       | GPIO 6    | Pressionar para modo BOOTSEL                 |
| LED Verde           | GPIO 11   | Indicador de aceleração baixa                 |
| LED Azul            | GPIO 12   | (não usado no código para LEDs)               |
| LED Vermelho        | GPIO 13   | Indicador de aceleração alta                  |
| Buzzer esquerdo     | GPIO 10   | PWM para buzzer esquerdo (aceleração negativa no eixo Y) |
| Buzzer direito      | GPIO 21   | PWM para buzzer direito (aceleração positiva no eixo Y)  |
| Matriz LED RGB (PIO)| GPIO 7    | Controle via PIO                              |

---

## Bibliotecas e Dependências

- FreeRTOS para multitarefa
- SDK do Raspberry Pi Pico para periféricos (GPIO, PWM, I2C, PIO)
- Biblioteca SSD1306 para controle do display OLED
- Biblioteca MPU6050 para leitura do sensor inercial
- Biblioteca FATFS para sistema de arquivos no cartão SD
- Código customizado para controle PIO da matriz LED RGB

---

## Estrutura das Tarefas

| Tarefa              | Descrição                                                       |
|---------------------|-----------------------------------------------------------------|
| `vMPUTask`          | Leitura do MPU6050, cálculo de roll/pitch, atualização do display OLED com gráficos e valores. Publica dados em filas para outras tarefas. |
| `vTaskLedFromAccel` | Controla LEDs RGB indicativos da magnitude da aceleração total. |
| `vTaskBuzzerFromAccel` | Controla buzzer estéreo PWM baseado no valor do eixo Y da aceleração. |
| `vMatrixTask`       | Controla matriz LED RGB via PIO, mostrando padrões que mudam conforme aceleração no eixo Y. |
| `vSDTask`           | Interface serial para comandos de gerenciamento do cartão SD (montar, desmontar, listar, formatar, salvar dados ADC). |

---

## Uso

- Conecte o Raspberry Pi Pico e sensores conforme as ligações acima.
- Compile e grave o firmware.
- Ao ligar, use o terminal serial para acessar comandos para gerenciamento do cartão SD.
- Pressione o botão conectado ao GPIO 6 para entrar no modo BOOTSEL e atualizar firmware.
- Observe a matriz de LEDs, LEDs RGB e ouça o buzzer reagindo ao movimento.
- No display OLED, veja em tempo real os dados do sensor.

---

## Comandos via Serial USB (dentro da tarefa SD)

| Tecla | Função                             |
|-------|----------------------------------|
| a     | Montar cartão SD                  |
| b     | Desmontar cartão SD              |
| c     | Listar arquivos do cartão SD     |
| d     | Exibir conteúdo de arquivo       |
| e     | Mostrar espaço livre no cartão SD|
| f     | Capturar e salvar dados ADC no arquivo |
| g     | Formatar cartão SD               |
| h     | Mostrar ajuda                    |

---

## Personalizações e Melhorias Possíveis

- Ajustar cores e padrões na matriz LED RGB conforme outros eixos ou sensores.
- Implementar filtro de Kalman ou média móvel para suavizar dados do MPU6050.
- Adicionar controle de volume PWM mais sofisticado para o buzzer.
- Incluir comunicação via UART/Bluetooth para enviar dados remotamente.
- Expandir interface de comandos para manipulação avançada de arquivos no cartão SD.
- Incluir sensor magnético para cálculo de Yaw (orientação).

---

## Referências

- [Raspberry Pi Pico SDK Documentation](https://www.raspberrypi.com/documentation/pico-sdk/)
- [MPU6050 Datasheet](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [SSD1306 OLED Datasheet](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)
- [FreeRTOS Official Site](https://www.freertos.org/)
- [PIO Programming Guide (Raspberry Pi)](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)

