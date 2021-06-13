#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "driver/dac.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#define pi (3.14159265)

// Timer Config ------------------------------------------------------------------------

#define PRESCALER (4000)  // Fator de escala
#define TIMER_SCALE (1.0*PRESCALER / TIMER_BASE_CLK)  // Conversão para segundos
// TIMER_BASE_CLOCK = 240/3 MHz

typedef struct {
    int timer_group; // Grupo
    int timer_idx; // Timer do grupo
    int alarm_interval; // Intervalo de alarme
    bool auto_reload; // Reinício automático
} timer_info_t; // Informações do timer

typedef struct {
    timer_info_t info;
    uint64_t timer_counter_value;
} timer_event_t; // Eventos do timer

static xQueueHandle s_timer_queue; // Gerenciador de queue

// Configuração do timer
static void t_init(int group, int timer, bool auto_reload, uint64_t timer_interval_count);

// Gerenciador do interruptor do timer
static bool IRAM_ATTR timer_group_isr_callback(void *args);

// Timer Config ------------------------------------------------------------------------

// MODBUS Config -----------------------------------------------------------------------

#define MB_PORT_NUM     (UART_NUM_0)   // Porta UART
#define MB_SLAVE_ADDR   (1)      // ID do equipamento na rede
#define MB_DEV_SPEED    (115200)  // Taxa de transmissão de dados

// Definindo endereço do registrador de contenção na memória
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
#define MB_REG_HOLDING_START_AREA0          (HOLD_OFFSET(freq))

#ifndef _DEVICE_PARAMS
#define _DEVICE_PARAMS

#pragma pack(push, 1)
typedef struct
{
    float freq;
    float Amp;
    float Kp;
    float Ki;
    uint16_t test_regs[150];

} holding_reg_params_t; // Registrador de contenção
#pragma pack(pop)

#endif // !defined(_DEVICE_PARAMS)

holding_reg_params_t holding_reg_params = { 0 };

void mdb_init(void);

// MODBUS Config -----------------------------------------------------------------------

// UART Config -------------------------------------------------------------------------

static const int RX_BUF_SIZE = 1024;

void UART_init(void);

// UART Config -------------------------------------------------------------------------

// ADC Config --------------------------------------------------------------------------

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t channelA = ADC1_CHANNEL_6; // GPIO34
static const adc_channel_t channelB = ADC1_CHANNEL_7; // GPIO35

void ADC_init(void);

// ADC Config --------------------------------------------------------------------------

// controlador PI ----------------------------------------------------------------------

float e[2] = {0., 0.}; // Erro
float u[2] = {0., 0.}; // Variável de comando

float Kp; // Ganho da parcela proporcional
float Ki; // Ganho da parcela integral

float Umin; // Limite inferior de u
float Umax; // Limite superior de u

// controlador PI ----------------------------------------------------------------------

float t; // tempo

// -------------------------------------------------------------------------------------------------

void app_main() {

    mdb_init(); // Configurações do modbus

    mbc_slave_start(); // Inicializar o escravo

    UART_init(); // Configurações do UART

    ADC_init(); // Configurações do conversor analógico - digital

    t_init(TIMER_GROUP_1, TIMER_1, true, 20); // Configurações do timer
    // Timer 1 do grupo 1
    // 20 contas -> 0.001 s

    // Variáveis para leitura dos ADC's
    uint32_t adc_readingA = 0;
    uint32_t adc_readingB = 0;

    uint64_t timer_count = 0; // Contador de tempo

    t = 0.; // Valor inicial do tempo igual a zero
    float Ts = 0.001; // Tempo de amostragem
    float v_control = 0.; // controle de frequência

    float Va = 0.; // Tensão no polo positivo da carga
    float Vb = 0.; // Tensão no polo negativo da carga
    float vo = 0.; // Tensão na carga

    float w = 0.; // Frquência
    float A = 0.; // Amplitude

    bool test = true; // Variável secundária

    // Valores iniciais dos parâmetros ajustáveis
    // Registrador de contenção
    holding_reg_params.freq = 10.;
    holding_reg_params.Amp = 1.;
    holding_reg_params.Kp = 0.1;
    holding_reg_params.Ki = 10.;

    // Conversão de 3.3V para inteiro de 8 bits
    float kappa = 255/3.3;

    // Valores dos limites de saturação
    Umin = 0.142;
    Umax = 3.3;
    
    // Valor inicial de erro
    // Com vo = 0 para t = 0, e0 = A
    e[0] = A;

    // Caracterização inicial do PI
    Kp = holding_reg_params.Kp;
    Ki = holding_reg_params.Ki;

    // Interface entre main e o interruptor do timer
    s_timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Configurando GPIO's 18 e 19 como saídas
    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);

    // Configurando o DAC
    dac_output_enable(DAC_CHANNEL_2); // GPIO 26
    dac_output_voltage(DAC_CHANNEL_2, 0); // Valor inical nulo

    while(1){

        // Recebendo dados do interruptor
        xQueueReceive(s_timer_queue, &timer_count, portMAX_DELAY);

        // Alterando o valor do DAC
        dac_output_voltage(DAC_CHANNEL_2, (int)(kappa*u[0]));

        // Atualizando o tempo
        t += TIMER_SCALE*timer_count;

        // Bloco de controle da frequência
        w = 2*pi*holding_reg_params.freq;

        v_control = sin(w*t);

        if(test & (v_control > 0)){
            gpio_set_level(GPIO_NUM_18, 0);
            gpio_set_level(GPIO_NUM_19, 1);
            test = false; // test permite entrada no if uma vez por ciclo
        }
        else if(!test & (v_control < 0)){
            gpio_set_level(GPIO_NUM_19, 0);
            gpio_set_level(GPIO_NUM_18, 1);
            test = true;
        }

        // Bloco de medição de tensão
        adc_readingA = adc1_get_raw(channelA);
        adc_readingB = adc1_get_raw(channelB);

        Va = (float)esp_adc_cal_raw_to_voltage(adc_readingA, adc_chars)*0.001;
        Vb = (float)esp_adc_cal_raw_to_voltage(adc_readingB, adc_chars)*0.001;

        vo = Va - Vb;

        A = holding_reg_params.Amp;

        e[1] = A - fabs(vo);

        // Bloco do controlador PI
        Kp = holding_reg_params.Kp;

        u[1] = u[0] + Kp*e[1] + (Ki*Ts - Kp)*e[0];

        e[0] = e[1];
        u[0] = u[1];

        if(u[0] > Umax){
            u[0] = Umax;
            Ki = 0;
        }
        else if(u[0] < Umin){
            u[0] = Umin;
            Ki = 0;
        }
        else{
            Ki = holding_reg_params.Ki;
        }

        // printf("e: %f V, u: %f V, vo: %f, t: %f s\r\n", e[0], u[0], fabs(vo), t);
    }  
}

// -------------------------------------------------------------------------------------------------

// Timer Functions ------------------------------------------------------------------------

static void t_init(int group, int timer, bool auto_reload, uint64_t timer_interval_count)
{
    // Selecionar e inicializar as propriedades básicas do timer
    timer_config_t config = {
        .divider = PRESCALER, // Fator de escala
        .counter_dir = TIMER_COUNT_UP, // Contar para cima
        .counter_en = TIMER_PAUSE, // Permite contagem
        .alarm_en = TIMER_ALARM_EN, // Permite alarme
        .auto_reload = auto_reload, // Reinício automático
    };
    timer_init(group, timer, &config); // Inicializa o timer de um grupo

    // Valor inicial do contador
    timer_set_counter_value(group, timer, 0);

    // Configurando o valor que ativa o alarme do timer
    timer_set_alarm_value(group, timer, timer_interval_count);

    // Permite interruptor
    timer_enable_intr(group, timer);

    timer_info_t *timer_info = calloc(1, sizeof(timer_info_t));
    timer_info->timer_group = group; // grupo
    timer_info->timer_idx = timer; // timer do grupo
    timer_info->auto_reload = auto_reload; // reinício automático
    timer_info->alarm_interval = timer_interval_count; // intervalo de contagem
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0); // interruptor

    // Inicia o timer
    timer_start(group, timer);
}

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    // Prioridade
    BaseType_t high_task_awoken = pdFALSE;
    timer_info_t *info = (timer_info_t *) args;

    // Extrai o valor atual do contador
    uint64_t timer_count = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    // Prepara dados de eventos que serão enviados a seção principal
    if (info->auto_reload) {
        timer_count += info->alarm_interval;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_count);
    }

    // Envio de dados ao programa principal
    xQueueSendFromISR(s_timer_queue, &timer_count, &high_task_awoken);

    return high_task_awoken == pdTRUE;
}

// Timer Functions ------------------------------------------------------------------------

// MODBUS Functions -----------------------------------------------------------------------

void mdb_init(void){
    mb_communication_info_t comm_info; // Parâmetros de comunicação do Modbus
    mb_register_area_descriptor_t reg_area; // Estrutura do descritor do registrador de área

    void* mbc_slave_handler = NULL;

    mbc_slave_init(MB_PORT_SERIAL_SLAVE, &mbc_slave_handler); // Inicialização do escravo

    // Configurando parâmetros de comunicação e inicializando stack
    comm_info.mode = MB_MODE_RTU, // Tipo
    comm_info.slave_addr = MB_SLAVE_ADDR; // ID
    comm_info.port = MB_PORT_NUM; // Porta UART
    comm_info.baudrate = MB_DEV_SPEED; // Taxa de transmissão
    comm_info.parity = MB_PARITY_NONE; // Sem paridade
    mbc_slave_setup((void*)&comm_info);

    reg_area.type = MB_PARAM_HOLDING; // Tipo de registrador
    reg_area.start_offset = MB_REG_HOLDING_START_AREA0; // Deslocando o endereço do registrador
    reg_area.address = (void*)&holding_reg_params.freq; // Pointer para armazenamento
    // Configura o tamanho do armazenamento do registrador -> 150 registradores de contenção
    reg_area.size = sizeof(float) << 2; // Tamanho do armazenamento individual (por variável)
    mbc_slave_set_descriptor(reg_area);
}

// MODBUS Functions -----------------------------------------------------------------------

// UART Functions -------------------------------------------------------------------------

void UART_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = MB_DEV_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 20, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// UART Functions -------------------------------------------------------------------------

// ADC Functions --------------------------------------------------------------------------

void ADC_init(void){
    adc1_config_width(ADC_WIDTH_12Bit);

    adc1_config_channel_atten(channelA, ADC_ATTEN_11db);
    adc1_config_channel_atten(channelB, ADC_ATTEN_11db);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, DEFAULT_VREF, adc_chars);
}

// ADC Functions --------------------------------------------------------------------------
