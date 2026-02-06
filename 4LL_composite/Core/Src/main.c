#include "main.h"
#include "adc.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_hid.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#define KEY_COUNT 20
#define RT_ACTUATION_DIST  20   
#define RT_RELEASE_DIST    20   
#define STATIC_AP_THRESHOLD 50 

// --- ボタンID定義 ---
#define ID_UP 0
#define ID_DOWN 1
#define ID_LEFT 2
#define ID_RIGHT 3
#define ID_BTN1 4   
#define ID_BTN2 5   
#define ID_BTN3 6   
#define ID_BTN4 7   
#define ID_BTN5 8   
#define ID_BTN6 9   
#define ID_BTN7 10  
#define ID_BTN8 11  
#define ID_BTN9 12  
#define ID_BTN10 13  
#define ID_BTN11 14  
#define ID_BTN12 15  
#define ID_BTN13 16  
#define ID_BTN14 17  
#define ID_BTN15 18  
#define ID_BTN16 19  

// XInput定義
#define XINPUT_DPAD_UP    0x0001
#define XINPUT_DPAD_DOWN  0x0002
#define XINPUT_DPAD_LEFT  0x0004
#define XINPUT_DPAD_RIGHT 0x0008
#define XINPUT_START      0x0010
#define XINPUT_BACK       0x0020
#define XINPUT_LB         0x0100
#define XINPUT_RB         0x0200
#define XINPUT_BTN_A      0x1000
#define XINPUT_BTN_B      0x2000
#define XINPUT_BTN_X      0x4000
#define XINPUT_BTN_Y      0x8000
#define XINPUT_THUMB_L    0x0040
#define XINPUT_THUMB_R    0x0080
#define XINPUT_GUIDE      0x0400

uint8_t key_map[KEY_COUNT] = {
    ID_RIGHT, ID_DOWN, ID_LEFT, ID_BTN16, ID_BTN8, ID_BTN7, ID_BTN15, ID_BTN14, 
    ID_BTN13, ID_BTN12, ID_BTN11, ID_BTN10, ID_BTN9, ID_BTN3, ID_BTN4, ID_BTN5, 
    ID_BTN6, ID_BTN2, ID_UP, ID_BTN1
};

extern USBD_HandleTypeDef hUsbDeviceFS;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

uint16_t adc_values[KEY_COUNT];
uint16_t startup_values[KEY_COUNT];
bool key_active[KEY_COUNT];        

typedef struct { uint16_t anchor; bool is_active; } rt_state_t;
rt_state_t key_rt_states[KEY_COUNT];

typedef struct __attribute__((packed)) {
    uint8_t msg_type; uint8_t pkt_size; uint16_t buttons; uint8_t lt; uint8_t rt;
    int16_t lx; int16_t ly; int16_t rx; int16_t ry; uint8_t reserved[6]; 
} XInputReport_t;

XInputReport_t current_report, prev_report;

// WebHID送信バッファ
uint8_t webhid_tx_buf[64];

// プロトタイプ宣言
void SystemClock_Config(void);
void Error_Handler(void);
void scan_all_keys_fast(uint16_t *buffer);
bool update_key_state_rt(int key_index, uint16_t current_raw_val);
void MUX1_Select(uint8_t ch);
void MUX2_Select(uint8_t ch);
void short_delay(void);
uint8_t USBD_WebHID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);

void short_delay(void) { for(volatile int i = 0; i < 30; i++) { __NOP(); } }

void MUX1_Select(uint8_t ch) {
    HAL_GPIO_WritePin(MUX1_A_GPIO_Port, MUX1_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX1_B_GPIO_Port, MUX1_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX1_C_GPIO_Port, MUX1_C_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MUX2_Select(uint8_t ch) {
    HAL_GPIO_WritePin(MUX2_A_GPIO_Port, MUX2_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX2_B_GPIO_Port, MUX2_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX2_C_GPIO_Port, MUX2_C_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void scan_all_keys_fast(uint16_t *buffer) {
    for (int ch = 0; ch < 8; ch++) {
        MUX2_Select(ch); MUX1_Select(ch); short_delay(); 
        HAL_ADC_Start(&hadc4); HAL_ADC_Start(&hadc3); HAL_ADC_Start(&hadc2);

        if (HAL_ADC_PollForConversion(&hadc4, 1) == HAL_OK) buffer[ch] = HAL_ADC_GetValue(&hadc4);
        if (HAL_ADC_PollForConversion(&hadc3, 1) == HAL_OK) {
            uint16_t val = HAL_ADC_GetValue(&hadc3);
            if (ch < 4) buffer[ch + 8] = val; else if (ch == 7) buffer[12] = val; 
        }
        if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK) {
            uint16_t val = HAL_ADC_GetValue(&hadc2);
            switch(ch) { 
                case 7: buffer[13]=val; break; case 6: buffer[14]=val; break; 
                case 5: buffer[15]=val; break; case 4: buffer[16]=val; break; 
                case 3: buffer[17]=val; break; case 2: buffer[18]=val; break; 
                case 1: buffer[19]=val; break; 
            }
        }
    }
}

bool update_key_state_rt(int key_index, uint16_t current_raw_val) {
    rt_state_t *state = &key_rt_states[key_index];
    uint16_t base = startup_values[key_index];
    int current_depth = abs((int)current_raw_val - (int)base);
    if (current_depth < STATIC_AP_THRESHOLD) {
        state->is_active = false; state->anchor = current_depth; return false;
    }
    if (state->is_active) {
        if (current_depth > state->anchor) state->anchor = current_depth;
        else if ((state->anchor - current_depth) > RT_RELEASE_DIST) { state->is_active = false; state->anchor = current_depth; }
    } else {
        if (current_depth < state->anchor) state->anchor = current_depth;
        else if ((current_depth - state->anchor) > RT_ACTUATION_DIST) { state->is_active = true; state->anchor = current_depth; }
    }
    return state->is_active;
}

void setup(void) {
    HAL_Delay(1000); 
    for(int k=0; k<10; k++) scan_all_keys_fast(startup_values);
    for(int i=0; i<KEY_COUNT; i++) { key_active[i] = false; key_rt_states[i].is_active = false; key_rt_states[i].anchor = 0; }
    memset(&current_report, 0, sizeof(current_report)); current_report.msg_type = 0x00; current_report.pkt_size = 0x14;
    memset(&prev_report, 0, sizeof(prev_report));
    memset(webhid_tx_buf, 0, sizeof(webhid_tx_buf));
}

void loop(void) {
    scan_all_keys_fast(adc_values);
    uint16_t btns_mask = 0; uint8_t trigger_l = 0; uint8_t trigger_r = 0;
    bool up=false, down=false, left=false, right=false;

    for(int i = 0; i < KEY_COUNT; i++) {
        bool is_pressed = update_key_state_rt(i, adc_values[i]);
        key_active[i] = is_pressed; 
        if (is_pressed) {
            uint8_t id = key_map[i];
            switch(id) {
                case ID_UP: btns_mask |= XINPUT_DPAD_UP; up=true; break;
                case ID_DOWN: btns_mask |= XINPUT_DPAD_DOWN; down=true; break;
                case ID_LEFT: btns_mask |= XINPUT_DPAD_LEFT; left=true; break;
                case ID_RIGHT: btns_mask |= XINPUT_DPAD_RIGHT; right=true; break;
                case ID_BTN1: btns_mask |= XINPUT_THUMB_R; break; 
                case ID_BTN2: btns_mask |= XINPUT_THUMB_L; break;
                case ID_BTN3: btns_mask |= XINPUT_BTN_A; break; 
                case ID_BTN4: btns_mask |= XINPUT_BTN_B; break; 
                case ID_BTN5: trigger_l = 255; break; 
                case ID_BTN6: trigger_r = 255; break; 
                case ID_BTN7: btns_mask |= XINPUT_GUIDE; break; 
                case ID_BTN10: btns_mask |= XINPUT_BTN_X; break;
                case ID_BTN11: btns_mask |= XINPUT_BTN_Y; break; 
                case ID_BTN12: btns_mask |= XINPUT_LB; break; 
                case ID_BTN13: btns_mask |= XINPUT_RB; break; 
                case ID_BTN14: btns_mask |= XINPUT_START; break; 
                case ID_BTN15: btns_mask |= XINPUT_BACK; break; 
                default: break;
            }
        }
    }
    if (left && right) { left = false; right = false; }
    if (up && down) { btns_mask &= ~(XINPUT_DPAD_DOWN); btns_mask |= XINPUT_DPAD_UP; }
    current_report.buttons = btns_mask; current_report.lt = trigger_l; current_report.rt = trigger_r;

    if (memcmp(&current_report, &prev_report, sizeof(XInputReport_t)) != 0) {
        if (USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&current_report, sizeof(current_report)) == USBD_OK) {
            prev_report = current_report;
        }
    }

    memcpy(webhid_tx_buf, adc_values, sizeof(adc_values));
    USBD_WebHID_SendReport(&hUsbDeviceFS, webhid_tx_buf, 64);
}

int main(void) {
  HAL_Init(); 
  SystemClock_Config(); 
  MX_GPIO_Init(); 
  MX_ADC2_Init(); 
  MX_ADC3_Init(); 
  MX_ADC4_Init(); 
  MX_USB_DEVICE_Init();
  
  setup(); 
  while (1) { 
      loop(); 
  }
}

// ===============================================================================
// System Clock Configuration (修正: 16MHzクリスタル用)
// ===============================================================================
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  
  // ★重要: 16MHzを2で割って8MHzにしてPLLに入力
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2; 
  
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 8MHz * 9 = 72MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5; // 72MHz / 1.5 = 48MHz

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}