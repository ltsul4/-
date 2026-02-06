#include "usbd_hid.h"
#include "usbd_ctlreq.h"

// 関数プロトタイプ
static uint8_t USBD_XINPUT_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_XINPUT_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_XINPUT_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t *USBD_XINPUT_GetCfgDesc(uint16_t *length);
static uint8_t USBD_XINPUT_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_XINPUT_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

// ★追加: データ受信用バッファ (構造体メンバに依存しないように独立させる)
static uint8_t WebHID_RxBuffer[64];

// クラス構造体
USBD_ClassTypeDef USBD_HID = {
    USBD_XINPUT_Init,
    USBD_XINPUT_DeInit,
    USBD_XINPUT_Setup,
    NULL,
    NULL,
    USBD_XINPUT_DataIn,
    USBD_XINPUT_DataOut,
    NULL,
    NULL,
    NULL,
    USBD_XINPUT_GetCfgDesc, // HS
    USBD_XINPUT_GetCfgDesc, // FS
    USBD_XINPUT_GetCfgDesc, // OtherSpeed
    NULL,
};

// WebHID用レポートディスクリプタ (Raw HID 64bytes)
__ALIGN_BEGIN static uint8_t WebHID_ReportDesc[34] __ALIGN_END = {
    0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined)
    0x09, 0x01,        // Usage (0x01)
    0xA1, 0x01,        // Collection (Application)
    // Input Report
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x40, 0x09, 0x01, 0x81, 0x02,
    // Output Report
    0x95, 0x40, 0x09, 0x01, 0x91, 0x02,
    0xC0
};

// マクロ再定義回避
#ifdef USB_HID_CONFIG_DESC_SIZ
#undef USB_HID_CONFIG_DESC_SIZ
#endif
#define USB_HID_CONFIG_DESC_SIZ       88

__ALIGN_BEGIN static uint8_t USBD_XINPUT_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END = {
    0x09, 0x02, LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ), 
    0x02, 0x01, 0x00, 0x80, 0xFA, 
    // IAD (Interface 0)
    0x08, 0x0B, 0x00, 0x01, 0xFF, 0x5D, 0x01, 0x00,
    // Interface 0 (XInput)
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00,
    0x10, 0x21, 0x10, 0x01, 0x01, 0x24, 0x81, 0x14, 0x03, 0x00, 0x03, 0x13, 0x02, 0x00, 0x03, 0x00,
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x01, 
    0x07, 0x05, 0x02, 0x03, 0x20, 0x00, 0x08,
    // Interface 1 (WebHID)
    0x09, 0x04, 0x01, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00,
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(sizeof(WebHID_ReportDesc)), HIBYTE(sizeof(WebHID_ReportDesc)),
    0x07, 0x05, 0x83, 0x03, 0x40, 0x00, 0x01, 
    0x07, 0x05, 0x03, 0x03, 0x40, 0x00, 0x01
};

static uint8_t USBD_XINPUT_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    USBD_LL_OpenEP(pdev, 0x81, USBD_EP_TYPE_INTR, 32);
    USBD_LL_OpenEP(pdev, 0x02, USBD_EP_TYPE_INTR, 32);
    USBD_LL_OpenEP(pdev, 0x83, USBD_EP_TYPE_INTR, 64); // WebHID IN
    USBD_LL_OpenEP(pdev, 0x03, USBD_EP_TYPE_INTR, 64); // WebHID OUT

    pdev->pClassData = USBD_malloc(sizeof(USBD_HID_HandleTypeDef));
    
    // ★修正: 構造体のメンバを使わず、静的配列を指定して受信準備
    USBD_LL_PrepareReceive(pdev, 0x03, WebHID_RxBuffer, 64);

    return USBD_OK;
}

static uint8_t USBD_XINPUT_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    USBD_LL_CloseEP(pdev, 0x81);
    USBD_LL_CloseEP(pdev, 0x02);
    USBD_LL_CloseEP(pdev, 0x83);
    USBD_LL_CloseEP(pdev, 0x03);
    if(pdev->pClassData) USBD_free(pdev->pClassData);
    return USBD_OK;
}

static uint8_t USBD_XINPUT_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    uint16_t len = 0;
    uint8_t *pbuf = NULL;
    static uint8_t if_alt = 0;

    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
        if (req->bRequest == 0x0A) return USBD_OK; // Set_Idle
        USBD_CtlError(pdev, req);
        return USBD_FAIL;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest) {
        case USB_REQ_GET_DESCRIPTOR:
            if (req->wValue >> 8 == 0x22 && (req->wIndex & 0xFF) == 1) {
                pbuf = WebHID_ReportDesc;
                len = sizeof(WebHID_ReportDesc);
            }
            if (pbuf != NULL) {
                len = MIN(len, req->wLength);
                USBD_CtlSendData(pdev, pbuf, len);
            } else {
                USBD_CtlError(pdev, req);
                return USBD_FAIL;
            }
            break;
        case USB_REQ_GET_INTERFACE:
            if_alt = 0;
            USBD_CtlSendData(pdev, &if_alt, 1);
            break;
        case USB_REQ_SET_INTERFACE:
            USBD_CtlSendStatus(pdev);
            break;
        }
        break;
    }
    return USBD_OK;
}

static uint8_t USBD_XINPUT_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    return USBD_OK;
}

static uint8_t USBD_XINPUT_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum == 0x03) {
        // 受信データは WebHID_RxBuffer に入っています
        // ここで必要なら解析処理を行いますが、今は受信継続処理のみ行います
        
        // ★修正: 次のデータ受信準備 (静的配列を使用)
        USBD_LL_PrepareReceive(pdev, 0x03, WebHID_RxBuffer, 64);
    }
    return USBD_OK;
}

static uint8_t *USBD_XINPUT_GetCfgDesc(uint16_t *length) {
    *length = sizeof(USBD_XINPUT_CfgDesc);
    return USBD_XINPUT_CfgDesc;
}

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len) {
    return USBD_LL_Transmit(pdev, 0x81, report, len);
}

// WebHID送信関数
uint8_t USBD_WebHID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len) {
    return USBD_LL_Transmit(pdev, 0x83, report, len);
}