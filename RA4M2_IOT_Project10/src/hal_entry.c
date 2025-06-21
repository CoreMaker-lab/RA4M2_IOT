#include "hal_data.h"
#include <stdio.h>
#include "lsm6dsv16x_reg.h"
#include "oled.h"
#include "bmp.h"
#include "lps22df_reg.h"
FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

/* ==================  OLED Page Control  ================== */
static uint8_t g_oled_clear = 1;   // 1=需要清屏；0=正常增量刷新
static uint8_t g_oled_page  = 0;   // 当前页索引：0-Tuya 1-MEMS 2-Baro
static uint8_t g_tuya_num   = 0;   // 100ms 级计数器，用来节流页面刷新

/* —— Tuya（涂鸦）配网状态 —— */
static uint8_t  g_tuya_ap_mode      = 0;  // 1=处于 AP 快配；0=普通
static uint32_t g_tuya_ap_mode_num  = 0;  // AP 模式剩余倒计时 (ms)




/* UART0——涂鸦模块：单字节接收完成标志 */
volatile bool uart_wifi_RX_flag = false;
/* UART0——涂鸦模块：发送完成标志*/
volatile bool uart_wifi_TX_flag = false;
/* UART0 接收缓存大小*/
#define UART0_LENGTH  255
/* UART0 接收环形缓冲区*/
uint8_t  TUYA_wifi_buffer[UART0_LENGTH];
/* 已接收字节计数（环形缓存写指针）*/
uint32_t UART0_TUYA_LENGTH = 0;
/* 解析到一帧有效数据后置位，主循环里解析后需清零 */
uint32_t UART0_TUYA_flag = 0;

/*****************************************************************
 * UART0 回调：RX/TX/单字符事件处理
 *****************************************************************/
void user_uart_callback0(uart_callback_args_t *p_args)
{
    /* 整帧接收完成（需在 R_SCI_UART_Receive 调用后才会产生）*/
    if(p_args->event == UART_EVENT_RX_COMPLETE)
    {
        uart_wifi_RX_flag = true;
    }
    /* 发送完成 */
    else if(p_args->event == UART_EVENT_TX_COMPLETE)
    {
        uart_wifi_TX_flag = true;
    }
    /* 接收到 1 个字符*/
    else if(p_args->event == UART_EVENT_RX_CHAR)
    {
        /* 缓冲未满则写入*/
        if (sizeof(TUYA_wifi_buffer) > UART0_TUYA_LENGTH)
        {
            /* 仅在数据位 ≥ 8bit 时写入 1 字节 */
            if (UART_DATA_BITS_8 >= g_uart0_cfg.data_bits)
            {
                TUYA_wifi_buffer[UART0_TUYA_LENGTH++] = (uint8_t) p_args->data;
            }
            /* —— 以下为简单的涂鸦帧快速判定逻辑，可改为状态机 —— */
            /* 帧头非 0x55 则丢弃当前字节（回溯 1 位）*/
            if(TUYA_wifi_buffer[00]!=0x55)
                UART0_TUYA_LENGTH--;
            /* 普通指令帧长度 ≥ 7 且 CMD ≠ 0x07 时，即可认为一帧完 */
            if(UART0_TUYA_LENGTH>=7 && TUYA_wifi_buffer[3]!=0x06)
                UART0_TUYA_flag=1;
        }
    }
}

/*****************************************************************
 * 与涂鸦协议相关的工作变量／固定协议帧
 * *****************************************************************/

uint8_t wifi_first =0;/* 0：第一次心跳；1：第二次心跳     */
uint32_t wifi_num =0;//如果心跳频繁发送，可能是触发了复位，需要重新发送buff1，这里2秒内多次发送心跳指令则认为重启
/* ---- 固定格式下行帧：MCU 主动发送给涂鸦模块 ----------------- */
const uint8_t g_tuya_heartbeat1[8]={0x55,0xAA,0x03,0x00,0x00,0x01,0x00,0x03};//心跳检测,第1次 0x55 aa 00 00 00 01 00 03
const uint8_t g_tuya_heartbeat2[8]={0x55,0xAA,0x03,0x00,0x00,0x01,0x01,0x04};//心跳检测,第2次 0x55 aa 00 00 00 01 01 04
const uint8_t g_tuya_wifi_cfg[8]={0x55,0xAA,0x03,0x05,0x00,0x01,0x01,0x09};//WIFI配网
//0x55, 0xAA, 0x03, 0x01 (帧头)
//0x00, 0x2A       (长度)
//0x7B, 0x22, 0x70, 0x22, 0x3A, 0x22 ({"p":")
//0x36, 0x33, 0x70, 0x6E, 0x66, 0x69, 0x72, 0x6D, 0x72, 0x73, 0x6C, 0x78, 0x74, 0x75, 0x72, 0x38 (63pnfirmrslxtur8)
//20x22, 0x2C, 0x22, 0x76, 0x22, 0x3A, 0x22 (","v":")
//0x31, 0x2E, 0x30, 0x2E, 0x30  (1.0.0)
//0x22, 0x2C, 0x22, 0x6D, 0x22, 0x3A (","m":)
//0x30  (0)
//0x7D(})
//0x40(校验码)
uint8_t g_tuya_product_info[49]={
0x55, 0xAA, 0x03, 0x01,
0x00, 0x2A,
0x7B, 0x22, 0x70, 0x22, 0x3A, 0x22,
0x36, 0x33, 0x70, 0x6E, 0x66, 0x69, 0x72, 0x6D, 0x72, 0x73, 0x6C, 0x78, 0x74, 0x75, 0x72, 0x38,
0x22, 0x2C, 0x22, 0x76, 0x22, 0x3A, 0x22,
0x31, 0x2E, 0x30, 0x2E, 0x30,
0x22, 0x2C, 0x22, 0x6D, 0x22, 0x3A,
0x30,
0x7D,
0x40

};//接收模块发送的查询产品信息请求

const uint8_t g_tuya_query_mode[8]={0x55,0xaa,0x03,0x02,0x00,0x00,0x04};//查询工作模式
uint32_t wifi_ap_num =0;//wifi重置计时
const uint8_t g_tuya_rpt_net[8]={0x55,0xaa,0x03,0x03,0x00,0x00,0x05};//报告设备联网状态
uint32_t wifi_Update=0;//wifi发送标志位
uint32_t g_tuya_up_data=0;//定时上报计时
uint8_t g_tuya_mode_flag=0;//wifi模式，4的时候链接上网
uint8_t bat=0;//电池电量
uint8_t g_tuya_dp_bat[15]={0x55,0xAA,0x03,0x07,0x00,0x08,0x03,0x02,0x00,0x04,0x00,0x00,0x00,0x00,0x1A};//MCU上报电量

uint8_t up_down=0;//震动
uint8_t g_tuya_dp_tap[12]={0x55,0xAA,0x03,0x07,0x00,0x05,0x0A,0x04,0x00,0x01,0x00,0x1D};//MCU上报状态

uint16_t temp_tuya=0;//温度
uint8_t g_tuya_dp_temp[15]={0x55,0xAA,0x03,0x07,0x00,0x08,0x08,0x02,0x00,0x04,0x00,0x00,0x00,0x00,0x1F};//MCU上报稳定


fsp_err_t err = FSP_SUCCESS;
volatile bool uart_send_complete_flag = false;
void user_uart_callback (uart_callback_args_t * p_args)
{
    if(p_args->event == UART_EVENT_TX_COMPLETE)
    {
        uart_send_complete_flag = true;
    }
}

#ifdef __GNUC__                                 //串口重定向
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


PUTCHAR_PROTOTYPE
{
        err = R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *)&ch, 1);
        if(FSP_SUCCESS != err) __BKPT();
        while(uart_send_complete_flag == false){}
        uart_send_complete_flag = false;
        return ch;
}

int _write(int fd,char *pBuffer,int size)
{
    for(int i=0;i<size;i++)
    {
        __io_putchar(*pBuffer++);
    }
    return size;
}

/* Callback function */
i2c_master_event_t i2c_event = I2C_MASTER_EVENT_ABORTED;
uint32_t  timeout_ms = 100000;
void sci_i2c_master_callback(i2c_master_callback_args_t *p_args)
{
    i2c_event = I2C_MASTER_EVENT_ABORTED;
    if (NULL != p_args)
    {
        /* capture callback event for validating the i2c transfer event*/
        i2c_event = p_args->event;
    }
}

#define SENSOR_BUS g_i2c2_ctrl

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static lsm6dsv16x_interrupt_mode_t irq;
static lsm6dsv16x_tap_detection_t tap;
static lsm6dsv16x_tap_thresholds_t tap_ths;
static lsm6dsv16x_tap_time_windows_t tap_win;

/* Extern variables ----------------------------------------------------------*/
lsm6dsv16x_all_sources_t status;
lsm6dsv16x_pin_int_route_t pin_int;
lsm6dsv16x_reset_t rst;
/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void *handle);

static stmdev_ctx_t dev_ctx;
static uint8_t stap_event_catched = 0;
static uint8_t dtap_event_catched = 0;

bool irq_flag =0;
/* Called from icu_irq_isr */
void external_irq6_callback (external_irq_callback_args_t * p_args)
{
    (void) p_args;
    irq_flag = 1;
}

static void sensor_lsm6dsv16x_tap_init(void)
{
    // 配置传感器设备操作函数（读、写、延时），并指定 I2C 句柄
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &SENSOR_BUS;
    // 上电后延时，等待传感器稳定
    platform_delay(BOOT_TIME);
    // 读取芯片 ID，并打印，用于识别传感器是否连接正确
    lsm6dsv16x_device_id_get(&dev_ctx, &whoamI);
    printf("LSM6DSV16X_ID=0x%x, whoamI=0x%x\n", LSM6DSV16X_ID, whoamI);
    if (whoamI != LSM6DSV16X_ID) while (1);// 若芯片 ID 不匹配，则停留在此处
    // 若芯片 ID 不匹配，则停留在此处
    lsm6dsv16x_reset_set(&dev_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
    // 等待复位完成
    do {
        lsm6dsv16x_reset_get(&dev_ctx, &rst);
    } while (rst != LSM6DSV16X_READY);
    // 使能 Block Data Update，防止数据在读取时被更新
    lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    // 配置将“单击”和“双击”事件路由至 INT1 引脚
    pin_int.double_tap = PROPERTY_ENABLE;
    pin_int.single_tap = PROPERTY_ENABLE;
    lsm6dsv16x_pin_int1_route_set(&dev_ctx, &pin_int);
    // 配置中断输出方式：使能功能中断，关闭锁存模式（脉冲输出）
    irq.enable = 1;
    irq.lir = 0;
    lsm6dsv16x_interrupt_enable_set(&dev_ctx, irq);
    // 启用 Z 轴 Tap 检测（仅检测 Z 轴）
    tap.tap_z_en = 1;
    lsm6dsv16x_tap_detection_set(&dev_ctx, tap);
    // 设置 Tap 阈值，单位为 LSB（3 LSB ≈ 186mg @ ±8g）
    tap_ths.z = 3;
    lsm6dsv16x_tap_thresholds_set(&dev_ctx, tap_ths);
    // 设置 Tap 时间窗口（用于识别单击/双击时的时间控制）
    tap_win.tap_gap = 7;
    tap_win.shock = 3;
    tap_win.quiet = 3;
    lsm6dsv16x_tap_time_windows_set(&dev_ctx, tap_win);
    // 启用双击和单击检测模式
    lsm6dsv16x_tap_mode_set(&dev_ctx, LSM6DSV16X_BOTH_SINGLE_DOUBLE);
    // 设置加速度计工作参数：480Hz 采样率
    lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_480Hz);
    // 设置加速度计量程为 ±8g（灵敏度和范围的折中）
    lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_8g);
}

static void app_peripheral_init(void)
{
    // 点亮板载 LED（用于上电指示）
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_04, BSP_IO_LEVEL_HIGH);
    //LIS2MDL CS2->1
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_00_PIN_02, BSP_IO_LEVEL_HIGH);
    //LSM6DSV16X CS1->1
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_02, BSP_IO_LEVEL_HIGH);

    //LPS22DF CS->1
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_05_PIN_00, BSP_IO_LEVEL_HIGH);
    //LPS22DF SA0->0
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_00_PIN_13, BSP_IO_LEVEL_LOW);

    // 初始化串口 UART9
    err = R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    assert(FSP_SUCCESS == err);
    // 发送初始字符串用于串口测试
    char *msg = "RA E2STUDIO";
    err = R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *)msg, strlen(msg));
    if (FSP_SUCCESS != err) __BKPT();
    while (!uart_send_complete_flag) {}
    uart_send_complete_flag = false;
    // 打印欢迎信息
    printf("\nhello world!\r\n");
    // 初始化 I2C 控制器
    err = R_SCI_I2C_Open(&g_i2c2_ctrl, &g_i2c2_cfg);
    assert(FSP_SUCCESS == err);
    // 打开外部中断通道（用于接收 LSM6DSV16X 的中断输出）
    err = R_ICU_ExternalIrqOpen(&g_external_irq6_ctrl, &g_external_irq6_cfg);
    assert(FSP_SUCCESS == err);
    // 使能外部中断通道
    err = R_ICU_ExternalIrqEnable(&g_external_irq6_ctrl);
    assert(FSP_SUCCESS == err);

    // 初始化串口 UART0
    err = R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg);
    assert(FSP_SUCCESS == err);

    // 打开外部中断通道（用于接收 LPS22DF 的中断输出）
    fsp_err_t err = R_ICU_ExternalIrqOpen(&g_external_irq4_ctrl, &g_external_irq4_cfg);
    assert(FSP_SUCCESS == err);

    err = R_ICU_ExternalIrqEnable(&g_external_irq4_ctrl);
    assert(FSP_SUCCESS == err);

}

// Tap 检测主循环函数：用于持续检测并打印单击/双击事件
static void sensor_lsm6dsv16x_tap_loop(void)
{
    // 若外部中断触发，读取传感器中断源
    if (irq_flag)
    {
        wifi_Update=1;
        g_tuya_up_data=2000;
        lsm6dsv16x_all_sources_get(&dev_ctx, &status);
        irq_flag = false;

        if (status.single_tap)
            stap_event_catched = 1;

        if (status.double_tap)
            dtap_event_catched = 1;
        }

        // 若捕获到单击事件，处理并打印
        if (stap_event_catched)
        {
            stap_event_catched = 0;
            up_down=1;
            printf("Single TAP\r\n");
        }

        // 若捕获到双击事件，处理并打印
        if (dtap_event_catched)
        {
            dtap_event_catched = 0;
            up_down=2;
            printf("Double TAP\r\n");
        }
}

/*******************************************************************************************************************
 * @brief 向涂鸦模块上报最新 DP 数据 & 本地硬件同步
 *******************************************************************************************************************/
static void tuya_wifi_Update(void)
{
    if(wifi_Update)
    {
        printf("wifi_Update\n");
        wifi_Update=0;

        g_tuya_dp_tap[10]=up_down;//状态
        g_tuya_dp_tap[11]=0;//校验和
        for(int i=0;i<=10;i++)
            g_tuya_dp_tap[11]+=g_tuya_dp_tap[i];
        err = R_SCI_UART_Write(&g_uart0_ctrl, g_tuya_dp_tap, 12);
        if(FSP_SUCCESS != err) __BKPT();
        while(uart_wifi_TX_flag == false){}
        uart_wifi_TX_flag = false;

    }

}

/*******************************************************************************************************************
 * @brief 长按按键 3 s 进入配网模式 (发送 0x05 命令)
 *******************************************************************************************************************/
static void button_wifi_ap(void)
{
    //  wifi_ap_num
    bsp_io_level_t p_port_value_pin_111;
    R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_11, &p_port_value_pin_111);
    if(p_port_value_pin_111)/* 松开 */
        wifi_ap_num=0;
    else//长按3s复位wifi
    {
        if(wifi_ap_num<3000)
            wifi_ap_num++;
        else if(wifi_ap_num==3000)
        {
            g_oled_clear=1;// 切屏前清屏
            g_oled_page=0;// 0-Tuya 1-MEMS 2-Baro

            wifi_ap_num++;
            printf("[BTN] wifi_ap_mode\r\n");
            fsp_err_t err = R_SCI_UART_Write(&g_uart0_ctrl, g_tuya_wifi_cfg, 8);
            if(FSP_SUCCESS != err) __BKPT();
            while(uart_wifi_TX_flag == false){}
            uart_wifi_TX_flag = false;
        }

        if(wifi_ap_num==100)//轻触切换OLED显示
        {
            g_oled_clear=1;// 下一帧先清屏
            if(g_oled_page<2)// 0-Tuya 1-MEMS 2-Baro
                g_oled_page++;
            else
                g_oled_page=0;

        }

    }
}

/*******************************************************************************************************************
 * @brief 解析完成后，根据 CMD 打印清晰友好的日志
 *******************************************************************************************************************/
static void uart0_tuya(void)
{
    if(UART0_TUYA_flag ==1)//接收完成标志
    {

        fsp_err_t err = FSP_SUCCESS;
        UART0_TUYA_flag=0;
        if(TUYA_wifi_buffer[0]==0x55&&TUYA_wifi_buffer[1]==0xAA)//判断帧头和版本
        {
            if(TUYA_wifi_buffer[3]==0x00)//判断是否为心跳检测
            {
                printf("[TUYA] <heartbeat(SEQ=%u)\r\n", wifi_first);
//                if(wifi_num<2000&&wifi_first==1)//频繁发送心跳指令，认为重启
//                {
//                    wifi_first=0;
//                   }
//                wifi_num=0;
                if(wifi_first==0)//第一次发送心跳数据
                {
                    wifi_first=1;
                    //心跳检测，向涂鸦模块发送
                    err = R_SCI_UART_Write(&g_uart0_ctrl, g_tuya_heartbeat1, 8);
                    if(FSP_SUCCESS != err) __BKPT();
                    while(uart_wifi_TX_flag == false){}
                    uart_wifi_TX_flag = false;
                    }
                else
                {
                    //心跳检测，向涂鸦模块发送
                    err = R_SCI_UART_Write(&g_uart0_ctrl, g_tuya_heartbeat2, 8);
                    if(FSP_SUCCESS != err) __BKPT();
                    while(uart_wifi_TX_flag == false){}
                    uart_wifi_TX_flag = false;
                }
            }
            else if(TUYA_wifi_buffer[3]==0x01)//接收模块发送的查询产品信息请求
            {
                printf("[TUYA] <Query Product Information\r\n");
                //接收模块发送的查询产品信息请求，向涂鸦模块发送
                err = R_SCI_UART_Write(&g_uart0_ctrl, g_tuya_product_info, 49);
                if(FSP_SUCCESS != err) __BKPT();
                while(uart_wifi_TX_flag == false){}
                uart_wifi_TX_flag = false;
            }
            else if(TUYA_wifi_buffer[3]==0x02)//查询工作模式
            {
                printf("[TUYA] <Query Work Mode\r\n");
                //查询工作模式，向涂鸦模块发送
                err = R_SCI_UART_Write(&g_uart0_ctrl, g_tuya_query_mode, 8);
                if(FSP_SUCCESS != err) __BKPT();
                while(uart_wifi_TX_flag == false){}
                uart_wifi_TX_flag = false;
            }
            else if(TUYA_wifi_buffer[3]==0x03)//报告设备联网状态
            {
                g_tuya_mode_flag=TUYA_wifi_buffer[6];//联网状态
                printf("[TUYA] <WIFI_MODE=%02X\r\n", TUYA_wifi_buffer[6]);
                //查询工作模式，向涂鸦模块发送
                err = R_SCI_UART_Write(&g_uart0_ctrl, g_tuya_rpt_net, 8);
                if(FSP_SUCCESS != err) __BKPT();
                while(uart_wifi_TX_flag == false){}
                uart_wifi_TX_flag = false;

                if(TUYA_wifi_buffer[6]==0x04)//已连上路由器且连接到云端
                {
                    wifi_Update=1;//wifi跟新标志位
                }
            }
        }
        // 清除数组
        memset(TUYA_wifi_buffer, 0, UART0_LENGTH);
        // 同时把当前有效长度归零
        UART0_TUYA_LENGTH = 0;
    }
}

// 涂鸦数据上传函数：每 2000ms 上传一次电量和温度信息给涂鸦模块
static void tuya_up_data(void)
{
    // 当处于涂鸦工作模式 4（表示允许上传数据）
    if(g_tuya_mode_flag==4)
    {
        // g_tuya_up_data 每加一次表示延时 1ms（由定时器或主循环每 ms 调用此函数）
        if(g_tuya_up_data<2000)
            g_tuya_up_data++;// 计时增加
        else
        {
            g_tuya_up_data=0;// 超过 2000ms，开始上传数据
            /* ---------------- 电量数据上传 ---------------- */
            bat = 87;  // 模拟电量百分比（实际项目中建议从 ADC 获取真实电压）
            g_tuya_dp_bat[13] = bat;     // 将电量值填入数据帧第14字节
            g_tuya_dp_bat[14] = 0;       // 清空校验和
            // 计算校验和（累加前13个字节）
            for(int i=0;i<=13;i++)
                g_tuya_dp_bat[14]+=g_tuya_dp_bat[i];
            // 串口打印电量信息，供调试查看
            printf("[MCU] -> TUYA DP_BAT = %d\r\n",bat);
            // 通过 UART 发送电量数据给涂鸦 Wi-Fi 模块
            err = R_SCI_UART_Write(&g_uart0_ctrl, g_tuya_dp_bat, 15);
            if(FSP_SUCCESS != err) __BKPT();
            while(uart_wifi_TX_flag == false){}
            uart_wifi_TX_flag = false;

            /* ---------------- 温度数据上传 ---------------- */
            int16_t data_raw_temperature;
            double_t temperature_degC;
            /* Read temperature data */
              memset(&data_raw_temperature, 0x00, sizeof(int16_t));// 清空原始数据缓存
              // 读取 LSM6DSV16X 的原始温度数据
              lsm6dsv16x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
              // 将原始 LSB 数据转换为摄氏温度
              temperature_degC = lsm6dsv16x_from_lsb_to_celsius(
                                   data_raw_temperature);
              // 扩大10倍为整数（单位 0.1°C）方便打包进两个字节
              temp_tuya = temperature_degC * 10;

              g_tuya_dp_temp[12] = temp_tuya / 255;  // 高字节
              g_tuya_dp_temp[13] = temp_tuya % 255;  // 低字节

              g_tuya_dp_temp[14] = 0;  // 清空校验和
              // 计算校验和
              for(int i=0;i<=13;i++)
                  g_tuya_dp_temp[14]+=g_tuya_dp_temp[i];
              // 串口打印温度信息
              printf("[MCU] -> TUYA DP_tEMP = %6.2f\r\n",temperature_degC);
              // 通过 UART 发送温度数据给涂鸦模块
              err = R_SCI_UART_Write(&g_uart0_ctrl, g_tuya_dp_temp, 15);
              if(FSP_SUCCESS != err) __BKPT();
              while(uart_wifi_TX_flag == false){}
              uart_wifi_TX_flag = false;

//              printf("Temperature [degC]:%6.2f\r\n", temperature_degC);
        }
    }
}

// OLED 屏幕启动函数：完成初始化、清屏、颜色/方向配置
static void OLED_Start(void)
{
    OLED_Init();              // 初始化 OLED 屏幕（发送初始化命令序列，设置工作模式）
    OLED_Clear();             // 清空显存（OLED_GRAM），并刷新，使屏幕全黑
    OLED_ColorTurn(0);        // 设置显示颜色模式：0 为正常显示，1 为反色显示（黑白反转）
    OLED_DisplayTurn(0);      // 设置显示方向：0 为正常方向，1 为上下翻转显示

    OLED_Clear();//清空 OLED 显存
    OLED_ShowChinese(0,0,0,16,1);//在坐标 (0,0) 显示 Hzk1[0]，即汉字“瑞”，正显
    OLED_ShowChinese(16,0,1,16,1);//在坐标 (16,0) 显示 Hzk1[1]，即汉字“萨”，正显
    OLED_ShowChinese(32,0,2,16,0);//在坐标 (32,0) 显示 Hzk1[2]，即汉字“单”，反色
    OLED_ShowChinese(48,0,3,16,0);//在坐标 (48,0) 显示 Hzk1[3]，即汉字“片”，反色
    OLED_ShowChinese(64,0,4,16,0);//在坐标 (64,0) 显示 Hzk1[4]，即汉字“机”，反色
    OLED_Refresh();//将上述通过 OLED_ShowChinese() 写入 OLED_GRAM[][] 的内容，批量发送到 OLED 屏幕显示。如果没有调用这个函数，屏幕上不会出现任何内容（即写入显存但未更新）。

    OLED_ShowNum(0, 16, 20250615, 8, 16, 1);//在坐标 (0,16) 显示20250615，长度8，字体16，正显
    OLED_ShowString(84, 16, "RA4M2", 16, 1);//在坐标 (84,16) 显示RA4M2，字体16，正显
    OLED_Refresh();// 将 OLED_GRAM 显存内容刷新到 OLED 屏幕上，显示数字和字符串

    OLED_ShowPicture(0, 32, 117, 19, BMP3, 1);                // 在 OLED 屏幕的 (x=0, y=32) 位置显示一张宽 117、高 19 像素的图片 BMP3，正常模式显示
    OLED_Refresh();                                           // 将图片内容从缓冲区刷新到 OLED 屏幕上，真正显示出来
    R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);  // 延时 200 毫秒
}


static void OLED_switch(void)
{
    if (g_oled_clear)                     // ← 仅当需切屏时进入
    {
        g_oled_clear = 0;                 // 标记已完成清屏
        OLED_Clear();                     // ① 清显存并刷新，黑场一次

        switch (g_oled_page)              // ② 根据当前页写“标签”
        {
            /* ---------- Page-0 : Tuya 模组 ---------- */
            case 0:
                OLED_ShowString(0,  0, (u8 *)"TUYA",       16,1);
                OLED_ShowString(0, 16, (u8 *)"WIFI MODE:", 16,1);
                OLED_ShowString(0, 32, (u8 *)"AP MODE:",   16,1);
                break;

            /* ---------- Page-1 : LSM6DSV16X ---------- */
            case 1:
                OLED_ShowString(0,  0, (u8 *)"MEMS_LSM6DSV16X", 12,1);
                OLED_ShowString(0, 12, (u8 *)"TAP:",   12,1);
                OLED_ShowString(60,12, (u8 *)"TEMP:",  12,1);
                OLED_ShowString(0, 24, (u8 *)"X(mg):", 12,1);
                OLED_ShowString(0, 36, (u8 *)"Y(mg):", 12,1);
                OLED_ShowString(0, 48, (u8 *)"Z(mg):", 12,1);
                break;

            /* ---------- Page-2 : LPS22DF 气压计 ------- */
            case 2:
                OLED_ShowString(0,  0, (u8 *)"MEMS_LPS22DF", 16,1);
                OLED_ShowString(0, 16, (u8 *)"Pressure:", 16,1);
                OLED_ShowString(0, 32, (u8 *)"Temp:",   16,1);
                break;
        }
        OLED_Refresh();                   // ③ 推送骨架到屏幕
    }
}

/* ------------ Page-0 : TUYA 模组信息 -------------------------------- */

static void OLED_DrawPage_TUYA(void)
{
    /* ① 管理倒计时 ── 在线时清零；否则每 1 ms 递减 */
    if (g_tuya_mode_flag == 4)                // 4 = 已连云
        g_tuya_ap_mode_num = 0;
    else if (g_tuya_ap_mode_num > 0)
        g_tuya_ap_mode_num--;

    /* ② 100 ms 节流：g_tuya_num 递增至 100 再刷新一次 */
    if (++g_tuya_num >= 100)
    {
        g_tuya_num = 0;

        /* --- 行1: AP/normal --- */
        if (g_tuya_ap_mode_num > 0)           // 倒计时仍在 → AP
            OLED_ShowString(80,16,(u8 *)"AP    ",16,1);
        else                                  // 否则 normal
            OLED_ShowString(80,16,(u8 *)"normal",16,1);

        /* --- 行2: Wi-Fi work-mode 数字标志 --- */
        OLED_ShowNum(80, 32, g_tuya_mode_flag, 1, 16,1);

        OLED_Refresh();                       // 推送局部变更
    }
}

/* ------------ Page-1 : MEMS_LSM6DSV16X -------------------------- */
static void OLED_DrawPage_MEMS(void)
{
    if(g_tuya_num<100)
    g_tuya_num++;
    else
    {
        g_tuya_num=0;
        /* ---------- 1. TAP 状态 ---------- */
        if(up_down==0)
            OLED_ShowString(24, 12, "Normal", 12, 1);//在坐标 (24,12)
        else if(up_down==1)
            OLED_ShowString(24, 12, "Single", 12, 1);//在坐标 (24,12)
        else if(up_down==2)
            OLED_ShowString(24, 12, "Double", 12, 1);//在坐标 (24,12)
        /* ---------- 2. 读取并显示片内温度 ---------- */
        int16_t data_raw_temperature;
        double_t temperature_degC;
        /* Read temperature data */
        memset(&data_raw_temperature, 0x00, sizeof(int16_t));// 清空原始数据缓存
        // 读取 LSM6DSV16X 的原始温度数据
        lsm6dsv16x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
        // 将原始 LSB 数据转换为摄氏温度
        temperature_degC = lsm6dsv16x_from_lsb_to_celsius(
                                 data_raw_temperature);
        OLED_ShowNum  (90, 12,(uint32_t)temperature_degC, 3, 12, 1);// 整数位
        OLED_ShowChar (110, 12, '.',12, 1);
        uint32_t t100 = (uint32_t)(temperature_degC * 100);
        OLED_ShowNum  (116, 12,(uint32_t)t100%100, 2, 12, 1);// 整数位

        /* ---------- 3. 读取并显示 X/Y/Z 加速度 ---------- */
        int16_t data_raw_acceleration[3];
        double_t acceleration_mg[3];
        lsm6dsv16x_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
        acceleration_mg[0] =
                lsm6dsv16x_from_fs8_to_mg(data_raw_acceleration[0]);
        acceleration_mg[1] =
                lsm6dsv16x_from_fs8_to_mg(data_raw_acceleration[1]);
        acceleration_mg[2] =
                lsm6dsv16x_from_fs8_to_mg(data_raw_acceleration[2]);

        if(acceleration_mg[0]<0)
            OLED_ShowChar (36, 24, '-',12, 1);
        else
            OLED_ShowChar (36, 24, '+',12, 1);
        OLED_ShowNum (42, 24,(uint32_t)fabs(acceleration_mg[0]),5,12, 1);
        OLED_ShowChar (72, 24, '.',12, 1);
        t100=(uint32_t)(fabs(acceleration_mg[0])*100);
        OLED_ShowNum (78, 24,(uint32_t)t100%100,2,12, 1);

        if(acceleration_mg[1]<0)
            OLED_ShowChar (36, 36, '-',12, 1);
        else
            OLED_ShowChar (36, 36, '+',12, 1);
        OLED_ShowNum (42, 36,(uint32_t)fabs(acceleration_mg[1]),5,12, 1);
        OLED_ShowChar (72, 36, '.',12, 1);
        t100=(uint32_t)(fabs(acceleration_mg[1])*100);
        OLED_ShowNum (78, 36,(uint32_t)t100%100,2,12, 1);

        if(acceleration_mg[2]<0)
            OLED_ShowChar (36, 48, '-',12, 1);
        else
            OLED_ShowChar (36, 48, '+',12, 1);
        OLED_ShowNum (42, 48,(uint32_t)fabs(acceleration_mg[2]),5,12, 1);
        OLED_ShowChar (72, 48, '.',12, 1);
        t100=(uint32_t)(fabs(acceleration_mg[2])*100);
        OLED_ShowNum (78, 48,(uint32_t)t100%100,2,12, 1);
        OLED_Refresh();
//        printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
//                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);

    }
}

bool lps22df_irq_flag =0;
/* Called from icu_irq_isr */
void external_irq4_callback (external_irq_callback_args_t * p_args)
{
    (void) p_args;
    lps22df_irq_flag = 1;
}


/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write_lps22df(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read_lps22df(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com_lps22df( uint8_t *tx_buffer, uint16_t len );
static void platform_delay_lps22df(uint32_t ms);
static void platform_init_lps22df(void);
stmdev_ctx_t            dev_ctx_LPS22DF; // 设备上下文



/* ---------------------------------------------------------------------------
 * @brief  初始化 LPS22DF 数字气压计（气压 + 片内温度）
 *         - 共用 I²C 总线（SENSOR_BUS）
 *         - 配置为 10 Hz / 256 次平均 / ODR÷9 低通
 *         - 打开 DRDY 中断，MCU 可用外部中断捕获新数据
 * --------------------------------------------------------------------------*/
static void sensor_lps22df_init(void)
{
    /* 1. 局部缓冲/结构体 -------------------------------------------------- */
    lps22df_pin_int_route_t int_route;     // 中断路由寄存器镜像
    lps22df_bus_mode_t      bus_mode;      // 接口&滤波选项
    lps22df_id_t            id;            // WHO_AM_I 读取结果
    lps22df_md_t            md;            // 模式配置
    int                     ret;           // ST 驱动函数返回值

    /* 2. 绑定底层读/写/延时 ------------------------------------------------ */
    dev_ctx_LPS22DF.write_reg = platform_write_lps22df;  // I²C 写函数
    dev_ctx_LPS22DF.read_reg  = platform_read_lps22df;   // I²C 读函数
    dev_ctx_LPS22DF.mdelay    = platform_delay_lps22df;  // 毫秒延时
    dev_ctx_LPS22DF.handle    = &SENSOR_BUS;             // I²C 句柄

    /* 3. 读取并校验设备 ID ------------------------------------------------- */
    lps22df_id_get(&dev_ctx_LPS22DF, &id);
    printf("LPS22DF_ID=0x%x, whoamI=0x%x\r\n", LPS22DF_ID, id.whoami);
    if (id.whoami != LPS22DF_ID)   // 若连线或地址错误，停在此处
        while (1);

    /* 4. 先 Boot -> 再软复位 ------------------------------------------------ */
    ret = lps22df_init_set(&dev_ctx_LPS22DF, LPS22DF_BOOT);   if (ret) while (1);
    ret = lps22df_init_set(&dev_ctx_LPS22DF, LPS22DF_RESET);  if (ret) while (1);

    /* 5. 启用驱动推荐配置：BDU=1 / IF_INC=1 ------------------------------- */
    lps22df_init_set(&dev_ctx_LPS22DF, LPS22DF_DRV_RDY);

    /* 6. 选择接口 + 滤波模式 ---------------------------------------------- */
    bus_mode.filter    = LPS22DF_FILTER_AUTO;  // 写寄存器自动关 LPF，之后再恢复
    bus_mode.interface = LPS22DF_SEL_BY_HW;    // 由 SEL 脚决定 I²C / SPI
    lps22df_bus_mode_set(&dev_ctx_LPS22DF, &bus_mode);

    /* 7. 设置测量输出速率 & 滤波/平均参数 --------------------------------- */
    md.odr = LPS22DF_10Hz;            // ODR = 10 Hz
    md.avg = LPS22DF_256_AVG;         // 256 次平均，降低噪声
    md.lpf = LPS22DF_LPF_ODR_DIV_9;   // 低通截止 ≈ 1.1 Hz
    lps22df_mode_set(&dev_ctx_LPS22DF, &md);

    /* 8. 使能气压/温度 DRDY 中断 ------------------------------------------- */
    lps22df_pin_int_route_get(&dev_ctx_LPS22DF, &int_route); // 读默认
    int_route.drdy_pres = PROPERTY_ENABLE;   // 数据就绪 → INT/DRDY 脚
    lps22df_pin_int_route_set(&dev_ctx_LPS22DF, &int_route);

    /* 现在 LPS22DF 已在 10 Hz 低噪声模式开始工作，可在主循环
       通过中断或轮询读取气压 (hPa) 与温度 (°C)。*/
  }


/******************************************************************************
 * @brief  通过 DRDY 中断读取 LPS22DF 气压计数据
 *         - 由外部中断服务程序置位 lps22df_irq_flag
 *         - 进入后读取最新气压 / 温度（若数据已就绪）
 *         - 同时触发涂鸦 DP 上报计时器
 ******************************************************************************/
double lps22df_data_pressure=0.0f;
double lps22df_data_temp=0.0f;
static void lps22df_read_data_drdy(void)
{
    /* -------- 1. 检测由 EXTI 产生的数据就绪标志 ------------------ */
    if (lps22df_irq_flag)                    // INT/DRDY 低电平到来
    {
        lps22df_irq_flag = false;            // 先清除本地标志

        /* -------- 2. 触发涂鸦数据上报计时 ----------------------- */
//        wifi_Update   = 1;                   // 下轮主循环立刻刷新 DP
//        g_tuya_up_data = 2000;               // 2 s 后再次上报 (计数器复位)

        /* -------- 3. 查询 LPS22DF 数据就绪标志 ------------------ */
        lps22df_all_sources_t all_src;
        lps22df_all_sources_get(&dev_ctx_LPS22DF, &all_src);

        /* -------- 4. 仅在有新数据时读取压强/温度 ---------------- */
        if (all_src.drdy_pres || all_src.drdy_temp)
        {
            static lps22df_data_t data;      // 静态减少栈开销
            lps22df_data_get(&dev_ctx_LPS22DF, &data);
            lps22df_data_pressure = data.pressure.hpa;
            lps22df_data_temp = data.heat.deg_c;
//            printf("pressure [hPa]: %6.2f  temperature [degC]: %6.2f\r\n",
//                   data.pressure.hpa, data.heat.deg_c);
            /* 此处可再把 data 填入 OLED/Page-2 或涂鸦 DP */
        }
    }
}


/* ------------ Page-2 : MEMS_lps22df -------------------------- */
static void OLED_lps22df_MEMS(void)
{
    /* ---------- 1. 节流判断：每 1 ms 主循环 +1，满 100 ≈ 100 ms ---------- */
    if(g_tuya_num<100)
    g_tuya_num++;
    else
    {
        g_tuya_num=0;
        uint32_t t100 = 0;// 小数两位 ×100 的临时变量
        OLED_ShowNum  (72, 16,(uint32_t)lps22df_data_pressure, 4, 16, 1);// 整数位
        /*   小数点 '.'       */
        OLED_ShowChar (104, 16, '.',16, 1);
        /*   小数部分：xx     */
        t100 = (uint32_t)(lps22df_data_pressure * 100);
        OLED_ShowNum  (112, 16,(uint32_t)t100%100, 2, 16, 1);

        /*   正负号 */
        if(lps22df_data_temp<0)
            OLED_ShowChar (72, 32, '-',16, 1);
        else
            OLED_ShowChar (72, 32, '+',16, 1);
        OLED_ShowNum (80, 32,(uint32_t)fabs(lps22df_data_temp),3,16, 1);// 整数位
        /*   小数点 '.'       */
        OLED_ShowChar (104, 32, '.',16, 1);
        /*   小数部分：xx     */
        t100=(uint32_t)(fabs(lps22df_data_temp)*100);
        OLED_ShowNum (112, 32,(uint32_t)t100%100,2,16, 1);
        OLED_Refresh();
    }
}

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    /* TODO: add your own code here */

    // 初始化外设（如 UART、I2C、中断引脚、GPIO 等）
    app_peripheral_init();
    // 初始化 LSM6DSV16X 传感器，配置单击/双击 Tap 检测功能
    sensor_lsm6dsv16x_tap_init();
    // 初始化 OLED,保证屏幕点亮且处于默认显示状态
    OLED_Start();
    // 初始化 LPS22DF 传感器，配置速率和模式
    sensor_lps22df_init();
    while (1)
    {

        //切屏完成后不再清屏，保持增量刷新，避免闪烁
        OLED_switch();
        if(g_oled_page==0) /* Page-0 : Tuya 状态页 */
            OLED_DrawPage_TUYA();// 显示 Wi-Fi / AP 配网信息
        else if(g_oled_page==1)/* Page-1 : LSM6DSV16X 状态页 */
            OLED_DrawPage_MEMS();// 显示单/双击、温度、XYZ 加速度
        else if(g_oled_page==2)/* Page-2 : LPS22DF 状态页 */
            OLED_lps22df_MEMS();// 显示气压、温度

        // 处理 LPS22DF事件，（例如气压，温度）
        lps22df_read_data_drdy();

        // 处理 LSM6DSV16X 的 Tap 检测事件（单击/双击），并串口打印
        sensor_lsm6dsv16x_tap_loop();

        // 处理涂鸦协议状态更新（例如心跳、连接状态等）
        tuya_wifi_Update();

        // 检查按键是否触发 Wi-Fi 配网模式（例如长按按键进入配网）
        button_wifi_ap();

        // 解析 UART 接收的涂鸦协议数据帧（如收到APP指令等）
        uart0_tuya();

        // 每 2000ms 上传一次传感器数据（电量、温度）到涂鸦模块
        tuya_up_data();

        // 软件延时 1ms，控制循环节奏（相当于每 1ms 执行一次）
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
    }
#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_lps22df(void *handle, uint8_t reg, const uint8_t *bufp,uint16_t len)
{
    R_SCI_I2C_SlaveAddressSet(&g_i2c2_ctrl, 0x5C, I2C_MASTER_ADDR_MODE_7BIT);
    assert(FSP_SUCCESS == err);
    // 创建一个足够大的缓冲区来包含寄存器地址和数据
    uint8_t data[len + 1];
    data[0] = reg; // 将寄存器地址放在数据的开始
    memcpy(&data[1], bufp, len); // 复制数据到缓冲区

    err = R_SCI_I2C_Write(&g_i2c2_ctrl, data, len+1, true);
    assert(FSP_SUCCESS == err);
    /* Since there is nothing else to do, block until Callback triggers*/
    //while ((I2C_MASTER_EVENT_TX_COMPLETE != i2c_event) && timeout_ms)
    while ((I2C_MASTER_EVENT_TX_COMPLETE != i2c_event) && timeout_ms>0)
    {
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MICROSECONDS);
        timeout_ms--;
        }
    if (I2C_MASTER_EVENT_ABORTED == i2c_event)
    {
        __BKPT(0);
    }
    /* Read data back from the I2C slave */
    i2c_event = I2C_MASTER_EVENT_ABORTED;
    timeout_ms           = 100000;
    return 0;
}


/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_lps22df(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
    R_SCI_I2C_SlaveAddressSet(&g_i2c2_ctrl, 0x5C, I2C_MASTER_ADDR_MODE_7BIT);
    assert(FSP_SUCCESS == err);
    err = R_SCI_I2C_Write(&g_i2c2_ctrl, &reg, 1, true);
    assert(FSP_SUCCESS == err);
    /* Since there is nothing else to do, block until Callback triggers*/
    //while ((I2C_MASTER_EVENT_TX_COMPLETE != i2c_event) && timeout_ms)
    while ((I2C_MASTER_EVENT_TX_COMPLETE != i2c_event) && timeout_ms>0)
    {
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MICROSECONDS);
        timeout_ms--;
        }
    if (I2C_MASTER_EVENT_ABORTED == i2c_event)
    {
        __BKPT(0);
        }
    /* Read data back from the I2C slave */
    i2c_event = I2C_MASTER_EVENT_ABORTED;
    timeout_ms           = 100000;

    /* Read data from I2C slave */
    err = R_SCI_I2C_Read(&g_i2c2_ctrl, bufp, len, false);
    assert(FSP_SUCCESS == err);
    while ((I2C_MASTER_EVENT_RX_COMPLETE != i2c_event) && timeout_ms)
    {
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
        timeout_ms--;
    }
    if (I2C_MASTER_EVENT_ABORTED == i2c_event)
    {
        __BKPT(0);
    }

    i2c_event = I2C_MASTER_EVENT_ABORTED;
    timeout_ms           = 100000;
  return 0;
}


/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay_lps22df(uint32_t ms)
{
    R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,uint16_t len)
{
    R_SCI_I2C_SlaveAddressSet(&g_i2c2_ctrl, 0x6B, I2C_MASTER_ADDR_MODE_7BIT);
    assert(FSP_SUCCESS == err);
    // 创建一个足够大的缓冲区来包含寄存器地址和数据
    uint8_t data[len + 1];
    data[0] = reg; // 将寄存器地址放在数据的开始
    memcpy(&data[1], bufp, len); // 复制数据到缓冲区

    err = R_SCI_I2C_Write(&g_i2c2_ctrl, data, len+1, true);
    assert(FSP_SUCCESS == err);
    /* Since there is nothing else to do, block until Callback triggers*/
    //while ((I2C_MASTER_EVENT_TX_COMPLETE != i2c_event) && timeout_ms)
    while ((I2C_MASTER_EVENT_TX_COMPLETE != i2c_event) && timeout_ms>0)
    {
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MICROSECONDS);
        timeout_ms--;
        }
    if (I2C_MASTER_EVENT_ABORTED == i2c_event)
    {
        __BKPT(0);
    }
    /* Read data back from the I2C slave */
    i2c_event = I2C_MASTER_EVENT_ABORTED;
    timeout_ms           = 100000;
    return 0;
}


/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
    R_SCI_I2C_SlaveAddressSet(&g_i2c2_ctrl, 0x6B, I2C_MASTER_ADDR_MODE_7BIT);
    assert(FSP_SUCCESS == err);
    err = R_SCI_I2C_Write(&g_i2c2_ctrl, &reg, 1, true);
    assert(FSP_SUCCESS == err);
    /* Since there is nothing else to do, block until Callback triggers*/
    //while ((I2C_MASTER_EVENT_TX_COMPLETE != i2c_event) && timeout_ms)
    while ((I2C_MASTER_EVENT_TX_COMPLETE != i2c_event) && timeout_ms>0)
    {
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MICROSECONDS);
        timeout_ms--;
        }
    if (I2C_MASTER_EVENT_ABORTED == i2c_event)
    {
        __BKPT(0);
        }
    /* Read data back from the I2C slave */
    i2c_event = I2C_MASTER_EVENT_ABORTED;
    timeout_ms           = 100000;

    /* Read data from I2C slave */
    err = R_SCI_I2C_Read(&g_i2c2_ctrl, bufp, len, false);
    assert(FSP_SUCCESS == err);
    while ((I2C_MASTER_EVENT_RX_COMPLETE != i2c_event) && timeout_ms)
    {
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
        timeout_ms--;
    }
    if (I2C_MASTER_EVENT_ABORTED == i2c_event)
    {
        __BKPT(0);
    }

    i2c_event = I2C_MASTER_EVENT_ABORTED;
    timeout_ms           = 100000;
  return 0;
}


/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
    R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}


/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        R_IOPORT_Open (&IOPORT_CFG_CTRL, &IOPORT_CFG_NAME);

#if BSP_CFG_SDRAM_ENABLED

        /* Setup SDRAM and initialize it. Must configure pins first. */
        R_BSP_SdramInit(true);
#endif
    }
}

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
FSP_CPP_FOOTER

#endif
