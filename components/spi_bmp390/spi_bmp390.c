#include <stdio.h>
#include "spi_bmp390.h"
#include "driver_bmp390.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *BMP = "BMP390";
static const char *TEST = "BMP390 TEST";
static spi_device_handle_t spi;       /**< spi handle */
static bmp390_handle_t handle;        /**< bmp390 handle */
static bool shot_configured = false;  /**< shot configured flag */

static uint8_t spi_init(void) {
    static bool initialized = false;
    if (initialized)
    {
        return BMP390_OK;
    }

    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5 *1000 * 1000,           //Clock out at 5 MHz
        .queue_size = 1,               //We want to be able to queue 7 transactions at a time
        .mode = 0,
        .spics_io_num=PIN_NUM_CS,
        .command_bits = 0,
        .address_bits = 0,
    };

    ret=spi_bus_initialize(BMP390_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE("BMP390", "spi_bus_initialize failed : %s", esp_err_to_name(ret));
        return BMP390_FAIL;
    }

    ret=spi_bus_add_device(BMP390_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(BMP, "spi_bus_add_device failed : %s", esp_err_to_name(ret));
        return BMP390_FAIL;
    }
    initialized = true;
    return BMP390_OK;
}

static uint8_t spi_deinit(void) {
    esp_err_t ret;
    ret=spi_bus_remove_device(spi);
    if (ret != ESP_OK) {
        ESP_LOGE("BMP390", "spi_bus_remove_device failed : %s", esp_err_to_name(ret));
        return BMP390_FAIL;
    }
    ret=spi_bus_free(BMP390_HOST);
    if (ret != ESP_OK) {
        ESP_LOGE("BMP390", "spi_bus_free failed : %s", esp_err_to_name(ret));
        return BMP390_FAIL;
    }
    return BMP390_OK;

}

uint8_t spi_bmp390_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    //Prepare tx / rx buffers
    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];
    tx_buf[0] = reg;
    memset(&tx_buf[1], 0x00, len);
    memset(&rx_buf, 0x00, len + 1);

    // Prepare transaction descriptor
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length =  8 * (len + 1);       //Len is in bytes, transaction length is in bits.
    t.rxlength = 8 * (len + 1);
    t.tx_buffer = &tx_buf;               //Data
    t.rx_buffer = &rx_buf;               //Data

    esp_err_t ret=spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(BMP, "spi_device_polling_transmit failed : %s", esp_err_to_name(ret));
        return BMP390_FAIL;
    }
    memcpy(buf, rx_buf +1 , len);
    return BMP390_OK;
}

uint8_t spi_bmp390_write(uint8_t reg, uint8_t *buf, uint16_t len) {
    //Prepare tx buffer
    uint8_t tx_buf[len + 1];
    tx_buf[0] = reg;
    memcpy(&tx_buf[1], buf, len);

    // Prepare transaction descriptor
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8 *(len + 1);       //Len is in bytes, transaction length is in bits.
    t.tx_buffer = &tx_buf;              //The data is the cmd itself

    esp_err_t ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    if (ret != ESP_OK) {
        ESP_LOGE("BMP390", "spi device transmit failed : %s", esp_err_to_name(ret));
        return BMP390_FAIL;
    }
    return BMP390_OK;
}

static void spi_bmp390_delay_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

static void spi_bmp390_debug_print(const char *const fmt, ...) {
    char str[256];
    va_list args;
    memset((char *)str, 0, sizeof(char)*256); 
    va_start(args, fmt);
    vsnprintf((char *)str, 256, (char const *)fmt, args);
    va_end(args);
    ESP_LOGI("BMP390 Debug", "%s", str);
}

static void spi_bmp390_receive_callback(uint8_t type) {
    ESP_LOGI("BMP390", "bmp390_spi_receive_callback");
    switch (type)
    {
        case BMP390_INTERRUPT_STATUS_FIFO_WATERMARK :
        {
            ESP_LOGE(BMP,"irq fifo watermark.\n");
            break;
        }
        case BMP390_INTERRUPT_STATUS_FIFO_FULL :
        {
            ESP_LOGE(BMP,"irq fifo full.\n");
            break;
        }
        case BMP390_INTERRUPT_STATUS_DATA_READY :
        {
            ESP_LOGE(BMP,"irq data ready.\n");
            break;
        }
        default :
        {
            ESP_LOGE(BMP,"unknown code.\n");
            break;
        }
    }

}

static uint8_t a_spi_bmp390_read(bmp390_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    reg |= 1 << 7;                                                                /* set read mode */
    if (handle->spi_read(reg, handle->buf, 
                            len > 512 ? (512 + 1) : (len + 1)) != 0)                 /* spi read */
    {
        return BMP390_FAIL;                                                                 /* return error */
    }
    memcpy(buf, handle->buf+1, (len > 512) ? 512 : len);                          /* copy data */
    
    return BMP390_OK;                                                                     /* success return 0 */
}

static uint8_t a_spi_bmp390_get_calibration_data(bmp390_handle_t *handle)
{
    uint8_t buf[2];
    
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_T1_L, (uint8_t *)buf, 2) != 0)  /* read t1 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->t1 = (uint16_t)buf[1] <<8 | buf[0];                                          /* set t1 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_T2_L, (uint8_t *)buf, 2) != 0)  /* read t2 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->t2 = (uint16_t)buf[1] << 8 | buf[0];                                         /* set t2 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_T3, (uint8_t *)buf, 1) != 0)    /* read t3 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->t3 = (int8_t)(buf[0]);                                                       /* set t3 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P1_L, (uint8_t *)buf, 2) != 0)  /* read p1 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p1 = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);                              /* set p1 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P2_L, (uint8_t *)buf, 2) != 0)  /* read p2 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p2 = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);                              /* set p2 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P3, (uint8_t *)buf, 1) != 0)    /* read p3 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p3 = (int8_t)(buf[0]);                                                       /* set p3 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P4, (uint8_t *)buf, 1) != 0)    /* read p4 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p4 = (int8_t)(buf[0]);                                                       /* set p4 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P5_L, (uint8_t *)buf, 2) != 0)  /* read p5 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p5 = (uint16_t)buf[1] << 8 | buf[0];                                         /* set p5 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P6_L, (uint8_t *)buf, 2) != 0)  /* read p6l */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p6 = (uint16_t)buf[1] << 8 | buf[0];                                         /* set p6 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P7, (uint8_t *)buf, 1) != 0)    /* read p7 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p7 = (int8_t)(buf[0]);                                                       /* set p7 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P8, (uint8_t *)buf, 1) != 0)    /* read p8 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p8 = (int8_t)(buf[0]);                                                       /* set p8 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P9_L, (uint8_t *)buf, 2) != 0)  /* read p9l */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p9 = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);                              /* set p9 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P10, (uint8_t *)buf, 1) != 0)   /* read p10 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p10 = (int8_t)(buf[0]);                                                      /* set p10 */
    if (a_spi_bmp390_read(handle, BMP390_REG_NVM_PAR_P11, (uint8_t *)buf, 1) != 0)   /* read p11 */
    {
        handle->debug_print("bmp390: get calibration data failed.\n");                   /* get calibration data failed */
       
        return BMP390_FAIL;                                                                        /* return error */
    }
    handle->p11 = (int8_t)(buf[0]);                                                      /* set p11 */

    return BMP390_OK;                                                                            /* success return 0 */
}

static uint8_t a_spi_bmp390_close(bmp390_handle_t *handle)
{
    if (handle->spi_deinit() != 0)                                  /* close spi */
    {
        handle->debug_print("bmp390: spi deinit failed.\n");        /* spi deinit failed */
    
        return BMP390_FAIL;                                                   /* return error */
    }
    else
    {
        return BMP390_OK;                                                   /* success return 0 */
    }
}

static uint8_t a_spi_bmp390_write(bmp390_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    
    reg &= ~(1 << 7);                                                    /* write mode */
    for (i = 0; i < len; i++)                                            /* write data one byte by one byte */
    {
        if (handle->spi_write((uint8_t)(reg + i), buf + i, 1) != 0)      /* spi write */
        {
            return BMP390_FAIL;                                                    /* return error */
        }
    }
    
    return BMP390_OK;                                                            /* success return 0 */
}

esp_err_t spi_bmp390_init() {
    if (handle.inited == true) {
        return ESP_OK;
    }
    
    uint8_t res;
    bmp390_interface_t interface = BMP390_INTERFACE_SPI;
    bmp390_address_t addr_pin = BMP390_ADDRESS_ADO_HIGH;

    // Configure / link handler
    handle.spi_init = spi_init;
    handle.spi_deinit = spi_deinit;
    handle.spi_read = spi_bmp390_read;
    handle.spi_write = spi_bmp390_write;
    handle.delay_ms = spi_bmp390_delay_ms;
    handle.debug_print = spi_bmp390_debug_print;
    handle.receive_callback = spi_bmp390_receive_callback;

     /* set interface */
    res = bmp390_set_interface(&handle, interface);
    if (res != 0)
    {
        ESP_LOGE(BMP, "set interface failed.");
        return ESP_FAIL;
    }

    /* set addr pin */
    res = bmp390_set_addr_pin(&handle, addr_pin);
    if (res != 0)
    {
        ESP_LOGE(BMP, "set addr pin failed.");
        return ESP_FAIL;
    }

    // Test and initialize handler
    uint8_t id;
    uint8_t reg;
    
    // Initialize spi
    if (handle.spi_init() != 0)                                                 /* initialize spi bus */
    {
        ESP_LOGE(BMP, "spi init failed.");        
        return ESP_FAIL;                                                                /* return error */
    }
    
    // test to read chip id
    if (a_spi_bmp390_read(&handle, BMP390_REG_CHIP_ID, (uint8_t *)&id, 1) != 0)   /* read chip id */
    {
        ESP_LOGE(BMP, "read chip id failed.");
        (void)a_spi_bmp390_close(&handle);                                                /* close bmp390 */
        
        return ESP_FAIL;                                                                    /* return error */
    }
    if (id != 0x60)                                                                  /* check chip id */
    {
        ESP_LOGE(BMP, "chip id is not correct.");
        (void)a_spi_bmp390_close(&handle);                                                /* close bmp390 */
        
        return ESP_FAIL;         
    }
    
    // Perform a soft reset on BMP390                                                                                /* return error */
    reg = 0xB6;                                                                      /* set command */
    if (a_spi_bmp390_write(&handle, BMP390_REG_CMD, (uint8_t *)&reg, 1) != 0)     /* write command */
    {
        ESP_LOGE(BMP, "soft reset failed.");
        (void)a_spi_bmp390_close(&handle);                                                /* close bmp390 */
        
        return ESP_FAIL;                                                                    /* return error */
    }

    handle.delay_ms(10);                                                            /* delay 10 ms */
    
    // Test to read error status register
    if (a_spi_bmp390_read(&handle, BMP390_REG_ERR_REG, (uint8_t *)&reg, 1) != 0)  /* read reg */
    {
        ESP_LOGE(BMP, "read error reg failed.");
        (void)a_spi_bmp390_close(&handle);                                                /* close bmp390 */
        
        return ESP_FAIL;                                                                    /* return error */
    }
    if ((reg & 0x07) != 0)                                                           /* check running status */
    {
        ESP_LOGE(BMP, "running status is not correct.");
        (void)a_spi_bmp390_close(&handle);                                                /* close bmp390 */
        
        return ESP_FAIL;                                                                    /* return error */
    }
    if (a_spi_bmp390_get_calibration_data(&handle) != 0)                                  /* get calibration data */
    {
        ESP_LOGE(BMP, "get calibration data failed.");
        (void)a_spi_bmp390_close(&handle);                                                /* close bmp390 */
        
        return ESP_FAIL;                                                                    /* return error */
    }

    handle.inited = 1;  /* flag finish initialization */
    return ESP_OK;
}

esp_err_t spi_bmp390_deinit() {
    uint8_t res = spi_deinit();
    if (res == BMP390_OK) {
        handle.inited = 0;
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t configure_BM390_for_temperature_and_pressure_shot() {
    shot_configured = false;

    if (handle.inited == 0) {
        ESP_LOGE(BMP, "not inited");
        return ESP_FAIL;
    }
    
    uint8_t res;

    /* set default spi wire */
    res = bmp390_set_spi_wire(&handle, BMP390_SHOT_DEFAULT_SPI_WIRE);
    if (res != 0)
    {
        ESP_LOGE(BMP, "set spi wire failed.");        
        return 1;
    }

    /* set default iic watchdog timer */
    res = bmp390_set_iic_watchdog_timer(&handle, BMP390_SHOT_DEFAULT_IIC_WATCHDOG_TIMER);
    if (res != 0)
    {
        ESP_LOGE(BMP, "set iic watchdog timer failed.");        
        return 1;
    }
    
    /* set default iic watchdog period */
    res = bmp390_set_iic_watchdog_period(&handle, BMP390_SHOT_DEFAULT_IIC_WATCHDOG_PERIOD);
    if (res != 0)
    {
        ESP_LOGE(BMP, "set iic watchdog period failed.");        
        return 1;
    }

    /* disable fifo */
    res = bmp390_set_fifo(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(BMP,"set fifo failed.");        
        return ESP_FAIL;
    }
    
    /* set pressure */
    res = bmp390_set_pressure(&handle, SHOT_PRESSURE);
    if (res != 0)
    {
        ESP_LOGE(BMP,"set pressure failed.");
        
        return ESP_FAIL;
    }

    /* set default temperature */
    res = bmp390_set_temperature(&handle, SHOT_TEMPERATURE);
    if (res != 0)
    {
        ESP_LOGE(BMP,"set temperature failed.");        
        return ESP_FAIL;
    }

    /* set default pressure oversampling */
    res = bmp390_set_pressure_oversampling(&handle,  SHOT_PRESSURE_OVERSAMPLING);
    if (res != 0)
    {
        ESP_LOGE(BMP,"set pressure oversampling failed.");
        
        return ESP_FAIL;
    }

    /* set default temperature oversamping */
    res = bmp390_set_temperature_oversampling(&handle, BMP390_SHOT_DEFAULT_TEMPERATURE_OVERSAMPLING);
    if (res != 0)
    {
        ESP_LOGE(BMP,"set temperature oversampling failed.");
        
        return ESP_FAIL;
    }

    /* set default odr */
    res = bmp390_set_odr(&handle, SHOT_ODR);
    if (res != 0)
    {
        ESP_LOGE(BMP,"set odr failed.");        
        return ESP_FAIL;
    }

    /* set filter coefficient */
    res = bmp390_set_filter_coefficient(&handle, SHOT_FILTER_COEFFICIENT);
    if (res != 0)
    {
        ESP_LOGE(BMP,"set filter coefficient failed.");

        return ESP_FAIL;
    }

    /* set forced mode */
    res = bmp390_set_mode(&handle, BMP390_MODE_NORMAL_MODE);
    if (res != 0)
    {
        ESP_LOGE(BMP,"set mode failed.\n");

        return ESP_FAIL;
    }

    shot_configured = true;
    return ESP_OK;

}

esp_err_t bmp390_temperature_and_pressure_shot_read(float *temperature_c, float *pressure_pa)
{
    uint32_t temperature_yaw;
    uint32_t pressure_yaw;
    
    /* read temperature and pressure */
    if (bmp390_read_temperature_pressure(&handle, (uint32_t *)&temperature_yaw, temperature_c,
                                        (uint32_t *)&pressure_yaw, pressure_pa) != 0)
    {
        return ESP_FAIL;
    }
    else
    {
        return ESP_OK;
    }
}

esp_err_t spi_bmp390_test()
{
    uint8_t res;
    bmp390_info_t info;
    bmp390_interface_t interface_test;
    bmp390_address_t addr_pin_test;
    bmp390_interface_t interface = BMP390_INTERFACE_SPI;
    bmp390_address_t addr_pin = BMP390_ADDRESS_ADO_HIGH;
    uint16_t fifo_watermark_in;
    uint16_t fifo_watermark_out;
    bmp390_bool_t enable;
    uint8_t subsampling_in;
    uint8_t subsampling_out;
    uint32_t sensortime;
    bmp390_event_t event;
    uint16_t length;
    uint8_t data;
    uint8_t id;
    uint8_t err;
    uint8_t status;
    bmp390_filter_coefficient_t coef;
    bmp390_oversampling_t oversampling;
    bmp390_odr_t odr;
    bmp390_mode_t mode;
    bmp390_iic_watchdog_period_t period;
    bmp390_interrupt_pin_type_t pin_type;
    bmp390_interrupt_active_level_t level;
    bmp390_fifo_data_source_t source;
    bmp390_spi_wire_t wire;

    

    /* bmp390 init */
    esp_err_t esp_err = spi_bmp390_init();
    if (esp_err != ESP_OK)
    {
        ESP_LOGE(TEST, "init failed.");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "init ok.");
    }
    
    /* bmp390 info */
    res = bmp390_info(&info);
    if (res != 0)
    {
        ESP_LOGE(TEST, "get info failed.");
       
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "chip is %s.", info.chip_name);
        ESP_LOGI(TEST, "interface is %s.", info.interface);
        ESP_LOGI(TEST, "driver version is %d.%d.", info.driver_version/1000, (info.driver_version%1000)/100);
        ESP_LOGI(TEST, "min supply voltage is %0.1fV.", info.supply_voltage_min_v);
        ESP_LOGI(TEST, "max supply voltage is %0.1fV.", info.supply_voltage_max_v);
        ESP_LOGI(TEST, "max current is %0.2fmA.", info.max_current_ma);
        ESP_LOGI(TEST, "temperature is from %0.1fC to %0.1fC.", info.temperature_min, info.temperature_max);
    }

    /* set spi interface */
    res = bmp390_set_interface(&handle, BMP390_INTERFACE_SPI);
    if (res != 0)
    {
        ESP_LOGE(TEST, "set interface failed.");
        return ESP_FAIL;
    }
    res = bmp390_get_interface(&handle, &interface_test);
    if (res != 0)
    {
        ESP_LOGE(TEST, "get interface failed.");
        return ESP_FAIL;
    }
    if (interface_test != BMP390_INTERFACE_SPI)
    {
        ESP_LOGE(TEST, "check set interface spi failed.");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "set interface spi ok.");
    }

    /* set low */
    res = bmp390_set_addr_pin(&handle, BMP390_ADDRESS_ADO_LOW);
    if (res != 0)
    {
        ESP_LOGE(TEST, "set addr pin failed.");
        return ESP_FAIL;
    };
    res = bmp390_get_addr_pin(&handle, &addr_pin_test);
    if (res != 0)
    {
        ESP_LOGE(TEST, "get addr pin failed.");
        return ESP_FAIL;
    }
    if (addr_pin_test != BMP390_ADDRESS_ADO_LOW)
    {
        ESP_LOGE(TEST, "check set addr pin low failed.");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "set addr pin low ok.");
    }

    /* set high */
    res = bmp390_set_addr_pin(&handle, BMP390_ADDRESS_ADO_HIGH);
    if (res != 0)
    {
        ESP_LOGE(TEST, "set addr pin failed.");
        return ESP_FAIL;
    }
    res = bmp390_get_addr_pin(&handle, &addr_pin_test);
    if (res != 0)
    {
        ESP_LOGE(TEST, "get addr pin failed.");
        return ESP_FAIL;
    }
    if (addr_pin_test != BMP390_ADDRESS_ADO_HIGH)
    {
        ESP_LOGE(TEST, "check set addr pin high failed.");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "check set addr pin high ok.");
    }

     /* set interface */
    res = bmp390_set_interface(&handle, interface);
    if (res != 0)
    {
        ESP_LOGE(TEST, "set interface failed.");
        return ESP_FAIL;
    }

    /* set addr pin */
    res = bmp390_set_addr_pin(&handle, addr_pin);
    if (res != 0)
    {
        ESP_LOGE(TEST, "set addr pin failed.");
        return ESP_FAIL;
    }


    /* set fifo watermark test*/
    fifo_watermark_in = rand()%256 + 256;
    res = bmp390_set_fifo_watermark(&handle, fifo_watermark_in);
    if (res != 0)
    {
        ESP_LOGE(TEST, "bmp390: set fifo watermark failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    }
    
    /* get fifo watermark test*/
    res = bmp390_get_fifo_watermark(&handle, (uint16_t *)&fifo_watermark_out);
    if (res != 0)
    {
        ESP_LOGE(TEST, "get fifo watermark failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    }

    /* fifo watermark test in & out is same*/
    if (fifo_watermark_in != fifo_watermark_out)
    {
        ESP_LOGE(TEST, "check fifo watermark failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "check fifo watermark ok.");
    }


    /* enable / disable  */
    res = bmp390_set_fifo(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST, "set fifo failed");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    }
    res = bmp390_get_fifo(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get fifo failed.\n");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    if (enable != BMP390_BOOL_TRUE)
    {
        ESP_LOGE(TEST, "check set fifo enable failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "check set fifo enable ok.");
    }

    /* Sensortime */
    res = bmp390_set_fifo_sensortime_on(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set fifo sensortime on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_fifo_sensortime_on(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get fifo sensortime on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    if (enable != BMP390_BOOL_FALSE)
    {
        ESP_LOGE(TEST, "check set fifo sensortime on disable failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "check set fifo sensortime on disable ok.");
    }
    
    /* Pressure*/
    /* enable */
    res = bmp390_set_fifo_pressure_on(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set fifo pressure on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_fifo_pressure_on(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get fifo pressure on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    if (enable != BMP390_BOOL_TRUE)
    {
        ESP_LOGE(TEST, "check set fifo pressure on enable failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "check set fifo pressure on enable ok.");
    }    
    /* disable */
    res = bmp390_set_fifo_pressure_on(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set fifo pressure on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_fifo_pressure_on(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get fifo pressure on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    if (enable != BMP390_BOOL_FALSE)
    {
        ESP_LOGE(TEST, "check set fifo pressure on disable failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "check set fifo pressure on disable ok.");
    }

    /* temperature on test */
    /* enable */
    res = bmp390_set_fifo_temperature_on(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST, "set fifo temperature on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_fifo_temperature_on(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST, "get fifo temperature on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    if (enable != BMP390_BOOL_TRUE)
    {
        ESP_LOGE(TEST, "check set fifo temperature on enable failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "check set fifo temperature on enable ok.");
    }
    
    /* disable */
    res = bmp390_set_fifo_temperature_on(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST, "set fifo temperature on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    
    res = bmp390_get_fifo_temperature_on(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST, "get fifo temperature on failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    if (enable != BMP390_BOOL_FALSE)
    {
        ESP_LOGE(TEST, "check set fifo temperature on disable failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "check set fifo temperature on disable ok.");
    }

    /* Subsampling */
    subsampling_in = rand()%7;
    res = bmp390_set_fifo_subsampling(&handle, subsampling_in);
    if (res != 0)
    {
        ESP_LOGE(TEST, "set fifo subsampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_fifo_subsampling(&handle, (uint8_t *)&subsampling_out);
    if (res != 0)
    {
        ESP_LOGE(TEST, "get fifo subsampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    if (subsampling_in != subsampling_out)
    {
        ESP_LOGE(TEST, "check set fifo subsampling failed.");
        (void)bmp390_deinit(&handle);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TEST, "check set fifo subsampling ok.");
    }

    /* fifo data source test */    
    /* set unfiltered */
    res = bmp390_set_fifo_data_source(&handle, BMP390_FIFO_DATA_SOURCE_UNFILTERED);
    if (res != 0)
    {
        ESP_LOGE(TEST," set fifo data source failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_fifo_data_source(&handle, (bmp390_fifo_data_source_t *)&source);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get fifo data source failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check fifo data source %s.", source==BMP390_FIFO_DATA_SOURCE_UNFILTERED?"ok":"error");
    
    /* set filtered */
    res = bmp390_set_fifo_data_source(&handle, BMP390_FIFO_DATA_SOURCE_FILTERED);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set fifo data source failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_fifo_data_source(&handle, (bmp390_fifo_data_source_t *)&source);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get fifo data source failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check fifo data source %s.", source==BMP390_FIFO_DATA_SOURCE_FILTERED?"ok":"error");

    /* set & get_interrupt pin type test */
    ESP_LOGI(TEST,"bmp390_set_interrupt_pin_type/bmp390_get_interrupt_pin_type test.");
    
    /* set push pull */
    res = bmp390_set_interrupt_pin_type(&handle, BMP390_INTERRUPT_PIN_TYPE_PUSH_PULL);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt pin type failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_interrupt_pin_type(&handle, (bmp390_interrupt_pin_type_t *)&pin_type);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt pin type failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt pin type %s.", pin_type==BMP390_INTERRUPT_PIN_TYPE_PUSH_PULL?"ok":"error");
    
    /* set open drain */
    res = bmp390_set_interrupt_pin_type(&handle, BMP390_INTERRUPT_PIN_TYPE_OPEN_DRAIN);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt pin type failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_interrupt_pin_type(&handle, (bmp390_interrupt_pin_type_t *)&pin_type);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt pin type failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt pin type %s.", pin_type==BMP390_INTERRUPT_PIN_TYPE_OPEN_DRAIN?"ok":"error");
    
    /* set & get interrupt active level test */
    ESP_LOGI(TEST,"bmp390_set_interrupt_active_level/bmp390_get_interrupt_active_level test.");
    
    /* set lower */
    res = bmp390_set_interrupt_active_level(&handle, BMP390_INTERRUPT_ACTIVE_LEVEL_LOWER);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt active level failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_interrupt_active_level(&handle, (bmp390_interrupt_active_level_t *)&level);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt active level failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt active level %s.", level==BMP390_INTERRUPT_ACTIVE_LEVEL_LOWER?"ok":"error");
    
    /* set higher */
    res = bmp390_set_interrupt_active_level(&handle, BMP390_INTERRUPT_ACTIVE_LEVEL_HIGHER);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt active level failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_interrupt_active_level(&handle, (bmp390_interrupt_active_level_t *)&level);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt active level failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt active level %s.", level==BMP390_INTERRUPT_ACTIVE_LEVEL_HIGHER?"ok":"error");
    
    /* set & get latch interrupt pin and interrupt status test */
    /* enable */
    res = bmp390_set_latch_interrupt_pin_and_interrupt_status(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set latch interrupt pin and interrupt status failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_latch_interrupt_pin_and_interrupt_status(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get latch interrupt pin and interrupt status failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check latch interrupt pin and interrupt status %s.", enable==BMP390_BOOL_TRUE?"ok":"error");
    
    /* disable */
    res = bmp390_set_latch_interrupt_pin_and_interrupt_status(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set latch interrupt pin and interrupt status failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_latch_interrupt_pin_and_interrupt_status(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get latch interrupt pin and interrupt status failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check latch interrupt pin and interrupt status %s.", enable==BMP390_BOOL_FALSE?"ok":"error");
    
    /* set & get interrupt fifo watermark test */
    ESP_LOGI(TEST,"bmp390_set_interrupt_fifo_watermark/bmp390_get_interrupt_fifo_watermark test.");
    
    /* enable */
    res = bmp390_set_interrupt_fifo_watermark(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt fifo watermark failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_interrupt_fifo_watermark(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt fifo watermark failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt fifo watermark %s.", enable==BMP390_BOOL_TRUE?"ok":"error");
    
    /* disable */
    res = bmp390_set_interrupt_fifo_watermark(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt fifo watermark failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_interrupt_fifo_watermark(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt fifo watermark failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt fifo watermark %s.", enable==BMP390_BOOL_FALSE?"ok":"error");
    
    /* set & get interrupt fifo full test */    
    /* enable */
    res = bmp390_set_interrupt_fifo_full(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt fifo full failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"set interrupt fifo full enable.");
    res = bmp390_get_interrupt_fifo_full(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt fifo full failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt fifo full %s.", enable==BMP390_BOOL_TRUE?"ok":"error");
    
    /* disable */
    res = bmp390_set_interrupt_fifo_full(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt fifo full failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"set interrupt fifo full disable.");
    res = bmp390_get_interrupt_fifo_full(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt fifo full failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt fifo full %s.", enable==BMP390_BOOL_FALSE?"ok":"error");
    
    /* set & get interrupt fifo data ready test */    
    /* enable */
    res = bmp390_set_interrupt_data_ready(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt data ready failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_interrupt_data_ready(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt data ready failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt data ready %s.", enable==BMP390_BOOL_TRUE?"ok":"error");
    
    /* disable */
    res = bmp390_set_interrupt_data_ready(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set interrupt data ready failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_interrupt_data_ready(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt data ready failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check interrupt data ready %s.", enable==BMP390_BOOL_FALSE?"ok":"error");
    
    /* set & get spi wire test */
    /* set 4 wire */
    res = bmp390_set_spi_wire(&handle, BMP390_SPI_WIRE_4);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set spi wire failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"set spi 4 wire.");
    res = bmp390_get_spi_wire(&handle, (bmp390_spi_wire_t *)&wire);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get spi wire failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check spi wire %s.", wire==BMP390_SPI_WIRE_4?"ok":"error");
    if (interface == BMP390_INTERFACE_IIC)
    {
        /* set 3 wire */
        res = bmp390_set_spi_wire(&handle, BMP390_SPI_WIRE_3);
        if (res != 0)
        {
            ESP_LOGE(TEST,"set spi wire failed.");
            (void)bmp390_deinit(&handle);
            
            return ESP_FAIL;
        }
        ESP_LOGE(TEST,"set spi 3 wire.");
        res = bmp390_get_spi_wire(&handle, (bmp390_spi_wire_t *)&wire);
        if (res != 0)
        {
            ESP_LOGE(TEST,"get spi wire failed.");
            (void)bmp390_deinit(&handle);
            
            return ESP_FAIL;
        }
        ESP_LOGE(TEST,"check spi wire %s.", wire==BMP390_SPI_WIRE_3?"ok":"error");
    }
    
    /* set & get watchdog timer test */    
    /* enable */
    res = bmp390_set_iic_watchdog_timer(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set watchdog timer failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"set watchdog timer enable.");
    res = bmp390_get_iic_watchdog_timer(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get iic watchdog timer failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check watchdog timer %s.", enable==BMP390_BOOL_TRUE?"ok":"error");
    
    /* disable */
    res = bmp390_set_iic_watchdog_timer(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set watchdog timer failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_iic_watchdog_timer(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get watchdog timer failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check watchdog timer %s.", enable==BMP390_BOOL_FALSE?"ok":"error");
    
    /* set & get watchdog period test */    
    /* set 1.25ms period */
    res = bmp390_set_iic_watchdog_period(&handle, BMP390_IIC_WATCHDOG_PERIOD_1P25_MS);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set watchdog period failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_iic_watchdog_period(&handle, (bmp390_iic_watchdog_period_t *)&period);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get watchdog period failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check watchdog period 1.25ms %s.", period==BMP390_IIC_WATCHDOG_PERIOD_1P25_MS?"ok":"error");
    
    /* set 40ms period */
    res = bmp390_set_iic_watchdog_period(&handle, BMP390_IIC_WATCHDOG_PERIOD_40_MS);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set watchdog period failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_iic_watchdog_period(&handle, (bmp390_iic_watchdog_period_t *)&period);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get watchdog period failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check watchdog period 40ms %s.", period==BMP390_IIC_WATCHDOG_PERIOD_40_MS?"ok":"error");
    
    /*set & get pressure test */    
    /* disable */
    res = bmp390_set_pressure(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set pressure failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"set pressure disable.");
    res = bmp390_get_pressure(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get pressure failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check pressure not enabled %s.", enable==BMP390_BOOL_FALSE?"ok":"error");
    
    /* enable */
    res = bmp390_set_pressure(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set pressure failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_pressure(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get pressure failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check pressure enabled %s.", enable==BMP390_BOOL_TRUE?"ok":"error");
    
    /* set & get temperature test */
    /* disable */
    res = bmp390_set_temperature(&handle, BMP390_BOOL_FALSE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set temperature failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"set temperature disable.");
    res = bmp390_get_temperature(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get temperature failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check temperature not enabled %s.", enable==BMP390_BOOL_FALSE?"ok":"error");
    
    /* enable */
    res = bmp390_set_temperature(&handle, BMP390_BOOL_TRUE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set temperature failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_temperature(&handle, (bmp390_bool_t *)&enable);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get temperature failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check temperature enabled %s.", enable==BMP390_BOOL_TRUE?"ok":"error");
    
    /* set & get mode test */    
    /* sleep */
    res = bmp390_set_mode(&handle, BMP390_MODE_SLEEP_MODE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set mode failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_mode(&handle, (bmp390_mode_t *)&mode);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get mode failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check sleep mode %s.", mode==BMP390_MODE_SLEEP_MODE?"ok":"error");
    
    /* normal mode */
    res = bmp390_set_mode(&handle, BMP390_MODE_NORMAL_MODE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set mode failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_mode(&handle, (bmp390_mode_t *)&mode);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get mode failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check normal mode %s.", mode==BMP390_MODE_NORMAL_MODE?"ok":"error");
    
    /* set forced mode */
    res = bmp390_set_mode(&handle, BMP390_MODE_SLEEP_MODE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set mode failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    handle.delay_ms(50);
    //spi_bmp390_delay_ms(50);
    res = bmp390_set_mode(&handle, BMP390_MODE_FORCED_MODE);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set mode failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_mode(&handle, (bmp390_mode_t *)&mode);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get mode failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check forced mode %s.", mode==BMP390_MODE_FORCED_MODE?"ok":"error"); 
    
    /* set & get pressure oversampling test */
    /* set oversampling x1 */
    res = bmp390_set_pressure_oversampling(&handle, BMP390_OVERSAMPLING_x1);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_pressure_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check pressure oversampling x1 %s.", oversampling==BMP390_OVERSAMPLING_x1?"ok":"error");
    
    /* set oversampling x2 */
    res = bmp390_set_pressure_oversampling(&handle, BMP390_OVERSAMPLING_x2);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_pressure_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check pressure oversampling x2 %s.", oversampling==BMP390_OVERSAMPLING_x2?"ok":"error");
    
    /* set oversampling x4 */
    res = bmp390_set_pressure_oversampling(&handle, BMP390_OVERSAMPLING_x4);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_pressure_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check pressure oversampling x4 %s.", oversampling==BMP390_OVERSAMPLING_x4?"ok":"error");
    
    /* set oversampling x8 */
    res = bmp390_set_pressure_oversampling(&handle, BMP390_OVERSAMPLING_x8);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_pressure_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check pressure oversampling x8 %s.", oversampling==BMP390_OVERSAMPLING_x8?"ok":"error");
    
    /* set oversampling x16 */
    res = bmp390_set_pressure_oversampling(&handle, BMP390_OVERSAMPLING_x16);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_pressure_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check pressure oversampling x16 %s.", oversampling==BMP390_OVERSAMPLING_x16?"ok":"error");
    
    /* set oversampling x32 */
    res = bmp390_set_pressure_oversampling(&handle, BMP390_OVERSAMPLING_x32);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_pressure_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get pressure oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check pressure oversampling x32 %s.", oversampling==BMP390_OVERSAMPLING_x32?"ok":"error");
    
    /* set & get temperature oversampling test */    
    /* oversampling x1 */
    res = bmp390_set_temperature_oversampling(&handle, BMP390_OVERSAMPLING_x1);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_temperature_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check temperature oversampling x1 %s.", oversampling==BMP390_OVERSAMPLING_x1?"ok":"error");  
    
    /* set oversampling x2 */
    res = bmp390_set_temperature_oversampling(&handle, BMP390_OVERSAMPLING_x2);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_temperature_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check temperature oversampling x2 %s.", oversampling==BMP390_OVERSAMPLING_x2?"ok":"error");
    
    /* set oversampling x4 */
    res = bmp390_set_temperature_oversampling(&handle, BMP390_OVERSAMPLING_x4);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_temperature_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check temperature oversampling x4 %s.", oversampling==BMP390_OVERSAMPLING_x4?"ok":"error");
    
    /* set oversampling x8 */
    res = bmp390_set_temperature_oversampling(&handle, BMP390_OVERSAMPLING_x8);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_temperature_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check temperature oversampling x8 %s.", oversampling==BMP390_OVERSAMPLING_x8?"ok":"error");
    
    /* set oversampling x16 */
    res = bmp390_set_temperature_oversampling(&handle, BMP390_OVERSAMPLING_x16);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_temperature_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check temperature oversampling x16 %s.", oversampling==BMP390_OVERSAMPLING_x16?"ok":"error");
    
    /* set oversampling x32 */
    res = bmp390_set_temperature_oversampling(&handle, BMP390_OVERSAMPLING_x32);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_temperature_oversampling(&handle, (bmp390_oversampling_t *)&oversampling);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get temperature oversampling failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check temperature oversampling x32 %s.", oversampling==BMP390_OVERSAMPLING_x32?"ok":"error");
    
    /* set & get odr test */    
    /* set odr 200Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_200_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"set odr 200Hz.");
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 200Hz %s.", odr==BMP390_ODR_200_HZ?"ok":"error");
    
    /* set odr 100Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_100_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 100Hz %s.", odr==BMP390_ODR_100_HZ?"ok":"error");
    
    /* set odr 50Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_50_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 50Hz %s.", odr==BMP390_ODR_50_HZ?"ok":"error");
    
    /* set odr 25Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_25_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 25Hz %s.", odr==BMP390_ODR_25_HZ?"ok":"error");
    
    /* set odr 12.5Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_12P5_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 12.5Hz %s.", odr==BMP390_ODR_12P5_HZ?"ok":"error");
    
    /* set odr 6.25Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_6P25_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 6.25Hz %s.", odr==BMP390_ODR_6P25_HZ?"ok":"error");
    
    /* set odr 3.1Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_3P1_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 3.1Hz %s.", odr==BMP390_ODR_3P1_HZ?"ok":"error");
    
    /* set odr 1.5Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_1P5_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 1.5Hz %s.", odr==BMP390_ODR_1P5_HZ?"ok":"error");
    
    /* set odr 0.78Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P78_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 0.75Hz %s.", odr==BMP390_ODR_0P78_HZ?"ok":"error");
    
    /* set odr 0.39Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P39_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 0.39Hz %s.", odr==BMP390_ODR_0P39_HZ?"ok":"error");
    
    /* set odr 0.2Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P2_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 0.2Hz %s.", odr==BMP390_ODR_0P2_HZ?"ok":"error");
    
    /* set odr 0.1Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P1_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 0.1Hz %s.", odr==BMP390_ODR_0P1_HZ?"ok":"error");
    
    /* set odr 0.05Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P05_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check odr 0.05Hz %s.", odr==BMP390_ODR_0P05_HZ?"ok":"error");
    
    /* set odr 0.02Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P02_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check get odr 0.02Hz %s.", odr==BMP390_ODR_0P02_HZ?"ok":"error");
    
    /* set odr 0.01Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P01_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check get odr 0.01Hz %s.", odr==BMP390_ODR_0P01_HZ?"ok":"error");
    
    /* set odr 0.006Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P006_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check get odr 0.005Hz %s.", odr==BMP390_ODR_0P006_HZ?"ok":"error");
    
    /* set odr 0.003Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P003_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check get odr 0.003Hz %s.", odr==BMP390_ODR_0P003_HZ?"ok":"error");
    
    /* set odr 0.0015Hz */
    res = bmp390_set_odr(&handle, BMP390_ODR_0P0015_HZ);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_odr(&handle, (bmp390_odr_t *)&odr);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get odr failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check get odr 0.0015Hz %s.", odr==BMP390_ODR_0P0015_HZ?"ok":"error");
    
    /* set & get filter coefficient test */
    /* set coefficient 0 */
    res = bmp390_set_filter_coefficient(&handle, BMP390_FILTER_COEFFICIENT_0);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_filter_coefficient(&handle, (bmp390_filter_coefficient_t *)&coef);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check filter coefficient 0 %s.", coef==BMP390_FILTER_COEFFICIENT_0?"ok":"error");
    
    /* set coefficient 1 */
    res = bmp390_set_filter_coefficient(&handle, BMP390_FILTER_COEFFICIENT_1);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_filter_coefficient(&handle, (bmp390_filter_coefficient_t *)&coef);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check filter coefficient 1 %s.", coef==BMP390_FILTER_COEFFICIENT_1?"ok":"error");
    
    /* set coefficient 3 */
    res = bmp390_set_filter_coefficient(&handle, BMP390_FILTER_COEFFICIENT_3);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_filter_coefficient(&handle, (bmp390_filter_coefficient_t *)&coef);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check filter coefficient 3 %s.", coef==BMP390_FILTER_COEFFICIENT_3?"ok":"error");
    
    /* set coefficient 7 */
    res = bmp390_set_filter_coefficient(&handle, BMP390_FILTER_COEFFICIENT_7);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_filter_coefficient(&handle, (bmp390_filter_coefficient_t *)&coef);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check filter coefficient 7 %s.", coef==BMP390_FILTER_COEFFICIENT_7?"ok":"error");
    
    /* set coefficient 15 */
    res = bmp390_set_filter_coefficient(&handle, BMP390_FILTER_COEFFICIENT_15);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_filter_coefficient(&handle, (bmp390_filter_coefficient_t *)&coef);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check filter coefficient 15 %s.", coef==BMP390_FILTER_COEFFICIENT_15?"ok":"error");
    
    /* set coefficient 31 */
    res = bmp390_set_filter_coefficient(&handle, BMP390_FILTER_COEFFICIENT_31);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_filter_coefficient(&handle, (bmp390_filter_coefficient_t *)&coef);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check filter coefficient 31 %s.", coef==BMP390_FILTER_COEFFICIENT_31?"ok":"error");
    
    /* set coefficient 63 */
    res = bmp390_set_filter_coefficient(&handle, BMP390_FILTER_COEFFICIENT_63);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_filter_coefficient(&handle, (bmp390_filter_coefficient_t *)&coef);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check filter coefficient 63 %s.", coef==BMP390_FILTER_COEFFICIENT_63?"ok":"error");
    
    /* set coefficient 127 */
    res = bmp390_set_filter_coefficient(&handle, BMP390_FILTER_COEFFICIENT_127);
    if (res != 0)
    {
        ESP_LOGE(TEST,"set filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    res = bmp390_get_filter_coefficient(&handle, (bmp390_filter_coefficient_t *)&coef);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get filter coefficient failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"check filter coefficient 127 %s.", coef==BMP390_FILTER_COEFFICIENT_127?"ok":"error");
    
    /* get error */
    ESP_LOGI(TEST,"bmp390_get_error.");
    res = bmp390_get_error(&handle, (uint8_t *)&err);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get err failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }    
    /* bmp390 get status */
    ESP_LOGI(TEST,"bmp390_get_status.");
    res = bmp390_get_status(&handle, (uint8_t *)&status);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get status failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }    
    /* get sensortime */
    res = bmp390_get_sensortime(&handle, (uint32_t *)&sensortime);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get sensortime failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }    
    /* get event */
    res = bmp390_get_event(&handle, &event);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get event failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"event is %s.", event==BMP390_EVENT_NONE?"none":"power up or softrest");
    
    /* get interrupt status */
    res = bmp390_get_interrupt_status(&handle, (uint8_t *)&status);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get interrupt status failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }    
    /* get fifo length */
    res = bmp390_get_fifo_length(&handle, (uint16_t *)&length);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get fifo length failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }    
    /* get fifo data */
    res = bmp390_get_fifo_data(&handle, (uint8_t *)&data, 1);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get fifo data failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }    
    /* flush fifo */
    res = bmp390_flush_fifo(&handle);
    if (res != 0)
    {
        ESP_LOGE(TEST,"flush fifo failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"flush fifo %s.", res?"error":"ok");
    
    /* get revision id */
    res = bmp390_get_revision_id(&handle, &id);
    if (res != 0)
    {
        ESP_LOGE(TEST,"get revision id failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"revision id is 0x%02X.", id);
    
    /* softreset */
    res = bmp390_softreset(&handle);
    if (res != 0)
    {
        ESP_LOGE(TEST,"bmp390 softreset failed.");
        (void)bmp390_deinit(&handle);
        
        return ESP_FAIL;
    }
    ESP_LOGI(TEST,"softreset %s.", res?"error":"ok");
    
    /* finish register test */
    ESP_LOGI(TEST,"finish register test.");
    (void)bmp390_deinit(&handle);
        
    return ESP_OK;
}






