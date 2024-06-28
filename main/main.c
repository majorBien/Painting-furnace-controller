#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "ili9340.h"
#include "fontx.h"
#include "bmpfile.h"
#include "decode_jpeg.h"
#include "decode_png.h"
#include "pngle.h"

#include "driver/gpio.h"
#include "tft_operations.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"

#define INTERVAL 400
#define WAIT vTaskDelay(INTERVAL)

static const char *TAG = "MAIN";

// You have to set these CONFIG value using menuconfig.
#if 0
#define CONFIG_WIDTH	240
#define CONFIG_HEIGHT 320
#define CONFIG_TFT_CS_GPIO 5
#define CONFIG_DC_GPIO 26
#define CONFIG_RESET_GPIO 2
#define CONFIG_BL_GPIO 2
#endif


#define DEFAULT_VREF    1100        // Default reference voltage in mV
#define NO_OF_SAMPLES   64          // Number of samples for averaging



//temperature sensor
#define PIN_NUM_MISO 21
#define PIN_NUM_MOSI 22
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   19

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;
static const adc_channel_t channel2 = ADC_CHANNEL_7;
static const adc_atten_t atten = ADC_ATTEN_DB_0;        // Attenuation
static const adc_unit_t unit = ADC_UNIT_1;              // ADC1



float temperature1 = 0;
float temperature2 = 0;
float temperature_sensor = 0;
bool heaters = 0;
bool work = 0;

double prev_temperature1 = 0.0;
double prev_temperature2 = 0.0;
double prev_temperature_sensor = 0.0;
int prev_heaters = 0;
int prev_system = 0;

//temperature sensor functions
void spi_init(spi_device_handle_t *spi) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };


    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));

    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, spi));
}


float max6675_read_temp(spi_device_handle_t spi) {
    uint8_t rx_data[2];
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.rx_buffer = rx_data;

    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));


    uint16_t value = (rx_data[0] << 8) | rx_data[1];


    if (value & 0x4) {
        ESP_LOGE(TAG, "Thermocouple input open");
        return -1.0;
    }


    value >>= 3;
    return value * 0.25;
}









uint32_t scaleXnormX(uint32_t x, uint32_t in_min, uint32_t in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_gpio_pins(bool level) {
    gpio_set_level(5, level);
    gpio_set_level(17, level);
    gpio_set_level(16, level);
    gpio_set_level(4, level);
    gpio_set_level(2, level);
    gpio_set_level(16, level);
}


void Temperature(void *pvParameters)
{

	spi_device_handle_t spi;
	spi_init(&spi);
    while (1) {
        uint32_t adc_reading = 0;
        uint32_t adc_reading2 = 0;

        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;

        // Multisampling for channel 2
               for (int i = 0; i < NO_OF_SAMPLES; i++) {
                   if (unit == ADC_UNIT_1) {
                       adc_reading2 += adc1_get_raw((adc1_channel_t)channel2);
                   } else {
                       int raw;
                       adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_12, &raw);
                       adc_reading2 += raw;
                   }
               }
               adc_reading2 /= NO_OF_SAMPLES;


        // Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        uint32_t voltage2 = esp_adc_cal_raw_to_voltage(adc_reading2, adc_chars);
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        //ESP_LOGI(TAG, "Channel 2 - Raw: %d\tVoltage: %dmV", adc_reading2, voltage2);
        //ESP_LOGI(TAG, "Raw: %d\tVoltage: %dmV", adc_reading, voltage);

        temperature1 = scaleXnormX(adc_reading, 0, 4096, 150, 200);
        ESP_LOGI(TAG, "Temp1 %.1f", temperature1);
        temperature2 = scaleXnormX(adc_reading2, 0, 4096, 150, 200);
        ESP_LOGI(TAG, "Temp2 %.1f", temperature2);

        temperature_sensor = max6675_read_temp(spi);
        if (temperature_sensor != -1.0) {
           ESP_LOGI(TAG, "Current Temperature: %.2f°C", temperature_sensor);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Delay for 1 second
    }




}







void ILI9341(void *pvParameters)
{
	// set font file
	FontxFile fx16G[2];
	FontxFile fx24G[2];
	FontxFile fx32G[2];
	FontxFile fx32L[2];
	InitFontx(fx16G,"/spiffs/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/spiffs/ILGH24XB.FNT",""); // 12x24Dot Gothic
	InitFontx(fx32G,"/spiffs/ILGH32XB.FNT",""); // 16x32Dot Gothic
	InitFontx(fx32L,"/spiffs/LATIN32B.FNT",""); // 16x32Dot Latinc

	FontxFile fx16M[2];
	FontxFile fx24M[2];
	FontxFile fx32M[2];
	InitFontx(fx16M,"/spiffs/ILMH16XB.FNT",""); // 8x16Dot Mincyo
	InitFontx(fx24M,"/spiffs/ILMH24XB.FNT",""); // 12x24Dot Mincyo
	InitFontx(fx32M,"/spiffs/ILMH32XB.FNT",""); // 16x32Dot Mincyo
	
	TFT_t dev;
#if CONFIG_XPT2046_ENABLE_SAME_BUS
	ESP_LOGI(TAG, "Enable Touch Contoller using the same SPI bus as TFT");
	int XPT_MISO_GPIO = CONFIG_XPT_MISO_GPIO;
	int XPT_CS_GPIO = CONFIG_XPT_CS_GPIO;
	int XPT_IRQ_GPIO = CONFIG_XPT_IRQ_GPIO;
	int XPT_SCLK_GPIO = -1;
	int XPT_MOSI_GPIO = -1;
#elif CONFIG_XPT2046_ENABLE_DIFF_BUS
	ESP_LOGI(TAG, "Enable Touch Contoller using the different SPI bus from TFT");
	int XPT_MISO_GPIO = CONFIG_XPT_MISO_GPIO;
	int XPT_CS_GPIO = CONFIG_XPT_CS_GPIO;
	int XPT_IRQ_GPIO = CONFIG_XPT_IRQ_GPIO;
	int XPT_SCLK_GPIO = CONFIG_XPT_SCLK_GPIO;
	int XPT_MOSI_GPIO = CONFIG_XPT_MOSI_GPIO;
#else
	ESP_LOGI(TAG, "Disable Touch Contoller");
	int XPT_MISO_GPIO = -1;
	int XPT_CS_GPIO = -1;
	int XPT_IRQ_GPIO = -1;
	int XPT_SCLK_GPIO = -1;
	int XPT_MOSI_GPIO = -1;
#endif
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_TFT_CS_GPIO, CONFIG_DC_GPIO, 
		CONFIG_RESET_GPIO, CONFIG_BL_GPIO, XPT_MISO_GPIO, XPT_CS_GPIO, XPT_IRQ_GPIO, XPT_SCLK_GPIO, XPT_MOSI_GPIO);

#if CONFIG_ILI9225
	uint16_t model = 0x9225;
#endif
#if CONFIG_ILI9225G
	uint16_t model = 0x9226;
#endif
#if CONFIG_ILI9340
	uint16_t model = 0x9340;
#endif
#if CONFIG_ILI9341
	uint16_t model = 0x9341;
#endif
#if CONFIG_ST7735
	uint16_t model = 0x7735;
#endif
#if CONFIG_ST7796
	uint16_t model = 0x7796;
#endif
	lcdInit(&dev, model, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

#if CONFIG_INVERSION
	ESP_LOGI(TAG, "Enable Display Inversion");
	lcdInversionOn(&dev);
#endif

#if CONFIG_RGB_COLOR
	ESP_LOGI(TAG, "Change BGR filter to RGB filter");
	lcdBGRFilter(&dev);
#endif

#if CONFIG_XPT2046_ENABLE_SAME_BUS || CONFIG_XPT2046_ENABLE_DIFF_BUS
#if CONFIG_XPT_CHECK
	TouchPosition(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
#endif
#endif

#if 0
	// for test
	while(1) {
#if CONFIG_XPT2046_ENABLE_SAME_BUS || CONFIG_XPT2046_ENABLE_DIFF_BUS
		TouchCalibration(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT);
		TouchPenTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
		TouchKeyTest(&dev, fx32G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
		TouchMenuTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
		TouchMoveTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
		TouchIconTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
#endif

		ArrowTest(&dev, fx16G, model, CONFIG_WIDTH, CONFIG_HEIGHT);
		WAIT;
	}
#endif

	while(1) {

#if CONFIG_XPT2046_ENABLE_SAME_BUS || CONFIG_XPT2046_ENABLE_DIFF_BUS
		TouchCalibration(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT);
		TouchPenTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
		TouchKeyTest(&dev, fx32G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
		TouchMenuTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
		TouchMoveTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);

#ifdef ENABLE_PNG
		TouchIconTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
#endif

#endif


			char file[32];
			//strcpy(file, "/images/logo.png");
			//PNGTest(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
			//WAIT;

			if(temperature1==150&&temperature2==150)
			{
			    set_gpio_pins(0);
			    heaters = 0;
			    work = 0;
			}
			else
			{
				work = 1;
				if (temperature_sensor < temperature1) {
				    set_gpio_pins(1);
				    heaters = 1;
				} else if (temperature_sensor > temperature2) {
				    set_gpio_pins(0);
				    heaters = 0;
				}
				else if(temperature1==150&&temperature2==150)
				{
					set_gpio_pins(0);
					heaters = 0;
				}
			}




/*
		MainScreen(&dev, fx16G, model, CONFIG_WIDTH, CONFIG_HEIGHT, temperature1, temperature2, temperature_sensor,heaters);
		vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 1 second
		*/


    if (temperature1 != prev_temperature1 ||
        temperature2 != prev_temperature2 ||
        fabs(temperature_sensor - prev_temperature_sensor) > 1.0 ||
        heaters != prev_heaters || work != prev_system)  {


    MainScreen(&dev, fx16G, model, CONFIG_WIDTH, CONFIG_HEIGHT, temperature1, temperature2, temperature_sensor, heaters,work);


    prev_temperature1 = temperature1;
    prev_temperature2 = temperature2;
    prev_temperature_sensor = temperature_sensor;
    prev_heaters = heaters;
    prev_system = work;
}
    vTaskDelay(pdMS_TO_TICKS(100));

	} // end while

	// never reach here
	vTaskDelete(NULL);
}

static void listSPIFFS(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
	}
	closedir(dir);
}

esp_err_t mountSPIFFS(char * path, char * label, int max_files) {
	esp_vfs_spiffs_conf_t conf = {
		.base_path = path,
		.partition_label = label,
		.max_files = max_files,
		.format_if_mount_failed =true
	};

	// Use settings defined above toinitialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret ==ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret== ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return ret;
	}

#if 0
	ESP_LOGI(TAG, "Performing SPIFFS_check().");
	ret = esp_spiffs_check(conf.partition_label);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
		return ret;
	} else {
			ESP_LOGI(TAG, "SPIFFS_check() successful");
	}
#endif

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(conf.partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG,"Mount %s to %s success", path, label);
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	return ret;
}


void app_main(void)
{

	 	esp_rom_gpio_pad_select_gpio(5);
	    esp_rom_gpio_pad_select_gpio(17);
	    esp_rom_gpio_pad_select_gpio(16);
	    esp_rom_gpio_pad_select_gpio(4);
	    esp_rom_gpio_pad_select_gpio(2);
	    esp_rom_gpio_pad_select_gpio(15);

	    gpio_set_direction(5, GPIO_MODE_OUTPUT);
	    gpio_set_direction(17, GPIO_MODE_OUTPUT);
	    gpio_set_direction(16, GPIO_MODE_OUTPUT);
	    gpio_set_direction(4, GPIO_MODE_OUTPUT);
	    gpio_set_direction(2, GPIO_MODE_OUTPUT);
	    gpio_set_direction(15, GPIO_MODE_OUTPUT);



	// Initialize NVS
	ESP_LOGI(TAG, "Initialize NVS");
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );

	ESP_LOGI(TAG, "Initializing SPIFFS");
	esp_err_t ret;
	ret = mountSPIFFS("/spiffs", "storage0", 10);
	if (ret != ESP_OK) return;
	listSPIFFS("/spiffs/");

	// Image file borrowed from here
	// https://www.flaticon.com/packs/social-media-343
	ret = mountSPIFFS("/icons", "storage1", 10);
	if (ret != ESP_OK) return;
	listSPIFFS("/icons/");

	ret = mountSPIFFS("/images", "storage2", 14);
	if (ret != ESP_OK) return;
	listSPIFFS("/images/");

	xTaskCreate(ILI9341, "ILI9341", 1024*6, NULL, 2, NULL);
	xTaskCreate(Temperature, "Temperature", 1024*2, NULL, 0, NULL);
	 // Configure ADC
	    if (unit == ADC_UNIT_1) {
	        adc1_config_width(ADC_WIDTH_BIT_12);
	        adc1_config_channel_atten(channel, atten);
	    } else {
	        adc2_config_channel_atten((adc2_channel_t)channel, atten);
	    }

	    // Configure ADC for channel 2
	     if (unit == ADC_UNIT_1) {
	         adc1_config_channel_atten(channel2, atten);
	     } else {
	         adc2_config_channel_atten((adc2_channel_t)channel2, atten);
	     }

	    // Characterize ADC
	    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
	    //configure temperature sensor



}
