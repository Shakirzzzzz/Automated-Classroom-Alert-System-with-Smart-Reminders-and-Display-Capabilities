#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi.c"
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "ctype.h"
#include "file_serving.h"
#include "driver/gpio.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include <string.h>
#include <stdio.h>
#include <string.h>
#include "freertos/queue.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_sntp.h"

#include "ds3231.h"
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include "protocol_examples_common.h"
#include "driver/ledc.h"
#include "font8x8_basic.h"
#include <stdlib.h>



// BUzzzzer Ports define
#define BUZZER_PIN GPIO_NUM_23 
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_TIMER LEDC_TIMER_0
#define BUZZER_FREQ_HZ 2000 



#define tag "SSD1306"
extern const char *TAG;
#include <stdio.h>
#include <string.h>
#define ROWS 7
#define COLS 8
#define MAX_LEN 20
SSD1306_t display_dev;
char g_current_day[4];
char g_current_time[6];
#define NUM_TIME_SLOTS 8
char *valid_times[NUM_TIME_SLOTS] = {"8:00", "8:50", "9:40", "10:30", "11:20", "12:10", "13:00", "14:00"};
struct tm current_time;


#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
#define sntp_setoperatingmode esp_sntp_setoperatingmode
#define sntp_setservername esp_sntp_setservername
#define sntp_init esp_sntp_init
#endif

#if CONFIG_SET_CLOCK
	#define NTP_SERVER CONFIG_NTP_SERVER
#endif
#if CONFIG_GET_CLOCK
	#define NTP_SERVER " "
#endif

RTC_DATA_ATTR static int boot_count = 0;


void init_buzzer() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = BUZZER_TIMER,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = BUZZER_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BUZZER_CHANNEL,
        .timer_sel = BUZZER_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "Buzzer initialized");
}

void beep_buzzer(int duration_ms) {
    ESP_LOGI(TAG, "Buzzer ON");
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, 512); // 50% UwU
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
    vTaskDelay(duration_ms / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "Buzzer OFF");
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
}
// Return minutes until next class time or -1 if none found
int get_minutes_to_next_class(int current_hour, int current_min, char *next_time) {
    int current_minutes = current_hour * 60 + current_min;
    int min_diff = 24*60; // Initialized to a large value (24 hours)
    int next_minutes = -1;
    
    //closest upcoming time calculation
    for (int i = 0; i < NUM_TIME_SLOTS; i++) {
        int hour, min;
        sscanf(valid_times[i], "%d:%d", &hour, &min);
        int time_minutes = hour * 60 + min;
        int diff = time_minutes - current_minutes;
        
        // If this time is in the future and closer than our current best
        if (diff > 0 && diff < min_diff) {
            min_diff = diff;
            next_minutes = time_minutes;
            strcpy(next_time, valid_times[i]);
        }
    }
    
    if (next_minutes != -1) {
        return min_diff; // Return minutes until next class
    }
    return -1;
}


void time_sync_notification_cb(struct timeval *tv)
{
	ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
	ESP_LOGI(TAG, "Initializing SNTP");
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	//sntp_setservername(0, "pool.ntp.org");
	ESP_LOGI(TAG, "Your NTP Server is %s", NTP_SERVER);
	sntp_setservername(0, NTP_SERVER);
	sntp_set_time_sync_notification_cb(time_sync_notification_cb);
	sntp_init();
}

static bool obtain_time(void)
{
	ESP_ERROR_CHECK( nvs_flash_init() );
	ESP_ERROR_CHECK( esp_netif_init() );
	ESP_ERROR_CHECK( esp_event_loop_create_default() );

	/* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
	 * Read "Establishing Wi-Fi or Ethernet Connection" section in
	 * examples/protocols/README.md for more information about this function.
	 */
	ESP_ERROR_CHECK(example_connect());
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    

	initialize_sntp();

	// wait for time to be set
	int retry = 0;
	const int retry_count = 10;
	while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
		ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}

	ESP_ERROR_CHECK( example_disconnect() );
	if (retry == retry_count) return false;
	return true;
}


void setClock(void *pvParameters)
{
	// obtain time over NTP
	ESP_LOGI(pcTaskGetName(0), "Connecting to WiFi and getting time over NTP.");
	if(!obtain_time()) {
		ESP_LOGE(pcTaskGetName(0), "Fail to getting time over NTP.");
		while (1) { vTaskDelay(1); }
	}

	// update 'now' variable with current time
	time_t now;
	struct tm timeinfo;
	char strftime_buf[64];
	time(&now);
	now = now + (CONFIG_TIMEZONE*60*60);
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	ESP_LOGI(pcTaskGetName(0), "The current date/time is: %s", strftime_buf);

	// Initialize RTC
	i2c_dev_t dev;
	if (ds3231_init_desc(&dev, I2C_NUM_1, 5, 2) != ESP_OK) {
		ESP_LOGE(pcTaskGetName(0), "Could not init device descriptor.");
		while (1) { vTaskDelay(1); }
	}

	ESP_LOGD(pcTaskGetName(0), "timeinfo.tm_sec=%d",timeinfo.tm_sec);
	ESP_LOGD(pcTaskGetName(0), "timeinfo.tm_min=%d",timeinfo.tm_min);
	ESP_LOGD(pcTaskGetName(0), "timeinfo.tm_hour=%d",timeinfo.tm_hour);
	ESP_LOGD(pcTaskGetName(0), "timeinfo.tm_wday=%d",timeinfo.tm_wday);
	ESP_LOGD(pcTaskGetName(0), "timeinfo.tm_mday=%d",timeinfo.tm_mday);
	ESP_LOGD(pcTaskGetName(0), "timeinfo.tm_mon=%d",timeinfo.tm_mon);
	ESP_LOGD(pcTaskGetName(0), "timeinfo.tm_year=%d",timeinfo.tm_year);

	struct tm time = {
		.tm_year = timeinfo.tm_year + 1900,
		.tm_mon  = timeinfo.tm_mon,  // 0-based
		.tm_mday = timeinfo.tm_mday,
		.tm_hour = timeinfo.tm_hour,
		.tm_min  = timeinfo.tm_min,
		.tm_sec  = timeinfo.tm_sec
	};

	if (ds3231_set_time(&dev, &time) != ESP_OK) {
		ESP_LOGE(pcTaskGetName(0), "Could not set time.");
		while (1) { vTaskDelay(1); }
	}
	ESP_LOGI(pcTaskGetName(0), "Set initial date time done");

	// goto deep sleep
	const int deep_sleep_sec = 10;
	ESP_LOGI(pcTaskGetName(0), "Entering deep sleep for %d seconds", deep_sleep_sec);
	esp_deep_sleep(1000000LL * deep_sleep_sec);
}

void getClock(void *pvParameters)
{
    // Initialize RTC
    i2c_dev_t dev;
    int init_retry_count = 0;
    const int max_init_retries = 5;
    
    // Try to initialize the RTC with retries
    while (init_retry_count < max_init_retries) {
        if (ds3231_init_desc(&dev, I2C_NUM_1, 5, 2) == ESP_OK) {
            ESP_LOGI(pcTaskGetName(0), "Successfully initialized RTC device");
            break;
        }
        
        ESP_LOGE(pcTaskGetName(0), "Could not init device descriptor, retry %d/%d", 
                 init_retry_count + 1, max_init_retries);
        init_retry_count++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    if (init_retry_count >= max_init_retries) {
        ESP_LOGE(pcTaskGetName(0), "Failed to initialize RTC after multiple attempts");
        // Continue anyway -  might recover later // SORRYYYY
    }

    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Variables to track consecutive failures
    int consecutive_failures = 0;
    const int max_consecutive_failures = 10;

    while (1) {
        // float temp = 0;
        struct tm rtcinfo;
        bool success = true;


        if (ds3231_get_time(&dev, &rtcinfo) != ESP_OK) {
            ESP_LOGE(pcTaskGetName(0), "Could not get time, consecutive failures: %d", 
                     ++consecutive_failures);
            success = false;
            
            // If too many failures, try to reinitialize the device
            if (consecutive_failures >= max_consecutive_failures) {
                ESP_LOGE(pcTaskGetName(0), "Too many consecutive failures, reinitializing RTC");
                if (ds3231_init_desc(&dev, I2C_NUM_1, 5, 2) == ESP_OK) {
                    ESP_LOGI(pcTaskGetName(0), "Successfully reinitialized RTC device");
                    consecutive_failures = 0;
                }
            }
        } else {

            current_time = rtcinfo;
            consecutive_failures = 0;
            
            ESP_LOGI(pcTaskGetName(0), "%04d-%02d-%02d %02d:%02d:%02d", 
                rtcinfo.tm_year, rtcinfo.tm_mon + 1,
                rtcinfo.tm_mday, rtcinfo.tm_hour, rtcinfo.tm_min, rtcinfo.tm_sec);
        }
        
        if (!success) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            xLastWakeTime = xTaskGetTickCount(); // Reset reference time
        } else {

            vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
        }
    }
}


const char* getTimetableEntry(int row, int col) {
    static char timetable[ROWS][COLS][MAX_LEN];
    memset(timetable, 0, sizeof(timetable));  

    FILE* f = fopen("/storage/timetable.csv", "r");
    if (f == NULL) {
        return "Failed to open file";
    }

    char buffer[256];
    int r = 0;

    while (fgets(buffer, sizeof(buffer), f) && r < ROWS) {
        int c = 0;
        char* value = strtok(buffer, ", ");  

        while (value && c < COLS) {
            strcpy(timetable[r][c], value);
            c++;
            value = strtok(NULL, ", ");
        }
        r++;
    }

    fclose(f);


    if (row < 0 || row >= ROWS || col < 0 || col >= COLS) {
        return "Invalid index";
    }

    return timetable[row][col];
}

void getCoordinates(char *day, char *time, int *x, int *y) {

    char *days[] = {"MON", "TUE", "WED", "THU", "FRI", "SAT","SUN"};
    char *times[] = {"8:00", "8:50", "9:40", "10:30", "11:20", "12:10", "13:00", "14:00"};

    // Find day index
    for (*x = 0; *x < 7; (*x)++) {
        if (strcmp(days[*x], day) == 0) {
            break;
        }
    }

    // Find time index
    for (*y = 0; *y < 8; (*y)++) {
        if (strcmp(times[*y], time) == 0) {
            break;
        }
    }
}
bool is_valid_time(const char* time) {
    for (int i = 0; i < NUM_TIME_SLOTS; i++) {
        if (strcmp(time, valid_times[i]) == 0) {
            return true;
        }
    }
    return false;
}



void display_no_class(SSD1306_t *dev) {
    ssd1306_clear_screen(dev, false);
    ssd1306_contrast(dev, 0xff);
    

    ssd1306_display_text(dev, 3, " NO CLASS ", 11, false);
    

    ssd1306_display_text(dev, 7, g_current_day, 3, false);
    ssd1306_display_text(dev, 7, g_current_time, 5, true);
}


void display_logo(SSD1306_t *dev) {
    ssd1306_clear_screen(dev, false);

    ssd1306_display_text(dev, 5, " AMU ", 7, true);

    ssd1306_display_text(dev, 7, g_current_time, 5, true);

}


void display_current_class(SSD1306_t *dev, const char* current_class) {
    ssd1306_clear_screen(dev, false);
    ssd1306_contrast(dev, 0xff);
    

    ssd1306_display_text(dev, 0, "CURRENT CLASS", 16, false);

    

    int padding = (16 - strlen(current_class)) / 2;
    if (padding < 0) padding = 0;
    
    char padded_class[17] = "";
    for (int i = 0; i < padding; i++) {
        strcat(padded_class, " ");
    }
    strcat(padded_class, current_class);
    

    ssd1306_display_text(dev, 3, padded_class, strlen(padded_class), false);

    

    ssd1306_display_text(dev, 6, "----------------", 16, false);
    

    char day_time[17];
    snprintf(day_time, sizeof(day_time), "%s  %s", g_current_day, g_current_time);
    ssd1306_display_text(dev, 7, day_time, strlen(day_time), false);
}


void display_next_class(SSD1306_t *dev, const char* next_class_time, const char* next_subject) {
    ssd1306_clear_screen(dev, false);
    ssd1306_contrast(dev, 0xff);
    

    ssd1306_display_text(dev, 0, "    NEXT CLASS    ", 18, false);
    

    char time_str[17];
    snprintf(time_str, sizeof(time_str), "At %s:", next_class_time);
    ssd1306_display_text(dev, 3, time_str, strlen(time_str), false);
    

    int padding = (16 - strlen(next_subject)) / 2;
    if (padding < 0) padding = 0;
    
    char padded_subject[17] = "";
    for (int i = 0; i < padding; i++) {
        strcat(padded_subject, " ");
    }
    strcat(padded_subject, next_subject);
    
    ssd1306_display_text(dev, 5, padded_subject, strlen(padded_subject), false);
    ssd1306_display_text(dev, 6, "----------------", 16, false);

    char day_time[17];
    snprintf(day_time, sizeof(day_time), "%s  %s", g_current_day, g_current_time);
    ssd1306_display_text(dev, 7, day_time, strlen(day_time), false);
}


void update_display(SSD1306_t *dev, const char* current_class, bool show_next_class, const char* next_class_time) {

    int current_hour, current_min;
    sscanf(g_current_time, "%d:%d", &current_hour, &current_min);
    
    // Check if time is after all classes (after 14:00 or before 7:55)(matlab free time outside of classes)
    bool after_hours = false;
    if (current_hour > 14 || current_hour < 7 || (current_hour == 14 && current_min > 0) || (current_hour == 7 && current_min < 55)) {
        after_hours = true;
    }
    
    if (after_hours) {

        display_logo(dev);
    } else if (show_next_class) {

        int x, y;
        getCoordinates(g_current_day, (char*)next_class_time, &x, &y);
        const char* next_subject = getTimetableEntry(x+1, y+1);
        
        if (next_subject == NULL || strlen(next_subject) == 0 || strcmp(next_subject, "x") == 0) {
            next_subject = "Free Period";
        }
        display_next_class(dev, next_class_time, next_subject);
    } else {
        if (current_class == NULL || strlen(current_class) == 0 || strcmp(current_class, "x") == 0) {
            display_no_class(dev);
        } else {
            display_current_class(dev, current_class);
        }
    }
}


void update_display_task(void *pvParameters) {
    SSD1306_t *dev = (SSD1306_t *)pvParameters;
    
    char prev_day[4] = "";
    char prev_time[6] = "";
    const char *current_class = "Unknown";
    int x = -1, y = -1;
    bool valid_coordinates = false;
    bool show_next_class = false;
    char next_class_time[6] = "";
    
    while (1) {

        if (strlen(g_current_day) > 0 && strlen(g_current_time) > 0) {
            int current_hour, current_min;
            sscanf(g_current_time, "%d:%d", &current_hour, &current_min);
            

            char temp_next_time[6];
            int mins_to_next = get_minutes_to_next_class(current_hour, current_min, temp_next_time);
            

            if (mins_to_next == 5) {
                if (strcmp(next_class_time, temp_next_time) != 0) {

                    strcpy(next_class_time, temp_next_time);
                    ESP_LOGI(TAG, "5 minutes before class at %s!", next_class_time);
                    show_next_class = true;
                    //BUZZZZER set for 5 Sec #CHECK IT BRUH #5 CLASS BEEP
                    beep_buzzer(5000);
                }
            } else if (is_valid_time(g_current_time)) {

                if (show_next_class) {

                    ESP_LOGI(TAG, "Class time reached: %s", g_current_time);
                    show_next_class = false;
                    beep_buzzer(5000); // #CLASS BEEP
                }
            } else {
                show_next_class = false;
            }
            if (strcmp(g_current_day, prev_day) != 0 || 
                strcmp(g_current_time, prev_time) != 0) {
                
                ESP_LOGI(TAG, "Time or day changed: %s %s (was %s %s)", 
                         g_current_day, g_current_time, prev_day, prev_time);
                
                getCoordinates(g_current_day, g_current_time, &x, &y);
                valid_coordinates = (x >= 0 && x < 7 && y >= 0 && y < 8);
                
                if (valid_coordinates) {
                    current_class = getTimetableEntry(x+1, y+1);
                    if (current_class == NULL || strlen(current_class) == 0) {
                        current_class = "No class";
                    }
                } else {
                    current_class = "ERROR";
                }
                update_display(dev, current_class, show_next_class, next_class_time);
                strcpy(prev_day, g_current_day);
                strcpy(prev_time, g_current_time);
                
                ESP_LOGI(TAG, "Updated display for %s at %s - %s", 
                        g_current_day, g_current_time, 
                        show_next_class ? "Next class" : "Current class");
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); //CHECKING EVERY SECOND (MIGHT BE TOO FAST) #CHECK
    }
}






void time_simulation_task(void *pvParameters) {
    
    strcpy(g_current_day, "THU");

    strcpy(g_current_time, "7:50");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    


    strcpy(g_current_time, "7:55");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    

    strcpy(g_current_time, "8:00");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    


    strcpy(g_current_time, "8:45");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    


    strcpy(g_current_time, "8:50");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    

    strcpy(g_current_time, "9:35");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    

    strcpy(g_current_time, "9:40");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    

    strcpy(g_current_time, "10:25");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    

    strcpy(g_current_time, "10:30");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    

    strcpy(g_current_time, "11:15");
    vTaskDelay(10000 / portTICK_PERIOD_MS);


    strcpy(g_current_time, "11:20");
    vTaskDelay(10000 / portTICK_PERIOD_MS);


    strcpy(g_current_time, "12:05");
    vTaskDelay(10000 / portTICK_PERIOD_MS);


    strcpy(g_current_time, "12:10");
    vTaskDelay(10000 / portTICK_PERIOD_MS);


    strcpy(g_current_time, "12:55");
    vTaskDelay(10000 / portTICK_PERIOD_MS);



    strcpy(g_current_time, "13:00");
    vTaskDelay(10000 / portTICK_PERIOD_MS);


    strcpy(g_current_time, "14:01");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    
        
    ESP_LOGI(TAG, "Time simulation testing completed");
    vTaskDelete(NULL);
}


void update_rtc_globals(void *pvParameters) {

    char *day_names[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
    
    while (1) {

        struct tm adjusted_time = current_time;
        

        adjusted_time.tm_min += 51; // UTC 5.30 correction
        adjusted_time.tm_mday -= 1; // rtc is one day ahead so correction.

        mktime(&adjusted_time);
        

        if (adjusted_time.tm_wday >= 0 && adjusted_time.tm_wday < 7) {
            strcpy(g_current_day, day_names[adjusted_time.tm_wday]);
        } else {

            if (current_time.tm_wday >= 0 && current_time.tm_wday < 7) {
                strcpy(g_current_day, day_names[current_time.tm_wday]);
                ESP_LOGW(TAG, "Using unadjusted day due to invalid adjusted day");
            } else {

                strcpy(g_current_day, "MON");  // Default to Monday
                ESP_LOGW(TAG, "Invalid day index, defaulting to MON");
            }
        }
        

        snprintf(g_current_time, sizeof(g_current_time), "%d:%02d", 
                 adjusted_time.tm_hour, adjusted_time.tm_min);
        
        ESP_LOGI(TAG, "Updated globals: Day=%s, Time=%s", g_current_day, g_current_time);

        vTaskDelay(1000 / portTICK_PERIOD_MS); //updates the global current_time variable every one second
    }
}

void app_main(void)
{
    strcpy(g_current_day,"MON");
    strcpy(g_current_time, "7:30");
    // Wifi Connection //
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    i2c_config_t i2c_config_rtc = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 5,  // GPIO 5 for SDA (RTC)
        .scl_io_num = 2,  // GPIO 2 for SCL (RTC)
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 50000  // Most RTCs work well at 400kHz
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &i2c_config_rtc));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));

    // Initializing Spiffs Partition //
    const char* base_path = "/storage";

    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/storage",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };
    esp_err_t result = esp_vfs_spiffs_register(&conf);
    if (result != ESP_OK) {
        if (result == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (result == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(result));
        }
        return;
    }   
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(result));
        esp_spiffs_format(conf.partition_label);
        return;
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    // Arbitrary day and time // testing.......................

    int x, y;
    ESP_ERROR_CHECK(example_start_file_server(base_path));
    ESP_LOGI(TAG, "File server started");


    // Get coordinates for the given day and time
    getCoordinates(g_current_day, g_current_time, &x, &y);
    //testing....................
            
        // printf("Entry at (5,2): %s\n", getTimetableEntry(x+1, y+1));
        // sleep(5);

     // Print the timetable entry at the given coordinates
    //*************************************************************** */
    

    ESP_LOGI(tag, "INTERFACE is i2c");
    ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
    ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
    ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
    i2c_master_init(&display_dev, 21, 22, CONFIG_RESET_GPIO);

#if CONFIG_FLIP
    display_dev._flip = true;
    ESP_LOGW(tag, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
    ESP_LOGI(tag, "Panel is 128x64");
    ssd1306_init(&display_dev, 128, 64);
#endif
    const char *initial_class = getTimetableEntry(x+1, y+1);
    update_display(&display_dev, initial_class,false,"");
    xTaskCreate(update_display_task, "update_display", 4096, &display_dev, 5, NULL);
    ESP_LOGI(TAG, "Display monitoring task started");

    init_buzzer();
    // xTaskCreate(time_simulation_task, "time_sim", 2048, NULL, 1, NULL);
    ////////////
    #if CONFIG_SET_CLOCK
	// Set clock & Get clock
	if (boot_count == 1) {
		xTaskCreate(setClock, "setClock", 1024*4, NULL, 2, NULL);
	} else {
		xTaskCreate(getClock, "getClock", 1024*4, NULL, 2, NULL);
	}
#endif

#if CONFIG_GET_CLOCK
	// Get clock
	xTaskCreate(getClock, "getClock", 1024*4, NULL, 2, NULL);
#endif

xTaskCreate(update_rtc_globals, "update_rtc", 2048, NULL, 3, NULL);
// while(1){
// 	ESP_LOGI(TAG, "%04d-%02d-%02d %02d:%02d:%02d", 
// 			current_time.tm_year, current_time.tm_mon + 1,
// 			current_time.tm_mday, current_time.tm_hour, current_time.tm_min, current_time.tm_sec);

// 	vTaskDelay(1000);		
// }


}




    

 


    



