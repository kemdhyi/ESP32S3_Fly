// 硬件抽象层：电池电压 ADC 实现

#include "hal/hal_adc.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"
#include "esp_log.h"

static const char* TAG = "hal_adc";

static adc_oneshot_unit_handle_t s_unit = nullptr;
static adc_cali_handle_t         s_cali = nullptr;
static adc_channel_t             s_chan = ADC_CHANNEL_0;
static bool                      s_ready = false;

void hal_battery_adc_init(int gpio) {
	adc_unit_t unit = ADC_UNIT_1;
	adc_channel_t chan = ADC_CHANNEL_0;

	esp_err_t err = adc_oneshot_io_to_channel(gpio, &unit, &chan);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "GPIO%d is not a valid ADC pin: %s", gpio, esp_err_to_name(err));
		return;
	}
	s_chan = chan;

	adc_oneshot_unit_init_cfg_t ucfg = {};
	ucfg.unit_id = unit;
	ucfg.ulp_mode = ADC_ULP_MODE_DISABLE;
	err = adc_oneshot_new_unit(&ucfg, &s_unit);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
		return;
	}

	adc_oneshot_chan_cfg_t ccfg = {};
	ccfg.atten    = ADC_ATTEN_DB_12;      // 满量程约 3.1~3.3 V
	ccfg.bitwidth = ADC_BITWIDTH_DEFAULT; // 12 bit
	err = adc_oneshot_config_channel(s_unit, s_chan, &ccfg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
		return;
	}

	// eFuse 曲线拟合校准
	adc_cali_curve_fitting_config_t cfit = {};
	cfit.unit_id  = unit;
	cfit.atten    = ADC_ATTEN_DB_12;
	cfit.bitwidth = ADC_BITWIDTH_DEFAULT;
	if (adc_cali_create_scheme_curve_fitting(&cfit, &s_cali) == ESP_OK) {
		ESP_LOGI(TAG, "ADC%d_CH%d ready on GPIO%d (curve-fitting calibration)",
		         (int)unit + 1, (int)chan, gpio);
	} else {
		s_cali = nullptr;
		ESP_LOGW(TAG, "ADC calibration unavailable, falling back to raw scaling");
	}

	s_ready = true;
}

float hal_battery_read_voltage(int samples, float divider) {
	if (!s_ready) return 0.0f;
	if (samples <= 0) samples = 1;

	int64_t sum = 0;
	for (int i = 0; i < samples; i++) {
		int raw = 0;
		if (adc_oneshot_read(s_unit, s_chan, &raw) != ESP_OK) continue;

		int mv = 0;
		if (s_cali != nullptr) {
			if (adc_cali_raw_to_voltage(s_cali, raw, &mv) != ESP_OK) continue;
		} else {
			mv = raw * 3300 / 4095;   // 退化路径：无校准数据
		}
		sum += mv;
	}

	return ((float)sum / (float)samples / 1000.0f) * divider;
}
