#ifndef NEST_SENSORS_H
#define NEST_SENSORS_H

#include <stdio.h>

#include "nest_adc.h"
#include "nest_realtime.h"
#include "nest_sht.h"

#define SENSOR_UPDATE_RATE_MS 3000

class Sensors {

    public:
        bool has_temp();
        double get_temp();
        bool has_rh();
        double get_rh();
        float get_brightness();
        void post_wifi();
        void update();

    private:
        uint64_t _last_read {0};
        bool _sht_0_valid {false};
        double _sht_0_temp {-1};
        double _sht_0_rh {-1};
        bool _sht_1_valid {false};
        double _sht_1_temp {-1};
        double _sht_1_rh {-1};
        float _brightness_perc {0};
};

extern Sensors Sensors;

#endif // NEST_SENSORS_H
