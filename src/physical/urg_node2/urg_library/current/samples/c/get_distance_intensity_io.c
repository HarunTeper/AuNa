/*!
  \~japanese
  \example get_distance_intensity.c 距離・強度・IOデータを取得する
  \~english
  \example get_distance_intensity.c Obtains distance, intensity and IO(input/output) data
  \~

  $Id$
*/

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"
#include <stdio.h>
#include <stdlib.h>


static void print_data(urg_t *urg, long data[], unsigned short intensity[],
                       long io[], long time_stamp)
{
    enum {
        IO_BIT_SIZE = 18,
    };

    printf("timestamp: %ld\n", time_stamp);

    // \~japanese IO情報を表示
    // \~english Display IO information
    unsigned long mask = (int)1 << (IO_BIT_SIZE - 1);
    char in[IO_BIT_SIZE + 1];
    char out[IO_BIT_SIZE + 1];
    int i;
    for (i = 0; i < IO_BIT_SIZE; ++i) {
        in[i] = (mask & io[0] ? '1' : '0');
        out[i] = (mask & io[1] ? '1' : '0');
        mask >>= 1;
    }
    in[IO_BIT_SIZE] = '\0';
    out[IO_BIT_SIZE] = '\0';
    printf("input    : %s (%ld)\n", in, io[0]);
    printf("output   : %s (%ld)\n", out, io[1]);

    // \~japanese 前方のデータのみを表示
    // \~english Shows only the front step
    int front_index = urg_step2index(urg, 0);
    printf("distance : %ld [mm]\n", data[front_index]);
    printf("intensity: %ld [1]\n\n", intensity[front_index]);
}


int main(int argc, char *argv[])
{
    enum {
        CAPTURE_TIMES = 10,
    };
    urg_t urg;
    int max_data_size;
    long *io = malloc(2 * sizeof(long));
    long *data = NULL;
    unsigned short *intensity = NULL;
    long time_stamp;
    int n;
    int i;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    max_data_size = urg_max_data_size(&urg);
    data = (long *)malloc(max_data_size * sizeof(data[0]));
    if (!data) {
        perror("urg_max_index()");
        return 1;
    }
    intensity = malloc(max_data_size * sizeof(intensity[0]));
    if (!intensity) {
        perror("urg_max_index()");
        return 1;
    }

    // \~japanese データ取得
    // \~english Gets measurement data
    urg_start_measurement(&urg, URG_DISTANCE_INTENSITY_IO, URG_SCAN_INFINITY, 0);
    for (i = 0; i < CAPTURE_TIMES; ++i) {
        n = urg_get_distance_intensity_io(&urg, data, intensity, io, &time_stamp);
        if (n <= 0) {
            printf("urg_get_distance_intensity_io: %s\n", urg_error(&urg));
            free(data);
            urg_close(&urg);
            return 1;
        }
        print_data(&urg, data, intensity, io, time_stamp);
    }

    // \~japanese 切断
    // \~english Disconnects
    free(intensity);
    free(data);
    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
