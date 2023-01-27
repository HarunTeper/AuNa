/*!
  \~japanese
  \example get_distance_io.cpp 距離・IOデータを取得する
  \~english
  \example get_distance_io.cpp Obtains distance and IO(input/output) data
  \~

  $Id$
*/

#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"
#include <iostream>
#include <sstream> 
#include <bitset>

using namespace qrk;
using namespace std;


namespace
{
    void print_data(const Urg_driver& urg,
                    const vector<long>& data, const vector<long>& io, long time_stamp)
    {
		cout << "timestamp: " << time_stamp << " [msec]" << endl;

	// \~japanese IO情報を表示
	// \~english Display IO information
		stringstream ss;
		ss << bitset<18>(io[0]);
		string input = ss.str();
		stringstream ss2;
		ss2 << bitset<18>(io[1]);
		string output = ss2.str();
		cout << "input    : " << input  << " (" << io[0] << ")" << endl;
		cout << "output   : " << output << " (" << io[1] << ")" << endl;

    // \~japanese 前方のデータのみを表示
    // \~english Shows only the front step
        int front_index = urg.step2index(0);
        cout << "distance : " << data[front_index] << " [mm]" << endl;
		
        cout << endl;
    }
}


int main(int argc, char *argv[])
{
    Connection_information information(argc, argv);

    // \~japanese 接続
    // \~english Connects to the sensor
    Urg_driver urg;
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type())) {
        cout << "Urg_driver::open(): "
             << information.device_or_ip_name() << ": " << urg.what() << endl;
        return 1;
    }

    // \~japanese データ取得
    // \~english Gets measurement data
#if 1
    // \~japanese データの取得範囲を変更する場合
    // \~english Case where the measurement range (start/end steps) is defined
    urg.set_scanning_parameter(urg.deg2step(-90), urg.deg2step(+90), 0);
#endif
    enum { Capture_times = 10 };
    urg.start_measurement(Urg_driver::Distance_io, Urg_driver::Infinity_times, 0);
    for (int i = 0; i < Capture_times; ++i) {
        vector<long> data;
		vector<long> io;
        long time_stamp = 0;

        if (!urg.get_distance_io(data, io, &time_stamp)) {
            cout << "Urg_driver::get_distance_io(): " << urg.what() << endl;
            return 1;
        }
        print_data(urg, data, io, time_stamp);
    }

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
