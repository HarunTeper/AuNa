Refer to this guide:

https://de.mathworks.com/matlabcentral/answers/581567-how-do-i-configure-ros2-for-matlab-r2020a-on-ubuntu-18-04

In terminal, execute:

export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libcurl.so"

sudo matlab

In matlab script, add:

pyenv('Version','/usr/bin/python3.7');

setenv('PATH', strcat('/usr/bin', pathsep, getenv('PATH')));
