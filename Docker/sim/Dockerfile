FROM mcgillrobotics/auv_2025:base

RUN pip3 install --no-cache-dir numpy-quaternion ultralytics \
    && pip install --no-cache-dir --upgrade numpy \
    && pip3 install scikit-learn

RUN rm -f /usr/local/bin/cmake  \
    && sudo apt-get autoremove -y  \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip
