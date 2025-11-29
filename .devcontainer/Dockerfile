FROM espressif/idf:release-v5.4

# 這行是關鍵！自動安裝 udev 以解決 "spawn udevadm ENOENT" 錯誤
# 同時安裝 usbutils 讓你可以用 lsusb 指令檢查
RUN apt-get update && apt-get install -y \
    udev \
    usbutils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 設定環境變數 (確保工具路徑正確)
ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp

# 進入點
ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]
CMD [ "/bin/bash" ]