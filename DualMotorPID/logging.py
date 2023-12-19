import serial
import time

# シリアルポートとボーレートを設定
serial_port = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_554323331383519051E0-if00' # 環境に合わせて変更する
baud_rate = 115200 # Arduinoの設定と合わせる

# ファイルに保存する設定
output_file_path = 'arduino_data_log.txt'

# シリアルポートを開く
ser = serial.Serial(serial_port, baud_rate)

# ファイルを開いてデータ Read/Write
with open(output_file_path, 'w') as file:
    while True:
        try:
            line = ser.readline()
            line = line.decode('utf-8').rstrip() # バイト列を文字列に変換して、改行を削除
            print(line)
            file.write(line + '\n')
        except KeyboardInterrupt:
            print("終了します")
            break

# Close serial port
ser.close()
