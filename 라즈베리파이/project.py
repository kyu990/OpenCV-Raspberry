# 센서, OpenCV 카메라(xml) 병합 코드
import RPi.GPIO as GPIO
import spidev
import Adafruit_BMP.BMP085 as BMP085
from time import sleep
import time
import json
import cv2
import threading
import base64
import websocket
from queue import Queue

q = Queue(maxsize=30)  # 큐 구조 생성

flame = 17  # GPIO 불꽃감지센서 핀번호

sensor = BMP085.BMP085(busnum=1)  # BMP bus번호 1 지정

GPIO.setmode(GPIO.BCM)
GPIO.setup(flame, GPIO.IN)

spi = spidev.SpiDev()  # MCP3008 사용을 위한 spi 제어
spi.open(0, 0)
spi.max_speed_hz = 1000000  # SPI 통신 속도 설정

# xml 경로 설정. 리눅스 기반은 절대경로
fire_cascade = cv2.CascadeClassifier("/home/pi/webapps/project/guna.xml")


# 아날로그 통신 값 계산 (가스센서)
def read_spi_adc(channel):
    buffer = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((buffer[1] & 3) << 8) + buffer[2]
    return data


# 아날로그 통신 값 계산 (조도 센서)
def read_spi_CDS(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data


def send_data_sensor(ws):
    try:
        while True:
            if ws.connected:
                try:
                    now = time.localtime()
                    timestamp = "%04d-%02d-%02d %02d:%02d:%02d" % (
                        now.tm_year,
                        now.tm_mon,
                        now.tm_mday,
                        now.tm_hour,
                        now.tm_min,
                        now.tm_sec,
                    )

                    adcValue = read_spi_adc(0)  # MCP3008 채널 0번 (가스 센서)
                    CDSValue = read_spi_CDS(1)  # MCP3008 채널 1번 (조도 센서)
                    flame_state = GPIO.input(flame)
                    temp = sensor.read_temperature()

                    data_sensor = {
                        "tempSensor": temp,  # 온도
                        "gasSensor": adcValue,  # 가스
                        "lightSensor": CDSValue,  # 조도
                        "fireSensor": flame_state,  # 불꽃감지
                        "time": timestamp,
                    }

                    data_json = json.dumps(data_sensor, indent=4)
                    print(data_json)
                    ws.send(data_json)
                    print("센서 값 전송 완료")
                    sleep(1)

                except Exception as e:
                    print("Error:", e)

    except KeyboardInterrupt:
        spi.close()

    except Exception as e:
        print("Error sending data:", e)


def frames_in_queue():
    global q  # 실시간 프레임 저장을 위한 큐
    cap = cv2.VideoCapture(-1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to read frame from the camera.")
                break

            adcValue = read_spi_adc(0)  # MCP3008 채널 0번 (가스 센서)
            CDSValue = read_spi_CDS(1)  # MCP3008 채널 1번 (조도 센서)
            flame_state = GPIO.input(flame)
            temp = sensor.read_temperature()

            if (
                flame_state == 1
            ):  # 평소에 1을 전송, 화재 감지 시 0으로 되고, 불꽃감지 센서를 포함해 다른 센서의 기준치 초과 시 영상처리 프레임 전송
                _, buffer1 = cv2.imencode(".jpg", frame)
                binary_frame1 = buffer1.tobytes()
                encoded_frame1 = base64.b64encode(binary_frame1).decode("utf-8")
                q.put(encoded_frame1)

            elif flame_state == 0 and CDSValue < 100:
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame_gaussian = cv2.GaussianBlur(frame_gray, (0, 0), 3)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
                frame_close = cv2.morphologyEx(frame_gaussian, cv2.MORPH_CLOSE, kernel)

                # 화재 감지
                fires = fire_cascade.detectMultiScale(
                    frame_close, scaleFactor=1.1, minNeighbors=2, minSize=(30, 30)
                )
                for x, y, w, h in fires:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                _, buffer2 = cv2.imencode(".jpg", frame)
                binary_frame2 = buffer2.tobytes()
                encoded_frame2 = base64.b64encode(binary_frame2).decode("utf-8")
                q.put(encoded_frame2)

            elif flame_state == 0 and temp > 40:
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame_gaussian = cv2.GaussianBlur(frame_gray, (0, 0), 3)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
                frame_close = cv2.morphologyEx(frame_gaussian, cv2.MORPH_CLOSE, kernel)

                # 화재 감지
                fires = fire_cascade.detectMultiScale(
                    frame_close, scaleFactor=1.1, minNeighbors=2, minSize=(30, 30)
                )
                for x, y, w, h in fires:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                _, buffer2 = cv2.imencode(".jpg", frame)
                binary_frame2 = buffer2.tobytes()
                encoded_frame2 = base64.b64encode(binary_frame2).decode("utf-8")
                q.put(encoded_frame2)

            elif flame_state == 0 and adcValue > 60:

                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame_gaussian = cv2.GaussianBlur(frame_gray, (0, 0), 3)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
                frame_close = cv2.morphologyEx(frame_gaussian, cv2.MORPH_CLOSE, kernel)

                # 화재 감지
                fires = fire_cascade.detectMultiScale(
                    frame_close, scaleFactor=1.1, minNeighbors=2, minSize=(30, 30)
                )
                for x, y, w, h in fires:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                _, buffer2 = cv2.imencode(".jpg", frame)
                binary_frame2 = buffer2.tobytes()
                encoded_frame2 = base64.b64encode(binary_frame2).decode("utf-8")
                q.put(encoded_frame2)

    except cv2.error as e:
        print("OpenCV Error:", e)
    finally:
        cap.release()


def send_frames(ws):
    global q

    try:
        while True:
            # 웹 소켓 통신이 제대로 이루어지고 있는지 확인.
            if ws.connected:
                # q가 비어있지 않을 때, get()을 통해 q의 요소에 접근(첫번째부터)
                if not q.empty():
                    ws.send_binary(q.get())
                    print("Sent a frame to the server")

                time.sleep(1 / 24)

            else:
                print("Error: WebSocket connection closed.")
                break  # 웹소켓 연결이 닫혀 있으면 루프를 종료하고 카메라를 닫는다.

    except Exception as e:
        print("Error sending frames:", e)


def main():
    global q
    while True:
        try:
            ws = websocket.create_connection(
                "ws://192.168.0.205:8080/ws?sessionId=admin"
            )  # 쿼리로 id 보내기
            print("Connected to the server")

            # 스레드를 사용해 영상 데이터 q에 넣기
            queue_thread = threading.Thread(target=frames_in_queue)
            queue_thread.start()

            # 스레드를 사용해 실시간 영상 데이터 전송
            frame_thread = threading.Thread(target=send_frames, args=(ws,))
            frame_thread.start()

            # 스레드를 사용해 실시간 센서 데이터 전송
            sensor_thread = threading.Thread(target=send_data_sensor, args=(ws,))
            sensor_thread.start()

            while True:
                result = ws.recv()
                print("Received: " + result)

        except websocket.WebSocketException as e:
            print("WebSocket Error:", e)
        except Exception as e:
            print("Connection error:", e)

        finally:
            if "ws" in locals() and ws.connected:
                ws.close()
                q.clear()


if __name__ == "__main__":
    main()
