import bluetooth

def start_bluetooth_server():
    server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    port = 1  # 블루투스 통신에 사용할 포트 번호
    server_socket.bind(("", port))
    server_socket.listen(1)

    print("블루투스 서버 시작. 클라이언트 연결을 기다리는 중...")

    client_socket, client_address = server_socket.accept()
    print("클라이언트와 연결 성공:", client_address)

    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                break

            # 클라이언트로부터 받은 데이터를 처리하는 로직을 구현합니다.
            # 원격 제어 등의 기능을 추가하여 해당 데이터를 사용합니다.
            print("받은 데이터:", data.decode())

            # 클라이언트에게 응답을 보낼 수도 있습니다.
            # client_socket.send("서버에서 보낸 응답 메시지".encode())

    except Exception as e:
        print("오류 발생:", str(e))

    finally:
        client_socket.close()
        server_socket.close()
        print("서버 종료.")

if __name__ == "__main__":
    start_bluetooth_server()