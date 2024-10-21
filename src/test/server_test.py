import socket

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 5001))  # IP와 포트에 바인딩
    server_socket.listen(1)
    print("Server listening on port 5001")

    while True:
        conn, addr = server_socket.accept()
        print(f"Connected by {addr}")

        while True:
            try:
                # 파일 이름 받기 (구분자를 기준으로 읽기)
                file_name = b""
                while True:
                    chunk = conn.recv(1)
                    if chunk == b'\n':  # 파일 이름과 파일 내용 사이에 '\n'을 구분자로 사용
                        break
                    if not chunk:
                        raise ConnectionResetError  # 연결이 끊어졌다면 예외 발생
                    file_name += chunk

                file_name = file_name.decode('utf-8')  # 파일 이름 디코딩
                print(f"Receiving file: {file_name}")

                # 파일 데이터 받기
                with open(f"received_{file_name}", 'wb') as f:
                    while True:
                        data = conn.recv(1024)
                        if data == b'EOF':  # EOF로 파일 전송 완료 신호를 받음
                            break
                        if not data:  # 클라이언트 연결 끊김 처리
                            raise ConnectionResetError
                        f.write(data)
                print(f"File {file_name} received.")
            
            except ConnectionResetError:
                print("Client disconnected.")
                break

        conn.close()  # 클라이언트 연결을 종료하고 다음 연결 대기

if __name__ == "__main__":
    start_server()
