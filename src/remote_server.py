import socket
import time

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 5001))  # IP와 포트에 바인딩
    server_socket.listen(1)
    print("Server listening on port 5001")

    while True:
        conn, addr = server_socket.accept()  # 새로운 클라이언트 연결
        print(f"Connected by {addr}")

        try:
            while True:
                print("Waiting for filename...")
                file_name = b""
                
                while True:
                    chunk = conn.recv(1)
                    if chunk == b'\n':  # 파일 이름의 끝을 알리는 구분자 ('\n')
                        break
                    if not chunk:  # 연결이 끊기면 루프 종료
                        raise ConnectionResetError
                    file_name += chunk

                # 파일 이름 수신 후 처리
                file_name = file_name.decode('utf-8').strip()
                
                if not file_name:  # 파일 이름이 없으면 대기하고 다시 요청
                    print("No filename received, waiting for next input...")
                    time.sleep(1)  # 잠시 대기 후 다시 루프 시작
                    continue  # 연결을 유지하고 다시 파일 이름 수신을 시도

                print(f"Receiving file: {file_name}")

                # 파일 데이터를 수신하여 저장
                with open(f"received_{file_name}", 'wb') as f:
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            print("Connection closed by client.")
                            break
                        if b'EOF' in data:  # 클라이언트에서 전송 완료 신호 확인
                            data = data.replace(b'EOF', b'')  # '\n' 제거
                            f.write(data)  # 남은 데이터를 파일에 작성
                            print("File transfer complete.")
                            break
                        f.write(data)  # 데이터를 파일에 작성

                print(f"File {file_name} received.")

        except ConnectionResetError:
            print("Client disconnected.")

        finally:
            # 연결 종료
            conn.close()
            print(f"Connection with {addr} closed.")

if __name__ == "__main__":
    start_server()
