from flask import Flask, jsonify, request
from flask_cors import CORS  # flask-cors 모듈 가져오기
import os
from print import print_file  # printer 모듈 가져오기
import rclpy
from rclpy.node import Node
from print_node import PrintNode

app = Flask(__name__)
CORS(app)  # 모든 도메인에서의 접근을 허용합니다.

rclpy.init()
node = PrintNode() #ROS 발행

######################################################
# 테스트용 - 미사용 코드
######################################################

# 저장할 디렉토리 설정
UPLOAD_FOLDER = './print_system/request_print'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

@app.route("/request_print", methods=["POST"])
def request_print():
    if 'file' not in request.files:
        return jsonify({"error": "No file part"}), 400

    file = request.files['file']  # 클라이언트에서 업로드한 파일
    user = request.form.get('user')  # 클라이언트에서 프린터 이름을 받음

    if file.filename == '':
        return jsonify({"error": "No selected file"}), 400

    if not user:
         return jsonify({"error": "user is required"}), 400

    # 파일 저장
    file_path = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
    file.save(file_path)  # 파일 저장
    print("file_path"+file_path)
    print("user :"+user)
    try:
        # 프린터에 파일 출력
        #job_id = print_file(user, file_path)  # 프린터 모듈 사용
        
        #발행 - 스토리지 프린터기로 이동 요청
        node.send_move_callback(user)


        # 프린트 하기
        msg = print_file(file_path)  # 프린터 모듈 사용    


        return jsonify({"success": True, "file_path": file_path, "user": user}), 200 

    except ValueError as e:
        print(jsonify({"error": str(e)}), 404)
        msg = "error : 404"
        return jsonify({"error": str(e)}), 404 
    except Exception as e:
        print(jsonify({"error": f"An error occurred: {str(e)}"}), 500)
        msg = "An error occurred: 500"
        return jsonify({"error": f"An error occurred: {str(e)}"}), 500 

if __name__ == "__main__":
     #app.run(host="0.0.0.0", port=5000)  # 서버의 IP 주소로 변경
     app.run(host="192.168.0.24", port=5000)