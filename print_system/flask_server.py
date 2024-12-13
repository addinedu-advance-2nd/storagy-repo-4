from flask import Flask, jsonify, request
from flask_cors import CORS  # flask-cors 모듈 가져오기
import os
import rclpy
from print_node import PrintNode

app = Flask(__name__)
CORS(app)  # 모든 도메인에서의 접근을 허용합니다.

# ROS 초기화
rclpy.init()
node = PrintNode()  # ROS 발행

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
    user_id = request.form.get('user_id')  # 챗봇 사용자 id
  
    if not user:
        return jsonify({"error": "User is required"}), 400

    # 파일 저장
    file_path = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
    file.save(file_path)  # 파일 저장
    print("File saved at:", file_path)
    print("User:", user)
    print("User ID:", user_id)

    try:
        # ROS 발행 - 프린터 노드로 인쇄 요청(+ 스토리지 이동 포함)   
        node.send_request_print_callback(user, file_path)


        return jsonify({"success": True, "file_path": file_path, "user": user}), 200 

    except ValueError as e:
        print(jsonify({"error": str(e)}), 404)
        return jsonify({"error": str(e)}), 404 
    except Exception as e:
        print(jsonify({"error": f"An error occurred: {str(e)}"}), 500)
        return jsonify({"error": f"An error occurred: {str(e)}"}), 500 


@app.route("/request_cancel", methods=["POST"])
def request_print():
    
    user = request.form.get('user')  # 클라이언트에서 프린터 이름을 받음
    
    msg = request.form.get('msg')  # 클라이언트에서 프린터 이름을 받음

    if not user:
        return jsonify({"error": "User is required"}), 400

   
    print("User:", user)

    try:
        

        return jsonify({"success": True, "user": user, "msg": msg}), 200 

    except ValueError as e:
        print(jsonify({"error": str(e)}), 404)
        return jsonify({"error": str(e)}), 404 
    except Exception as e:
        print(jsonify({"error": f"An error occurred: {str(e)}"}), 500)
        return jsonify({"error": f"An error occurred: {str(e)}"}), 500 

@app.route("/request_origin", methods=["POST"])
def request_print():
    
    user = request.form.get('user')  # 클라이언트에서 프린터 이름을 받음
 
    if not user:
        return jsonify({"error": "User is required"}), 400

   
    print("User:", user)

    try:     
        return jsonify({"success": True, "user": user, "msg": msg}), 200 

    except ValueError as e:
        print(jsonify({"error": str(e)}), 404)
        return jsonify({"error": str(e)}), 404 
    except Exception as e:
        print(jsonify({"error": f"An error occurred: {str(e)}"}), 500)
        return jsonify({"error": f"An error occurred: {str(e)}"}), 500 


if __name__ == "__main__":
    app.run(host="192.168.0.24", port=5000)  # 서버의 IP 주소로 변경
