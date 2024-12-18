from flask import Flask, jsonify, request
from flask_cors import CORS  # flask-cors 모듈 가져오기
import os

def create_app(print_node):
    app = Flask(__name__)
    CORS(app)  # 모든 도메인에서의 접근을 허용합니다.

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
        user_id = request.form.get('user_id')  # 챗봇 사용자 ID

        if not user:
            return jsonify({"error": "User is required"}), 400

        # 파일 저장
        file_path = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        file.save(file_path)  # 파일 저장
        print("File saved at:", file_path)
        print("User:", user)
        print("User ID:", user_id)

        try:
            # ROS 발행 - PrintNode로 인쇄 요청
            print_node.send_request_print_callback(user, file_path)

            return jsonify({"success": True, "file_path": file_path, "user": user}), 200

        except ValueError as e:
            return jsonify({"error": str(e)}), 404
        except Exception as e:
            return jsonify({"error": f"An error occurred: {str(e)}"}), 500

    return app
