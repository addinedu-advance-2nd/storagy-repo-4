import numpy as np
import os

# 현재 작업 폴더에 저장
directory = './'  # 현재 폴더

# 더미 데이터 생성
cam_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)  # 카메라 행렬
dist_coef = np.zeros((4, 1), dtype=np.float32)  # 왜곡 계수
r_vector = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # 회전 벡터
t_vector = np.array([0.0, 0.0, 1000.0], dtype=np.float32)  # 변환 벡터

# .npz 파일로 저장
np.savez(os.path.join(directory, "MultiMatrix.npz"), 
         camMatrix=cam_matrix, 
         distCoef=dist_coef, 
         rVector=r_vector, 
         tVector=t_vector)

print("더미 MultiMatrix.npz 파일이 현재 폴더에 생성되었습니다.")

