# qrcode 설치 : pip install qrcode

import qrcode

# QR 코드 데이터
url = "https://kyb4188.github.io"

# QR 코드 생성
qr = qrcode.QRCode(
    version=1,  # 크기: 1 ~ 40 (숫자가 클수록 크고 복잡한 QR 코드)
    error_correction=qrcode.constants.ERROR_CORRECT_L,  # 오류 정정 수준
    box_size=10,  # 각 박스의 픽셀 크기
    border=4,  # 경계 크기
)

qr.add_data(url)
qr.make(fit=True)

# QR 코드 이미지 저장
img = qr.make_image(fill_color="black", back_color="white")
img.save("qr_code.png")

print("QR 코드가 'qr_code.png'로 저장되었습니다.")
