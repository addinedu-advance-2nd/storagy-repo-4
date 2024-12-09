#pip3 install flask
#pip3 install flask-cors
#pip3 install pycups

import cups
import os
import time
import subprocess

def print_file(file_path):
    PRINTER_NAME = "Canon_G7000_series"  # 프린터 이름
    

    # PDF 출력
    msg = print_pdf(PRINTER_NAME, file_path)    

    # 프린터 상태 확인
    check_printer_status(PRINTER_NAME)

    return msg


def print_pdf(printer_name, file_path):
    try:

        # printer_ip = "192.168.0.161"  # 프린터 IP
        # network_interface = "wlx1cbfce2fc83b"  # 프린터와 연결된 네트워크 인터페이스
        
        # # 필요 시 프린터 IP에 대한 라우팅 설정
       
        # # 프린터에 대한 라우팅 추가
        # subprocess.run(["sudo", "ip", "route", "add", f"{printer_ip}/32", "dev", network_interface], check=True)


        # CUPS 연결
        conn = cups.Connection()
        
        


        # 프린터 목록 확인
        printers = conn.getPrinters()
        if printer_name not in printers:
            raise Exception(f"프린터 '{printer_name}'를 찾을 수 없습니다.")
        
        # PDF 출력 요청
        job_id = conn.printFile(printer_name, file_path, "Print Job", {})
        print(f"출력 요청 성공! 작업 ID: {job_id}")
        
       # 작업 상태 확인
        while True:
            job_status = conn.getJobAttributes(job_id)
            state = job_status['job-state']
            
            if state == 9:  # 작업 완료 (IPP_JOB_COMPLETED)
                result = "프린터를 완료하였습니다."
                print(f"작업 {job_id} 완료!")
                delete_specific_file(file_path) #프린트 완료된 파일 삭제
                break
            elif state == 8:  # 작업 중단됨 (IPP_JOB_ABORTED)
                print(f"작업 {job_id}가 중단되었습니다.")
                result = "프린터를 실행중 중단되었습니다.."
                delete_specific_file(file_path) #프린트 완료된 파일 삭제
                break
            elif state == 7:  # 작업 취소됨 (IPP_JOB_CANCELLED)
                print(f"작업 {job_id}가 취소되었습니다.")
                result = "프린터를 작업이 취소 되었습니다.."
                delete_specific_file(file_path) #프린트 완료된 파일 삭제
                break
            else:
                print(f"작업 {job_id} 진행 중... 상태 코드: {state}")
                time.sleep(2)
    
    except Exception as e:
        print(f"오류 발생: {e}")
        result = "프린터를 사용 중 오류가 발생하였습니다."
    
    return result

def check_printer_status(printer_name):
    try:
        # CUPS 연결
        conn = cups.Connection()
        
        # 프린터 상태 확인
        printers = conn.getPrinters()
        if printer_name in printers:
            printer_status = printers[printer_name]
            print(f"프린터 '{printer_name}' 상태: {printer_status['printer-state-message']}")
        else:
            print(f"프린터 '{printer_name}'를 찾을 수 없습니다.")
    
    except Exception as e:
        print(f"오류 발생: {e}")

def delete_specific_file(file_path):
    try:
        if os.path.isfile(file_path):  # 파일인지 확인
            os.remove(file_path)
            print(f"File '{file_path}' has been deleted.")
        else:
            print(f"File '{file_path}' does not exist.")
    except Exception as e:
        print(f"Error: {e}")
