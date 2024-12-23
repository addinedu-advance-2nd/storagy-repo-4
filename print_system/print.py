#pip3 install flask
#pip3 install flask-cors
#pip3 install pycups

import cups
import os
import time
import subprocess

class Printer:
    def __init__(self):
        self.printer_name = "Canon_G7000_series"
        self.conn = cups.Connection()

    def print_file(self,file_path):
        
        # PDF 출력
        msg = self.print_pdf(file_path)    

        # 프린터 상태 확인
        #check_printer_status(PRINTER_NAME)

        return msg


    def print_pdf(self, file_path):
        try:
            # 프린터 목록 확인
            printers = self.conn.getPrinters()
            if self.printer_name not in printers:
                raise Exception(f"프린터 '{self.printer_name}'를 찾을 수 없습니다.")
            
            # PDF 출력 요청
            job_id = self.conn.printFile(self.printer_name, file_path, "Print Job", {})
            print(f"출력 요청 성공! 작업 ID: {job_id}")
            
        # 작업 상태 확인
            while True:
                job_status = self.conn.getJobAttributes(job_id)
                state = job_status['job-state'] #인쇄 상태
                               

                if state == 9:  # 작업 완료 (IPP_JOB_COMPLETED)
                    result = "프린터를 완료하였습니다."
                    print(f"작업 {job_id} 완료!")
                    self.delete_specific_file(file_path) #프린트 완료된 파일 삭제
                    break
                elif state == 8:  # 작업 중단됨 (IPP_JOB_ABORTED)
                    print(f"작업 {job_id}가 중단되었습니다.")
                    result = "프린터를 실행중 중단되었습니다.."
                    self.delete_specific_file(file_path) #프린트 완료된 파일 삭제
                    break
                elif state == 7:  # 작업 취소됨 (IPP_JOB_CANCELLED)
                    print(f"작업 {job_id}가 취소되었습니다.")
                    result = "프린터를 작업이 취소 되었습니다.."
                    self.delete_specific_file(file_path) #프린트 완료된 파일 삭제
                    break
                else:
                    print(f"작업 {job_id} 진행 중... 상태 코드: {state}")    
                     #프린터기의 상태
                    printer_status = printers[self.printer_name]  
                    state_message = printer_status.get('printer-state-message', 'Unknown')
                    state_reasons = printer_status.get('printer-state-reasons', 'Unknown') 

                    print(f"프린터 '{self.printer_name}' 상태: {state_message}")
                    print(f"프린터 '{self.printer_name}' 이유: {state_reasons}")
                    
                    # Check for specific errors
                    if "media-empty" in state_reasons:
                        print("용지가 부족합니다! 프린터에 용지를 채워주세요.")
                        result = "죄송합니다. 용지가 부족합니다! 프린터에 용지를 채워주세요."                       
                        break

                    elif "offline" in state_reasons:
                        print("프린터가 오프라인 상태입니다.")
                        result = "죄송합니다. 프린터가 오프라인 상태입니다."                        
                        break

                    time.sleep(2)
        
        except Exception as e:
            print(f"오류 발생: {e}")
            result = "프린터를 사용 중 오류가 발생하였습니다."
        
        return result

    def check_printer_status(self):
        try:
                 
            # 프린터 상태 확인
            printers = self.conn.getPrinters()
            if self.printer_name in printers:
                printer_status = printers[self.printer_name]
                print(f"프린터 '{self.printer_name}' 상태: {printer_status['printer-state-message']}")
            else:
                print(f"프린터 '{self.printer_name}'를 찾을 수 없습니다.")
        
        except Exception as e:
            print(f"오류 발생: {e}")

    def delete_specific_file(self,file_path):
        try:
            if os.path.isfile(file_path):  # 파일인지 확인
                os.remove(file_path)
                print(f"File '{file_path}' has been deleted.")
            else:
                print(f"File '{file_path}' does not exist.")
        except Exception as e:
            print(f"Error: {e}")
