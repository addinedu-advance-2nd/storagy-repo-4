import discord
from discord.ext import commands
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading  # 스레드 라이브러리
import os
import shutil   #파일 작업 (복사 이동 디렉토리 복제) 라이브러리
import requests   #HTTP 요청 처리 
import asyncio
from control_msgs.msg import RobotRecevieMoving,AlertToChat
import json



intents = discord.Intents.default()
intents.messages = True
intents.members = True
intents.message_content = True    #메세지 내용에 접근 권한 활성화 

bot = commands.Bot(command_prefix="!", intents=intents)

user_requests = {}


# 전역 변수 선언
node = None

TOKEN = ""


def copy_to_local_folder(src_path, dest_folder, user_name):   #지정된 폴더에 복사
    try:
        if not os.path.exists(dest_folder):
            os.makedirs(dest_folder)
        file_name = os.path.basename(src_path)       #복사할 파일의 원본 경로 
        dest_path = os.path.join(dest_folder, file_name)  #파일 복사할 대상 폴더 
        shutil.copy(src_path, dest_path)   #파일 복사 
        return {"message": f"파일이 {dest_folder}로 성공적으로 복사되었습니다.", "dest_path": dest_path}
    except Exception as e:
        return {"error": str(e)}

class DiscordBot(Node):
    def __init__(self):
        super().__init__('discord_bot')
        self.publisher_ = self.create_publisher(RobotRecevieMoving, '/moving_receive', 10)
        self.subscriber_ = self.create_subscription(
            AlertToChat,  # 메시지 타입
            'alert_chat',  # 구독할 토픽 이름
            self.alert_callback,  # 메시지를 처리할 콜백 함수
            10  # 큐 크기
        )
        self.get_logger().info("DiscordBot node has started.")
        #self.timer = self.create_timer(1.0, self.send_command)

    def send_command(self, user_name=None):  # ROS2 노드와 Discord bot 상호 작용. 요청물 로봇 시스템에 전달 사용자 정보 저장
        if user_name is None:
            self.get_logger().warning("user_name이 제공되지 않았습니다. 명령을 발행하지 않습니다.")
            return
      
        msg = RobotRecevieMoving()
        msg.request_system = 'chatbot'
        msg.user_name = user_name      #명령 요청한 사용자 이름 
        #msg.user_id = user_id  
        self.publisher_.publish(msg)

                           # 요청한 사용자 정보 저장
        user_requests[user_name] = user_name
        #self.get_logger().info(f"Publishing command for user: '{user_name}'")
        self.get_logger().info(f"Publishing command : '{msg}'")

    def alert_callback(self, msg):
        self.get_logger().info(f"Received message: {msg}")
        asyncio.run_coroutine_threadsafe(
            self.notify_requester(msg), bot.loop
        )
       
    async def notify_requester(self, message):
        alert_message=message.msg
        user_id=message.user_id
        user_name =message.user_name
    # 요청한 사용자에게만 알림
        for user_id, user_name in user_requests.items():
            
            user = bot.get_user(user_id)  # 사용자 객체 가져오기
            print(user)
            if user:
                try:
                    await user.send(f"🚨 로봇 알림: {alert_message} (요청자: {user_name})")
                except Exception as e:
                    self.get_logger().error(f"사용자 {user_name}에게 알림 전송 실패: {e}")
    
    
    def load_commands():
        global custom_commands
        try:
            with open("commands.json", "r") as file:
                custom_commands = json.load(file)
        except FileNotFoundError:
            custom_commands = {}

def save_commands():
    with open("commands.json", "w") as file:
        json.dump(custom_commands, file, indent=4)    
  
'''
        
async def alert_callback(self, msg):
    alert_message = msg.data  # 수신된 메시지 내용
    # 디스코드 알림 전송
    channel = bot.get_channel(1313015691946889258)  # 디스코드 알림 채널 ID
    if channel:
        await channel.send(f"🚨 로봇 알림: {alert_message}")
        


async def send_alert_to_discord(self, alert_message):
    channel = bot.get_channel(1313015691946889258)
    if channel:
        await channel.send(f"🚨 경고: {alert_message}")
        # 강제 종료 명령 발행
        await wait_command(None)
'''


async def send_to_robot(user_name):  
    node.send_command(user_name)

from discord.ui import Button, View


# @bot.command(name="robote")
# async def robot_command(ctx, *, command=None):
#     if command is None:
#         await ctx.send("사용법: `!robot 명령어` 형태로 명령어를 입력해주세요.")
#         return

#     user = ctx.author  # 디스코드 사용자 정보 가져오기
#     user_id = user.id
#     user_name = user.display_name

#     # ROS로 명령 발행
#     node.send_command(user_id, user_name)
#     await ctx.send(f"{user_name}님의 명령 '{command}'을 로봇으로 전송했습니다.")

@bot.event
async def on_ready():
    print(f"봇이 준비되었습니다. 봇 이름: {bot.user}")
    await bot.change_presence(
        status=discord.Status.online,
        activity=discord.Game('명령 대기 중🖨')
    )

@bot.command(name="robot", aliases=["호출", "와", "come", "c","이리와"])
async def robot_command(ctx, *, command=None):
    if command is None:
        await ctx.send("사용법: `!robot 명령어` 형태로 입력해주세요.")
        return
    user = ctx.author    #여기서 디스코드 사용자 정보 갖고 옴
    user_info = f"User: {user.name} ({user.id}), Nickname: {user.display_name}"  #사용자 닉네임
    await ctx.send(f"로봇을 호출 하셨습니다. 잠시만 기다려 주세요")
    await ctx.send(f"You have called a robot. Please wait a moment.")    
    await send_to_robot( user.display_name)
    # await ctx.send(f"로봇 호출 완료!🎢")
    
    
@bot.command(name="back", aliases=["가", "go", "종료"])
async def robot_back(ctx, *, command=None):    
    user = ctx.author    #여기서 디스코드 사용자 정보 갖고 옴
    # user_info = f"User: {user.name} ({user.id}), Nickname: {user.display_name}"  #사용자 닉네임
    # await ctx.send(f"Sending command to robot: {command}")     #command 안쓰면 에러 
    await send_to_robot( user.display_name+"_B")
    await ctx.send(f"로봇을 제자리로 돌려 보냅니다.😊")
    await ctx.send(f"Return the robot to its place.🤖")


'''
@bot.command(name="handle_delay")
async def handle_delay(ctx):
    message = await ctx.send("로봇이 응답하지 않습니다. 다음 중 하나를 선택하세요:\n종료 강제 종료\n대기 대기")
    # 이모지 추가
    
    await message.add_reaction("종료")
    await message.add_reaction("대기")

    def check(reaction, user):
        #return user == ctx.author and str(reaction.emoji) in ["종료", "대기"]
        return user ==  "대기"
        
   
    try:
        #reaction, user = await bot.wait_for("reaction_add", timeout=30.0, check=check)
        user = await bot.wait_for("reaction_add", timeout=30.0, check=check)
        #if str(reaction.emoji) == "종료":
        if check == "종료":
            await ctx.send("강제 종료를 선택했습니다.")
            msg = RobotRecevieMoving()
            msg.request_system = 'chatbot'
            msg.user_name = "shutdown"  # 강제 종료 명령
            node.publisher_.publish(msg)
            await ctx.send("로봇에 강제 종료 명령을 발행했습니다.")
        #elif str(reaction.emoji) == "⏳":
        else:
            await ctx.send("대기를 선택했습니다.")
            msg = RobotRecevieMoving()
            msg.request_system = 'chatbot'
            msg.user_name = "wait"  # 대기 상태 명령
            node.publisher_.publish(msg)
            await ctx.send("로봇을 대기 상태로 전환했습니다.")
    except asyncio.TimeoutError:
        await ctx.send("⏳ 시간 초과: 아무런 선택도 이루어지지 않았습니다.")
'''



@bot.command(name="print", aliases=["프린트", "출력", "인쇄", "p"]) 
async def print_file(ctx):
    if ctx.message.attachments:               #디스코드 메세지에서 첨부 파일 확인 
        attachment = ctx.message.attachments[0]
        file_name = attachment.filename
        file_path = f"/home/ahn/Documents/{file_name}"  
        user_id = ctx.author.id
        user_name = ctx.author.name  
        user_display_name = ctx.author.display_name  
        dest_folder = "/home/ahn/PrinterJobs"  
        server_url = "http://192.168.0.24:5000/request_print"  

        try:
            await attachment.save(file_path)
            print(f"file_path: {file_path}")

            if os.path.exists(file_path):
                await ctx.send(f"파일 '{file_name}'을 저장했습니다. 서버에 요청을 전송합니다.")
                
                copy_result = copy_to_local_folder(file_path, dest_folder, user_name)
                
                if "error" in copy_result:
                    await ctx.send(f"파일 로컬 저장 중 오류 발생: {copy_result['error']}")
                else:
                    user_requests[user_id] = {
                        "user_name": user_name,
                        "display_name": user_display_name,
                        "file_name": file_name,
                        "file_path": file_path,
                        "dest_path": copy_result["dest_path"],
                        "status": "Copied to local folder"
                    }
                    
                    with open(file_path, "rb") as file:
                        files = {"file": (file_name, file, "application/pdf")}
                        data = {"user": user_display_name,"user_id": user_id}
                        try:
                            response = requests.post(server_url, files=files, data=data)
                            if response.status_code == 200:
                                server_response = response.json()
                                await ctx.send(f"서버 요청 성공: {server_response.get('message', '요청을 완료 하였습니다.🖨')}")
                                user_requests[user_id]["status"] = "Sent to server"
                            else:
                                await ctx.send(f"서버 요청 실패: {response.status_code} {response.text}")
                        except Exception as e:
                            await ctx.send(f"서버로 파일 전송 중 오류 발생: {e}")
            else:
                await ctx.send(f"파일 '{file_name}'을 저장하는 데 실패했습니다. 다시 시도해주세요.")
        except Exception as e:
            await ctx.send(f"파일 저장 중 오류 발생: {e}")
    else:
        await ctx.send("파일이 없어 인쇄를 못합니다.")
        await ctx.send("The file is missing, so printing is not possible.")
        
        

# @bot.command(name="requests")
# async def list_requests(ctx):
#     if user_requests:
#         response = "**현재 요청된 파일 목록:**\n"
#         for user_id, info in user_requests.items():
#             response += (
#                 f"- **사용자**: {info['user_name']} (ID: {user_id})\n"
#                 f"  닉네임: {info['display_name']}\n"
#                 f"  파일명: {info['file_name']}\n"
#                 f"  저장 경로: {info['dest_path']}\n"
#                 f"  상태: {info['status']}\n\n"
#             )
#         await ctx.send(response)
#     else:
#         await ctx.send("현재 요청된 파일이 없습니다.")

@bot.command(name="userinfo")
async def user_info(ctx):
    user = ctx.author
    user_info = (
        f"**사용자 정보**\n"
        f"ID: {user.id}\n"
        f"이름: {user.name}\n"
        f"닉네임: {user.display_name}\n"
        f"상태: {user.status}\n"
        f"계정 생성일: {user.created_at}\n"
    )
    await ctx.send(user_info)
    '''
@bot.command(name="wait")
async def wait_command(ctx):
    msg = RobotRecevieMoving()
    msg.request_system = 'chatbot'
    msg.user_name = "wait"  # 대기 상태
    node.publisher_.publish(msg)
    await ctx.send("로봇을 대기 상태로 전환했습니다.")
'''

@bot.command(name="list_requests")  # 저장된 프린트 요청 목록 출력 
async def list_requests(ctx):
    if user_requests:
        response = "**현재 요청된 파일 목록:**\n"
        for user_id, info in user_requests.items():
            response += (
                f"- **사용자**: {info['user_name']} (ID: {user_id})\n"
                f"  닉네임: {info['display_name']}\n"
                f"  파일명: {info['file_name']}\n"
                f"  저장 경로: {info['dest_path']}\n"
                f"  상태: {info['status']}\n\n"
            )
        await ctx.send(response)
    else:
        await ctx.send("현재 요청된 파일이 없습니다.")



async def complete_request(ctx):
    user_id = ctx.author.id
    user_name = ctx.author.name

    if user_id in user_requests:
        user_requests[user_id]["status"] = "Completed"
        file_name = user_requests[user_id]["file_name"]

        await ctx.send(f"✅ {user_name}님의 프린트 작업이 완료되었습니다. (파일명: {file_name})")
        # Store the data to send before deleting the request
        data = {"user": user_name, "user_id": user_id, "msg": "received", "file_name": file_name}
        del user_requests[user_id]  # 완료된 작업 삭제
    else:
        await ctx.send("⚠️ 현재 완료 처리할 프린트 작업이 없습니다.")
        return  # Exit the command if no request is found

    try:
        server_url = "http://192.168.0.24:5000/request_print"  
        # Send the data as JSON to the server
        response = requests.post(server_url, json=data)
        if response.status_code == 200:
            server_response = response.json()
            await ctx.send(f"서버 요청 성공: {server_response.get('message', '저는 제 자리로 가겠습니다.🦸🏼‍♂️')}")
        else:
            await ctx.send(f"서버 요청 실패: {response.status_code} {response.text}")
    except Exception as e:
        await ctx.send(f"서버로 파일 전송 중 오류 발생: {e}")



'''
@bot.command(name="cancel", aliases=["취소"])
async def cancel_request(ctx):
    user_id = ctx.author.id
    user_name=ctx.author.display_name  
    if user_id in user_requests:
        # 요청 취소 처리
        canceled_request = user_requests.pop(user_id)  # 요청 데이터 삭제
       

    else:
        await ctx.send("\u26a0 요청이 존재하지 않아 취소할 수 없습니다.")

    try:
        server_url = "http://192.168.0.24:5000/request_print"  
        data = {"user": user_name ,"user_id": user_id,"msg":"cancle"}
        response = requests.post(server_url, data= data )
        if response.status_code == 200:
            server_response = response.json()
            await ctx.send(f"서버 요청 성공: {server_response.get('message', '취소 완료.🙆🏼')}")
            user_requests[user_id]["status"] = "Sent to server"
        else:
            await ctx.send(f"서버 요청 실패: {response.status_code} {response.text}")
    except Exception as e:
        await ctx.send(f"서버로 파일 전송 중 오류 발생: {e}")

 
       '''


@bot.command(name="list")
async def list(ctx):
    if not custom_commands:
        await ctx.send("명령어 등록 해주세요.")
        return
    response = "**등록된 명령어:**\n" + "\n".join(f"!{cmd}: {resp}" for cmd, resp in custom_commands.items())
    await ctx.send(response)

custom_commands = {
    "p": "프린터 요청" '\n Print request',
    "c": "호출"'\n call',
    "back": "로봇 제자리"'\n robot in place'
    
    # 필요한 다른 명령어 추가
}

'''
# 모든 메시지 처리 (동적 명령어 실행)
@bot.event
async def on_message(message):
    if message.author == bot.user:
        return

    # 사용자 정의 명령어 처리
    if message.content.startswith("!"):
        command_name = message.content[1:]
        if command_name in custom_commands:
            await message.channel.send(custom_commands[command_name])
            return  # 사용자 정의 명령어 처리 후 종료

    # 기본 명령어 처리
    await bot.process_commands(message)

   '''
# ROS 2 노드 스레드 실행 함수
def ros_spin_thread():
    rclpy.init()
    global node  # 전역 변수를 사용한다고 명시
    node = DiscordBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ROS 2 노드가 종료되었습니다.")
    finally:
        rclpy.shutdown()


# ROS 2 스레드 실행
ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
ros_thread.start()

bot.run(TOKEN)

