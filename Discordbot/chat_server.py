import discord
from discord.ext import commands
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import shutil
import requests
import asyncio
from control_msgs.msg import RobotRecevieMoving
TOKEN = ""

intents = discord.Intents.default()
intents.messages = True
intents.members = True
intents.message_content = True

bot = commands.Bot(command_prefix="!", intents=intents)

user_requests = {}



def copy_to_local_folder(src_path, dest_folder, user_name):
    try:
        if not os.path.exists(dest_folder):
            os.makedirs(dest_folder)
        file_name = os.path.basename(src_path)
        dest_path = os.path.join(dest_folder, file_name)
        shutil.copy(src_path, dest_path)
        return {"message": f"파일이 {dest_folder}로 성공적으로 복사되었습니다.", "dest_path": dest_path}
    except Exception as e:
        return {"error": str(e)}

class DiscordBot(Node):
    def __init__(self):
        super().__init__('discord_bot')
        #self.publisher_ = self.create_publisher(String, 'simple_topic', 10)
        self.publisher_ = self.create_publisher(RobotRecevieMoving, '/moving_receive', 10)  # 주제 이름    
        # self.user_info = user_info  # 사용자 정보 저장
        self.get_logger().info("DiscordBot node has started.")
        self.timer = self.create_timer(1.0, self.send_command)  # 1초 간격으로 호출

    def send_command(self, user_info):
        ##msg = String()
        ##msg.data = f"Command: 'sehyeonkim', {user_info}"  # 사용자 정보 포함
        msg = RobotRecevieMoving()
        msg.request_system = 'chatbot'  # 메시지 내용
        msg.user_name =  user_info # 메시지 내용
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{user_info}'")

rclpy.init()

node = DiscordBot()

async def send_to_robot(command, user_info):
    node.send_command(user_info)


@bot.event
async def on_ready():
    print(f"봇이 준비되었습니다. 봇 이름: {bot.user}")
    await bot.change_presence(
        status=discord.Status.online,
        activity=discord.Game('프린터 준비 중')
    )

@bot.command(name="robot")
async def robot_command(ctx, *, command=None):
    if command is None:
        await ctx.send("사용법: `!robot <command>` 형태로 명령어를 입력해주세요.")
        return
    user = ctx.author
    user_info = f"User: {user.name} ({user.id}), Nickname: {user.display_name}"
    await ctx.send(f"Sending command to robot: {command}")
    await send_to_robot(command, user.display_name)
    await ctx.send(f"명령 '{command}'을 로봇으로 전송했습니다.")



@bot.command(name="print")
async def print_file(ctx):
    if ctx.message.attachments:
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
                        data = {"user": user_display_name}
                        try:
                            response = requests.post(server_url, files=files, data=data)
                            if response.status_code == 200:
                                server_response = response.json()
                                await ctx.send(f"서버 요청 성공: {server_response.get('message', '응답 메시지가 없습니다.')}")
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
        await ctx.send("첨부된 파일이 없습니다. 파일을 첨부해주세요.")

@bot.command(name="requests")
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

bot.run(TOKEN)
